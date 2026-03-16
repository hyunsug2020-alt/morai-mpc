#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESKF (Error-State Kalman Filter) GPS+IMU 융합 노드
=====================================================
논문: 2406.06427v3.pdf - "Error-State Kalman Filter" (Gyubeom Edward Im, 2024)

목표: GPS + IMU 융합으로 정밀한 헤딩 방향 추정

오차 상태 벡터 (15차원):
  δx = [δp(3), δv(3), δθ(3), δbg(3), δba(3)]^T

공칭 상태 (nominal state):
  p  : 위치 (local UTM, 3D)
  v  : 속도 (3D)
  q  : 자세 (쿼터니언, 4D)
  bg : 자이로 바이어스 (3D)
  ba : 가속도계 바이어스 (3D)

측정 업데이트:
  1. GPS 위치     (NavSatFix → UTM)
  2. GPS 속도     (TwistWithCovarianceStamped)
  3. 듀얼 GPS 헤딩 (Float64 from dual_gnss_heading_node)

구독:
  /handsfree/imu              (sensor_msgs/Imu)
  /gnss_rover/fix             (sensor_msgs/NavSatFix)
  /ublox_gps/fix_velocity     (geometry_msgs/TwistWithCovarianceStamped)
  /dual_f9p/heading           (std_msgs/Float64)
  /dual_f9p/heading_valid     (std_msgs/Bool)

발행:
  /odometry/filtered          (nav_msgs/Odometry)   - 기존 status_info 대체
  /eskf/heading_deg           (std_msgs/Float64)    - 정밀 헤딩
  /eskf/covariance_trace      (std_msgs/Float64)    - 추정 불확도
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy,
                        QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float64

try:
    from pyproj import Proj  # type: ignore
    _HAS_PYPROJ = True
except ImportError:
    _HAS_PYPROJ = False


# ============================================================
# 쿼터니언 유틸리티
# ============================================================

def quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """쿼터니언 곱 q1 ⊗ q2, 형식: [qw, qx, qy, qz]"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_norm(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 1e-10 else np.array([1., 0., 0., 0.])


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """쿼터니언 → 3×3 회전 행렬, [qw, qx, qy, qz]"""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-w*z),  2*(x*z+w*y)],
        [2*(x*y+w*z),  1-2*(x*x+z*z),  2*(y*z-w*x)],
        [2*(x*z-w*y),    2*(y*z+w*x),  1-2*(x*x+y*y)],
    ])


def quat_to_yaw(q: np.ndarray) -> float:
    """쿼터니언 → yaw(헤딩) [rad], [qw, qx, qy, qz]"""
    w, x, y, z = q / np.linalg.norm(q)
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def rotvec_to_quat(rv: np.ndarray) -> np.ndarray:
    """소각 회전벡터 → 쿼터니언 [qw, qx, qy, qz]"""
    angle = np.linalg.norm(rv)
    if angle < 1e-10:
        return np.array([1., rv[0]/2, rv[1]/2, rv[2]/2])
    axis = rv / angle
    half = angle / 2.0
    return np.array([math.cos(half),
                     math.sin(half)*axis[0],
                     math.sin(half)*axis[1],
                     math.sin(half)*axis[2]])


def skew(v: np.ndarray) -> np.ndarray:
    """3D 벡터 → 스큐 대칭 행렬"""
    return np.array([
        [0.,    -v[2],  v[1]],
        [v[2],   0.,   -v[0]],
        [-v[1],  v[0],   0.],
    ])


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2*math.pi) - math.pi


# ============================================================
# ESKF 클래스
# ============================================================

class ESKF:
    """
    Error-State Kalman Filter
    논문 Section 5 (Im 2024) 기반 구현

    공칭 상태: p(3), v(3), q(4), bg(3), ba(3)
    오차 상태: δp(3), δv(3), δθ(3), δbg(3), δba(3)  → 15차원
    """

    NX = 15  # 오차 상태 차원

    def __init__(self,
                 sigma_accel:    float = 0.05,   # 가속도계 노이즈 [m/s^2]
                 sigma_gyro:     float = 0.005,  # 자이로 노이즈 [rad/s]
                 sigma_accel_bias: float = 0.001,
                 sigma_gyro_bias:  float = 0.0001,
                 sigma_gps_pos:  float = 0.5,    # GPS 위치 노이즈 [m]
                 sigma_gps_vel:  float = 0.1,    # GPS 속도 노이즈 [m/s]
                 sigma_heading:  float = 0.02,   # 듀얼 GPS 헤딩 노이즈 [rad]
                 gravity:        float = 9.81):
        # 노이즈 파라미터
        self.sigma_accel       = sigma_accel
        self.sigma_gyro        = sigma_gyro
        self.sigma_accel_bias  = sigma_accel_bias
        self.sigma_gyro_bias   = sigma_gyro_bias
        self.sigma_gps_pos     = sigma_gps_pos
        self.sigma_gps_vel     = sigma_gps_vel
        self.sigma_heading     = sigma_heading
        self.g = np.array([0., 0., -gravity])  # NED 기준 중력

        # 공칭 상태 초기화
        self.p  = np.zeros(3)
        self.v  = np.zeros(3)
        self.q  = np.array([1., 0., 0., 0.])  # [qw, qx, qy, qz]
        self.bg = np.zeros(3)
        self.ba = np.zeros(3)

        # 오차 공분산
        self.P = np.eye(self.NX) * 1.0
        self.P[0:3,  0:3]  *= 100.0   # 위치 초기 불확도 크게
        self.P[3:6,  3:6]  *= 1.0
        self.P[6:9,  6:9]  *= 0.1
        self.P[9:12, 9:12] *= 0.01
        self.P[12:15,12:15] *= 0.01

        self.initialized = False

    def init(self, p0: np.ndarray, v0: np.ndarray, yaw0: float) -> None:
        """초기 상태 설정"""
        self.p  = p0.copy()
        self.v  = v0.copy()
        # yaw → 쿼터니언 (roll=0, pitch=0)
        half = yaw0 / 2.0
        self.q  = np.array([math.cos(half), 0., 0., math.sin(half)])
        self.bg = np.zeros(3)
        self.ba = np.zeros(3)
        self.initialized = True

    # ----------------------------------------------------------
    # 예측 단계 (IMU 전파)
    # ----------------------------------------------------------

    def predict(self, accel_m: np.ndarray, omega_m: np.ndarray, dt: float) -> None:
        """
        IMU 측정으로 공칭 상태 전파 + 오차 공분산 전파
        논문 Section 5.2

        accel_m : 가속도계 측정값 [m/s^2] (바디 프레임)
        omega_m : 자이로 측정값 [rad/s]   (바디 프레임)
        dt      : 시간 간격 [s]
        """
        if not self.initialized or dt <= 0 or dt > 1.0:
            return

        # ── 바이어스 보정 ──
        a = accel_m - self.ba   # 보정된 가속도
        w = omega_m - self.bg   # 보정된 각속도

        R = quat_to_rot(self.q)  # 바디 → 월드 회전행렬

        # ── 공칭 상태 전파 ──
        p_new = self.p + self.v * dt + 0.5 * (R @ a + self.g) * dt**2
        v_new = self.v + (R @ a + self.g) * dt
        q_new = quat_norm(quat_mult(self.q, rotvec_to_quat(w * dt)))
        # bg, ba : random walk (변화 없음)

        # ── 오차 상태 Jacobian F (15×15) ──
        # 논문 식 (5.2) 기반
        F = np.eye(self.NX)

        # δp 행: F[0:3, 3:6] = I*dt
        F[0:3, 3:6] = np.eye(3) * dt

        # δv 행: F[3:6, 6:9] = -R*[a]x*dt,  F[3:6, 12:15] = -R*dt
        F[3:6, 6:9]   = -R @ skew(a) * dt
        F[3:6, 12:15] = -R * dt

        # δθ 행: F[6:9, 6:9] = I - [w]x*dt,  F[6:9, 9:12] = -I*dt
        F[6:9, 6:9]  = np.eye(3) - skew(w) * dt
        F[6:9, 9:12] = -np.eye(3) * dt

        # bg, ba : 랜덤 워크 → F 대각 = I (기본값)

        # ── 프로세스 노이즈 공분산 Q ──
        Qc = np.zeros((12, 12))
        Qc[0:3,  0:3]  = np.eye(3) * self.sigma_accel**2
        Qc[3:6,  3:6]  = np.eye(3) * self.sigma_gyro**2
        Qc[6:9,  6:9]  = np.eye(3) * self.sigma_gyro_bias**2
        Qc[9:12, 9:12] = np.eye(3) * self.sigma_accel_bias**2

        # Fw: 노이즈가 상태에 미치는 Jacobian (15×12)
        Fw = np.zeros((self.NX, 12))
        Fw[3:6,  0:3]  = -R          # 가속도 → δv
        Fw[6:9,  3:6]  = -np.eye(3)  # 각속도 → δθ
        Fw[9:12, 6:9]  = np.eye(3)   # 자이로 바이어스
        Fw[12:15,9:12] = np.eye(3)   # 가속도 바이어스

        # ── 공분산 전파 ──
        self.P = F @ self.P @ F.T + Fw @ (Qc * dt) @ Fw.T

        # 상태 업데이트
        self.p = p_new
        self.v = v_new
        self.q = q_new

    # ----------------------------------------------------------
    # 보정 단계 공통 함수
    # ----------------------------------------------------------

    def _correct(self, H: np.ndarray, R_noise: np.ndarray,
                 innovation: np.ndarray) -> None:
        """
        오차 상태 보정 + 공칭 상태 주입 + 리셋
        논문 Section 5.3

        H          : 관측 Jacobian (m×15)
        R_noise    : 측정 노이즈 공분산 (m×m)
        innovation : y = z - h(x) (m×1)
        """
        S = H @ self.P @ H.T + R_noise
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ innovation  # 오차 상태 추정

        # ── 오차 주입 (injection) ──
        self.p  += dx[0:3]
        self.v  += dx[3:6]
        dtheta   = dx[6:9]
        self.q   = quat_norm(quat_mult(self.q, rotvec_to_quat(dtheta)))
        self.bg += dx[9:12]
        self.ba += dx[12:15]

        # ── 공분산 업데이트 ──
        I_KH = np.eye(self.NX) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_noise @ K.T  # Joseph form

        # ── 오차 상태 리셋 (δx ← 0) ──
        # 쿼터니언 기반 리셋 Jacobian G ≈ I (소각 근사)
        # P ← G P G^T ≈ P (변화 없음)

    # ----------------------------------------------------------
    # GPS 위치 업데이트
    # ----------------------------------------------------------

    def update_gps_pos(self, pos_world: np.ndarray,
                       cov: Optional[np.ndarray] = None) -> None:
        """
        GPS 위치 측정 업데이트
        h(x) = p,  H = [I  0  0  0  0]
        """
        H = np.zeros((3, self.NX))
        H[0:3, 0:3] = np.eye(3)

        if cov is not None:
            R_n = cov
        else:
            R_n = np.eye(3) * self.sigma_gps_pos**2

        innovation = pos_world - self.p
        self._correct(H, R_n, innovation)

    # ----------------------------------------------------------
    # GPS 속도 업데이트
    # ----------------------------------------------------------

    def update_gps_vel(self, vel_world: np.ndarray,
                       cov: Optional[np.ndarray] = None) -> None:
        """
        GPS 속도 측정 업데이트
        h(x) = v,  H = [0  I  0  0  0]
        """
        H = np.zeros((3, self.NX))
        H[0:3, 3:6] = np.eye(3)

        if cov is not None:
            R_n = cov
        else:
            R_n = np.eye(3) * self.sigma_gps_vel**2

        innovation = vel_world - self.v
        self._correct(H, R_n, innovation)

    # ----------------------------------------------------------
    # 듀얼 GPS 헤딩 업데이트
    # ----------------------------------------------------------

    def update_heading(self, heading_rad: float,
                       sigma: Optional[float] = None) -> None:
        """
        듀얼 GPS 헤딩 측정 업데이트
        h(x) = yaw(q),  H = ∂yaw/∂δθ (1×15)

        yaw = atan2(2(qw*qz+qx*qy), 1-2(qy²+qz²))
        ∂yaw/∂δθ 는 쿼터니언 미분으로 계산
        """
        w, x, y, z = self.q / np.linalg.norm(self.q)

        # yaw 예측값
        yaw_pred = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

        # ∂yaw/∂q (1×4)
        denom1 = 2*(w*z + x*y)
        denom2 = 1 - 2*(y*y + z*z)
        denom  = denom1**2 + denom2**2
        if denom < 1e-10:
            return

        dyaw_dqw = (2*z * denom2) / denom
        dyaw_dqx = (2*y * denom2) / denom
        dyaw_dqy = (2*x * denom2 + 4*y * denom1) / denom
        dyaw_dqz = (2*w * denom2 - 4*z * denom2) / denom
        # 실제:
        dyaw_dqw = ( 2*z*denom2) / denom
        dyaw_dqx = ( 2*y*denom2) / denom
        dyaw_dqy = ( 2*x*denom2 + 4*y*denom1) / denom
        dyaw_dqz = ( 2*w*denom2) / denom
        dq = np.array([dyaw_dqw, dyaw_dqx, dyaw_dqy, dyaw_dqz])

        # ∂q/∂δθ (4×3) : 쿼터니언 오차에서 각도 오차로 변환
        # q = q_nom ⊗ [1, δθ/2]  →  ∂q/∂δθ = 0.5 * Q_L 의 마지막 3열
        # Q_L (left-multiply matrix)
        Q_L = np.array([
            [-x, -y, -z],
            [ w, -z,  y],
            [ z,  w, -x],
            [-y,  x,  w],
        ]) * 0.5

        # ∂yaw/∂δθ (1×3)
        dyaw_dtheta = dq @ Q_L  # (1×4) × (4×3) = (1×3)

        # H 행렬 (1×15)
        H = np.zeros((1, self.NX))
        H[0, 6:9] = dyaw_dtheta

        # 혁신 (heading 차이, 각도 래핑)
        inno = np.array([wrap_angle(heading_rad - yaw_pred)])

        sigma_h = sigma if sigma is not None else self.sigma_heading
        R_n = np.array([[sigma_h**2]])

        self._correct(H, R_n, inno)

    # ----------------------------------------------------------
    # 상태 접근자
    # ----------------------------------------------------------

    @property
    def yaw_rad(self) -> float:
        return quat_to_yaw(self.q)

    @property
    def yaw_deg(self) -> float:
        return math.degrees(self.yaw_rad)

    @property
    def heading_uncertainty_deg(self) -> float:
        """헤딩 불확도 (1σ) [deg]"""
        return math.degrees(math.sqrt(max(0., self.P[6, 6]
                                          + self.P[7, 7]
                                          + self.P[8, 8])))

    @property
    def covariance_trace(self) -> float:
        return float(np.trace(self.P))


# ============================================================
# ROS2 노드
# ============================================================

class ESKFNode(Node):

    def __init__(self) -> None:
        super().__init__('eskf_node')

        # ── 파라미터 ──
        self.declare_parameter('utm_zone',              52)
        self.declare_parameter('odom_frame',            'odom')
        self.declare_parameter('base_frame',            'base_footprint')
        self.declare_parameter('imu_topic',             '/handsfree/imu')
        self.declare_parameter('gps_fix_topic',         '/gnss_rover/fix')
        self.declare_parameter('gps_vel_topic',         '/ublox_gps/fix_velocity')
        self.declare_parameter('dual_heading_topic',    '/dual_f9p/heading')
        self.declare_parameter('dual_heading_valid_topic', '/dual_f9p/heading_valid')
        self.declare_parameter('dual_heading_accuracy_topic', '/dual_f9p/heading_accuracy_deg')
        # 노이즈 파라미터
        self.declare_parameter('sigma_accel',       0.05)
        self.declare_parameter('sigma_gyro',        0.005)
        self.declare_parameter('sigma_accel_bias',  0.001)
        self.declare_parameter('sigma_gyro_bias',   0.0001)
        self.declare_parameter('sigma_gps_pos',     0.5)
        self.declare_parameter('sigma_gps_vel',     0.1)
        self.declare_parameter('sigma_heading',     0.02)
        self.declare_parameter('gravity',           9.81)
        # 초기화 파라미터
        self.declare_parameter('init_from_gps',     True)
        self.declare_parameter('init_heading_deg',  0.0)
        self.declare_parameter('yaw_offset_deg',    0.0)

        p = lambda n: self.get_parameter(n).value  # noqa

        self.utm_zone   = int(p('utm_zone'))
        self.odom_frame = str(p('odom_frame'))
        self.base_frame = str(p('base_frame'))
        self.yaw_offset = math.radians(float(p('yaw_offset_deg')))

        self.eskf = ESKF(
            sigma_accel      = float(p('sigma_accel')),
            sigma_gyro       = float(p('sigma_gyro')),
            sigma_accel_bias = float(p('sigma_accel_bias')),
            sigma_gyro_bias  = float(p('sigma_gyro_bias')),
            sigma_gps_pos    = float(p('sigma_gps_pos')),
            sigma_gps_vel    = float(p('sigma_gps_vel')),
            sigma_heading    = float(p('sigma_heading')),
            gravity          = float(p('gravity')),
        )

        # UTM 투영기
        self._proj: Optional[object] = None
        if _HAS_PYPROJ:
            self._proj = Proj(proj='utm', zone=self.utm_zone,
                              ellps='WGS84', preserve_units=False)

        self.origin_x: Optional[float] = None
        self.origin_y: Optional[float] = None
        self.last_imu_time: Optional[float] = None
        self.dual_heading_valid = False
        self.dual_heading_rad   = 0.0
        self.dual_heading_sigma = float(p('sigma_heading'))
        self.init_from_gps = bool(p('init_from_gps'))
        self.init_heading_deg = float(p('init_heading_deg'))

        # 초기화 플래그
        self._got_gps = False
        self._got_imu = False

        # ── QoS ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── 구독 ──
        self.create_subscription(Imu, str(p('imu_topic')),
                                 self._imu_cb, sensor_qos)
        self.create_subscription(NavSatFix, str(p('gps_fix_topic')),
                                 self._gps_fix_cb, sensor_qos)
        self.create_subscription(TwistWithCovarianceStamped, str(p('gps_vel_topic')),
                                 self._gps_vel_cb, sensor_qos)
        self.create_subscription(Float64, str(p('dual_heading_topic')),
                                 self._dual_heading_cb, 10)
        self.create_subscription(Bool, str(p('dual_heading_valid_topic')),
                                 self._dual_heading_valid_cb, 10)
        self.create_subscription(Float64, str(p('dual_heading_accuracy_topic')),
                                 self._dual_heading_acc_cb, 10)

        # ── 발행 ──
        self.odom_pub       = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.origin_pub     = self.create_publisher(Point, '/utm_origin', transient_qos)
        self.heading_pub    = self.create_publisher(Float64, '/eskf/heading_deg', 10)
        self.cov_pub        = self.create_publisher(Float64, '/eskf/covariance_trace', 10)

        self.get_logger().info('ESKF 노드 시작 (GPS+IMU 융합, 정밀 헤딩)')
        self.get_logger().info(f'  UTM zone: {self.utm_zone}')
        self.get_logger().info(f'  pyproj: {"사용 가능" if _HAS_PYPROJ else "미설치 (fallback)"}')

    # ----------------------------------------------------------
    # IMU 콜백 (예측 단계)
    # ----------------------------------------------------------

    def _imu_cb(self, msg: Imu) -> None:
        now = self._ros_time()
        self._got_imu = True

        # IMU 데이터 추출
        a_m = np.array([msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z])
        w_m = np.array([msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z])

        # ESKF 미초기화 시 IMU 자세로 초기화
        if not self.eskf.initialized:
            if not self.init_from_gps or self._got_gps:
                # IMU 쿼터니언으로 초기 자세 설정
                q_msg = msg.orientation
                q = np.array([q_msg.w, q_msg.x, q_msg.y, q_msg.z])
                if abs(np.linalg.norm(q) - 1.0) < 0.1:
                    yaw0 = quat_to_yaw(q) + self.yaw_offset
                else:
                    yaw0 = math.radians(self.init_heading_deg)
                self.eskf.init(self.eskf.p.copy(), np.zeros(3), yaw0)
                self.get_logger().info(
                    f'ESKF 초기화 완료 (yaw={math.degrees(yaw0):.1f}°)')
            self.last_imu_time = now
            return

        # 시간 간격 계산
        dt = now - self.last_imu_time if self.last_imu_time is not None else 0.0
        self.last_imu_time = now

        if 0 < dt < 0.5:
            self.eskf.predict(a_m, w_m, dt)
            self._publish_state()

    # ----------------------------------------------------------
    # GPS Fix 콜백 (위치 업데이트)
    # ----------------------------------------------------------

    def _gps_fix_cb(self, msg: NavSatFix) -> None:
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        x, y = self._latlon_to_local(msg.latitude, msg.longitude)
        pos = np.array([x, y, 0.0])

        # 최초 GPS 수신 → 초기화
        if not self._got_gps:
            self._got_gps = True
            if not self.eskf.initialized:
                yaw0 = math.radians(self.init_heading_deg)
                self.eskf.init(pos, np.zeros(3), yaw0)
                self.get_logger().info(
                    f'GPS로 ESKF 위치 초기화: ({x:.2f}, {y:.2f})')
            else:
                self.eskf.p = pos.copy()
            return

        if not self.eskf.initialized:
            return

        # GPS 위치 공분산
        cov_gps = None
        if len(msg.position_covariance) >= 9 and msg.position_covariance[0] > 0:
            c = msg.position_covariance
            cov_gps = np.array([
                [c[0], c[1], 0.],
                [c[3], c[4], 0.],
                [0.,   0.,   self.eskf.sigma_gps_pos**2],
            ])
        self.eskf.update_gps_pos(pos, cov_gps)

    # ----------------------------------------------------------
    # GPS 속도 콜백 (속도 업데이트 + 헤딩 보조)
    # ----------------------------------------------------------

    def _gps_vel_cb(self, msg: TwistWithCovarianceStamped) -> None:
        if not self.eskf.initialized:
            return

        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        vel = np.array([vx, vy, 0.0])

        cov_vel = None
        if len(msg.twist.covariance) >= 9:
            c = msg.twist.covariance
            var = max(c[0], self.eskf.sigma_gps_vel**2)
            cov_vel = np.diag([var, var, self.eskf.sigma_gps_vel**2])

        self.eskf.update_gps_vel(vel, cov_vel)

        # 속도 충분하면 GPS 진행방향으로 헤딩 보조
        speed = math.sqrt(vx*vx + vy*vy)
        if speed > 0.5 and not self.dual_heading_valid:
            gps_heading = math.atan2(vy, vx) + self.yaw_offset
            # 듀얼 GPS 없을 때만, 더 큰 불확도로 업데이트
            self.eskf.update_heading(
                gps_heading,
                sigma=math.radians(5.0) / max(speed, 0.5))

    # ----------------------------------------------------------
    # 듀얼 GPS 헤딩 콜백 (가장 정확한 헤딩 소스)
    # ----------------------------------------------------------

    def _dual_heading_cb(self, msg: Float64) -> None:
        if not self.eskf.initialized:
            return
        self.dual_heading_rad = math.radians(float(msg.data)) + self.yaw_offset
        if self.dual_heading_valid:
            self.eskf.update_heading(
                self.dual_heading_rad,
                sigma=self.dual_heading_sigma)

    def _dual_heading_valid_cb(self, msg: Bool) -> None:
        self.dual_heading_valid = bool(msg.data)

    def _dual_heading_acc_cb(self, msg: Float64) -> None:
        # 듀얼 GPS 헤딩 정확도 [deg] → sigma [rad]
        acc_deg = max(0.01, float(msg.data))
        self.dual_heading_sigma = math.radians(acc_deg)

    # ----------------------------------------------------------
    # 상태 발행
    # ----------------------------------------------------------

    def _publish_state(self) -> None:
        if not self.eskf.initialized:
            return

        now = self.get_clock().now().to_msg()

        # Odometry
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x = float(self.eskf.p[0])
        odom.pose.pose.position.y = float(self.eskf.p[1])
        odom.pose.pose.position.z = float(self.eskf.p[2])

        w, x, y, z = self.eskf.q
        odom.pose.pose.orientation.w = float(w)
        odom.pose.pose.orientation.x = float(x)
        odom.pose.pose.orientation.y = float(y)
        odom.pose.pose.orientation.z = float(z)

        odom.twist.twist.linear.x  = float(self.eskf.v[0])
        odom.twist.twist.linear.y  = float(self.eskf.v[1])
        odom.twist.twist.linear.z  = float(self.eskf.v[2])

        # pose 공분산 (6×6, row-major): pos(3) + rpy(3)
        pc = [0.0] * 36
        for i in range(3):
            pc[i*6+i]       = float(self.eskf.P[i, i])       # 위치
            pc[(i+3)*6+i+3] = float(self.eskf.P[i+6, i+6])   # 자세
        odom.pose.covariance = pc

        self.odom_pub.publish(odom)

        # 헤딩
        h = Float64()
        h.data = self.eskf.yaw_deg
        self.heading_pub.publish(h)

        # 공분산 트레이스
        ct = Float64()
        ct.data = self.eskf.covariance_trace
        self.cov_pub.publish(ct)

    # ----------------------------------------------------------
    # 유틸리티
    # ----------------------------------------------------------

    def _ros_time(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _latlon_to_local(self, lat: float, lon: float) -> Tuple[float, float]:
        if self._proj is not None:
            xg, yg = self._proj(lon, lat)
        else:
            r  = 6378137.0
            xg = math.radians(lon) * r * math.cos(math.radians(lat))
            yg = math.radians(lat) * r

        if self.origin_x is None:
            self.origin_x = xg
            self.origin_y = yg
            p = Point()
            p.x, p.y, p.z = xg, yg, 0.0
            self.origin_pub.publish(p)
            self.get_logger().info(
                f'UTM 원점 설정: ({xg:.3f}, {yg:.3f})')

        return xg - self.origin_x, yg - self.origin_y


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ESKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
