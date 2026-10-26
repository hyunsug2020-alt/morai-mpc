#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
morai_to_liosam_node.py  —  MORAI 점군 -> LIO-SAM 입력 변환 노드

LIO-SAM(imageProjection)은 다음을 요구한다:
  1) ring 필드 (uint16) — 없으면 "Point cloud ring channel not available" 로 즉시 종료 ★필수
  2) time 필드 (float32, 스캔 시작 기준 상대시간) — 없으면 deskew 만 비활성(동작은 함)
MORAI ROS 브리지의 /velodyne_points 는 보통 x,y,z,intensity 만 있으므로
수직각(elevation)으로 ring 을, 방위각(azimuth)으로 time 을 계산해 붙인다.

★ 반드시 "라이다 프레임 원시 점군" 을 입력으로 쓸 것.
  /lidar/points_proc (base_link 변환 + ego crop 완료본) 은 LIO-SAM 에 넣으면 안 된다.
  extrinsic 은 LIO-SAM params.yaml 쪽에서 처리한다.

동작:
  /velodyne_points (x,y,z,i) -> ring/time 계산 -> /velodyne_points_ring 발행
  (입력에 이미 ring 이 있으면 그대로 통과 + time 만 필요 시 추가)

실행:
  rosrun lidar_pointpillars morai_to_liosam_node.py
파라미터 (HDL-32E 기본값):
  ~n_scan       : 32        채널 수
  ~ang_bottom   : -30.67    최하단 채널 수직각 (deg)
  ~ang_res_y    : 1.33      채널 간 수직각 간격 (deg)
  ~scan_period  : 0.1       스캔 주기 (s, 10Hz)
  ~add_time     : true      azimuth 기반 time 필드 추가 여부
                            (MORAI 는 스캔을 순간 생성하므로 모션 왜곡이 없어
                             deskew 불필요 -> false 로 두고 LIO-SAM 경고 무시해도 됨)
"""

import numpy as np

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class MoraiToLiosamNode:
    def __init__(self):
        rospy.init_node("morai_to_liosam_node")

        in_topic = rospy.get_param("~input_topic", "/velodyne_points")
        out_topic = rospy.get_param("~output_topic", "/velodyne_points_ring")
        self.n_scan = int(rospy.get_param("~n_scan", 32))
        self.ang_bottom = float(rospy.get_param("~ang_bottom", -30.67))   # HDL-32E
        self.ang_res_y = float(rospy.get_param("~ang_res_y", 1.33))       # HDL-32E
        self.scan_period = float(rospy.get_param("~scan_period", 0.1))    # 10 Hz
        self.add_time = bool(rospy.get_param("~add_time", True))

        # 출력 레이아웃: x,y,z,intensity(f32) + ring(u16) + time(f32) = 22 byte
        # (pcl::fromROSMsg 는 필드 이름/오프셋으로 복사하므로 패딩 불필요)
        self.out_dtype = np.dtype([
            ("x", np.float32), ("y", np.float32), ("z", np.float32),
            ("intensity", np.float32),
            ("ring", np.uint16),
            ("time", np.float32),
        ])
        self.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
            PointField("ring", 16, PointField.UINT16, 1),
            PointField("time", 18, PointField.FLOAT32, 1),
        ]

        self.pub = rospy.Publisher(out_topic, PointCloud2, queue_size=3)
        self.sub = rospy.Subscriber(in_topic, PointCloud2, self.callback,
                                    queue_size=3, buff_size=2 ** 24, tcp_nodelay=True)
        self._warned_ring = False
        rospy.loginfo(f"[morai2liosam] {in_topic} -> {out_topic} "
                      f"(n_scan={self.n_scan}, bottom={self.ang_bottom}deg, "
                      f"res={self.ang_res_y}deg, add_time={self.add_time})")

    # ---------------------------------------------------------
    def _parse(self, msg):
        """PointCloud2 -> dict of arrays (x,y,z,intensity[,ring][,time])"""
        ps = msg.point_step
        n = len(msg.data) // ps if ps else 0
        if n == 0:
            return None
        raw = np.frombuffer(msg.data, dtype=np.uint8, count=n * ps).reshape(n, ps)
        off = {f.name: (f.offset, f.datatype) for f in msg.fields}

        def f32(name, default=None):
            if name not in off:
                return default
            o = off[name][0]
            return raw[:, o:o + 4].copy().view(np.float32).ravel()

        out = {
            "x": f32("x"), "y": f32("y"), "z": f32("z"),
            "intensity": f32("intensity", np.zeros(n, np.float32)),
        }
        if "ring" in off:
            o, dt = off["ring"]
            width = 2 if dt == PointField.UINT16 else 4
            view = np.uint16 if dt == PointField.UINT16 else np.uint32
            out["ring"] = raw[:, o:o + width].copy().view(view).ravel().astype(np.uint16)
        if "time" in off:
            out["time"] = f32("time")
        elif "t" in off:
            out["time"] = f32("t")
        return out

    # ---------------------------------------------------------
    def callback(self, msg):
        d = self._parse(msg)
        if d is None:
            return
        x, y, z, inten = d["x"], d["y"], d["z"], d["intensity"]

        good = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z, inten = x[good], y[good], z[good], inten[good]
        n = x.shape[0]
        if n == 0:
            return

        # ----- ring: 있으면 통과, 없으면 수직각으로 계산 -----
        if "ring" in d:
            ring = d["ring"][good]
        else:
            if not self._warned_ring:
                rospy.loginfo("[morai2liosam] 입력에 ring 없음 -> 수직각 기반 계산 사용")
                self._warned_ring = True
            r_xy = np.sqrt(x * x + y * y)
            elev = np.degrees(np.arctan2(z, np.maximum(r_xy, 1e-6)))
            ring_f = np.rint((elev - self.ang_bottom) / self.ang_res_y)
            # 각도 계산 오차로 범위를 벗어난 점(수직각이 채널 범위 밖)은 버림
            ok = (ring_f >= 0) & (ring_f <= self.n_scan - 1)
            x, y, z, inten, ring_f = x[ok], y[ok], z[ok], inten[ok], ring_f[ok]
            ring = ring_f.astype(np.uint16)
            n = x.shape[0]
            if n == 0:
                return

        # ----- time: 있으면 통과, 없으면 azimuth 로 근사 -----
        if "time" in d and d["time"] is not None and "ring" in d:
            t = d["time"][good]
        elif self.add_time:
            # 스캔 시작(azimuth=0, +x 방향)에서 시계방향 회전 가정한 근사.
            # MORAI 는 스캔을 순간 생성하므로 실제 왜곡은 없음 — deskew 를
            # 형식적으로 만족시키는 값이며, 정확도에 민감하지 않다.
            az = np.arctan2(y, x)                    # [-pi, pi]
            az = (-az) % (2.0 * np.pi)               # 시계방향 0 -> 2pi
            t = (az / (2.0 * np.pi) * self.scan_period).astype(np.float32)
        else:
            t = np.zeros(n, np.float32)

        # ----- 구조화 배열로 고속 직렬화 -----
        out = np.empty(n, dtype=self.out_dtype)
        out["x"], out["y"], out["z"] = x, y, z
        out["intensity"] = inten
        out["ring"] = ring
        out["time"] = t

        header = Header()
        header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
        header.frame_id = msg.header.frame_id or "velodyne"

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = n
        cloud.fields = self.fields
        cloud.is_bigendian = False
        cloud.point_step = self.out_dtype.itemsize   # 22
        cloud.row_step = cloud.point_step * n
        cloud.is_dense = True
        cloud.data = out.tobytes()
        self.pub.publish(cloud)


if __name__ == "__main__":
    try:
        MoraiToLiosamNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
