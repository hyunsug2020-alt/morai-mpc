#!/usr/bin/env python3
"""Planar vehicle dead reckoning from wheel kinematics and noisy IMU.

MORAI currently exposes no separate wheel encoder topic.  This adapter reads
only ``velocity`` and ``wheel_angle`` from EgoVehicleStatus.  It deliberately
never reads position, heading, acceleration, or any other ground-truth pose
field.  Ego position/heading are consumed only by the separate validation GUI.
"""

import json
import math
import threading
import time
from collections import deque

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def first_order_alpha(delta_time, time_constant):
    """Return a rate-independent first-order low-pass coefficient."""
    if time_constant <= 0.0:
        return 1.0
    return 1.0 - math.exp(-max(0.0, delta_time) / time_constant)


class PureVehicleOdometry:
    def __init__(self):
        rospy.init_node("pure_odometry")
        self.lock = threading.RLock()

        self.vehicle_topic = rospy.get_param(
            "~vehicle_state_topic", "/Ego_topic")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/data")
        self.output_topic = rospy.get_param(
            "~output_topic", "/odometry/pure")
        self.odom_frame = rospy.get_param("~odom_frame", "pure_odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.publish_tf = bool(rospy.get_param("~publish_tf", False))
        self.require_sim_time = bool(rospy.get_param(
            "~require_sim_time", False))
        self.use_sim_time = bool(rospy.get_param("/use_sim_time", False))

        self.wheelbase = float(rospy.get_param("~wheelbase", 3.0))
        self.model_wheelbase = float(rospy.get_param(
            "~model_wheelbase_m", self.wheelbase))
        self.model_input_delay = max(0.0, float(rospy.get_param(
            "~model_input_delay_source_s", 0.0)))
        self.steering_sign = float(rospy.get_param("~steering_sign", 1.0))
        self.wheel_angle_unit = rospy.get_param(
            "~wheel_angle_unit", "deg").strip().lower()
        self.imu_sign = float(rospy.get_param("~imu_sign", 1.0))
        self.imu_weight = float(rospy.get_param("~imu_weight", 0.95))
        self.imu_weight_min = float(rospy.get_param(
            "~imu_weight_min", 0.85))
        self.imu_weight_max = float(rospy.get_param(
            "~imu_weight_max", 0.98))
        self.motion_rate_scale = float(rospy.get_param(
            "~motion_rate_scale", 0.5 if not self.use_sim_time else 1.0))
        self.adaptive_time_scale = bool(rospy.get_param(
            "~adaptive_time_scale", not self.use_sim_time))
        self.time_scale_window = float(rospy.get_param(
            "~time_scale_window_s", 10.0))
        self.time_scale_min_rotation = float(rospy.get_param(
            "~time_scale_min_rotation_rad", 0.001))
        self.time_scale_alpha = float(rospy.get_param(
            "~time_scale_alpha", 0.05))
        self.time_scale_min = float(rospy.get_param(
            "~time_scale_min", 0.2))
        self.time_scale_max = float(rospy.get_param(
            "~time_scale_max", 1.2))
        self.understeer_coefficient = float(rospy.get_param(
            "~understeer_coefficient_s2pm", 0.022))
        self.imu_timeout = float(rospy.get_param(
            "~imu_timeout_s", 0.08))
        self.gyro_gate = float(rospy.get_param(
            "~gyro_model_gate_radps", 0.75))
        self.max_abs_gyro = float(rospy.get_param(
            "~max_abs_gyro_radps", 2.0))
        self.orientation_primary = bool(rospy.get_param(
            "~orientation_primary", True))
        self.orientation_timeout = max(0.0, float(rospy.get_param(
            "~orientation_timeout_s", 0.08)))
        self.orientation_gyro_gate = max(0.0, float(rospy.get_param(
            "~orientation_gyro_gate_radps", 0.25)))
        self.max_orientation_rate = max(0.0, float(rospy.get_param(
            "~max_orientation_rate_radps", self.max_abs_gyro)))
        self.nis_gate = float(rospy.get_param("~yaw_rate_nis_gate", 6.63))
        self.nis_hard_gate = float(rospy.get_param(
            "~yaw_rate_nis_hard_gate", 10.83))
        self.model_yaw_rate_noise = float(rospy.get_param(
            "~model_yaw_rate_noise_std_radps", 0.03))
        self.gyro_yaw_rate_noise = float(rospy.get_param(
            "~gyro_yaw_rate_noise_std_radps", 0.03))
        self.moving_bias_gain = float(rospy.get_param(
            "~moving_gyro_bias_gain", 0.002))
        self.stationary_speed = float(rospy.get_param(
            "~stationary_speed_mps", 0.05))
        self.stationary_gyro = float(rospy.get_param(
            "~stationary_gyro_radps", 0.03))
        self.stationary_bias_hold = float(rospy.get_param(
            "~stationary_bias_hold_s", 0.8))
        self.bias_alpha = float(rospy.get_param(
            "~gyro_bias_alpha", 0.02))
        self.speed_filter_tau = float(rospy.get_param(
            "~speed_filter_tau_s", 0.04))
        self.yaw_rate_filter_tau = float(rospy.get_param(
            "~yaw_rate_filter_tau_s", 0.02))
        self.max_dt = float(rospy.get_param("~max_dt_s", 0.25))
        self.max_recoverable_gap = float(rospy.get_param(
            "~max_recoverable_gap_s", 1.0))
        self.max_steering = math.radians(float(rospy.get_param(
            "~max_steering_deg", 40.0)))
        self.speed_noise = float(rospy.get_param(
            "~speed_noise_std_mps", 0.05))
        self.yaw_rate_noise = float(rospy.get_param(
            "~yaw_rate_noise_std_radps", 0.02))

        if self.wheelbase <= 0.0 or self.model_wheelbase <= 0.0:
            raise ValueError("wheelbase parameters must be positive")
        if self.wheel_angle_unit not in ("deg", "rad"):
            raise ValueError("~wheel_angle_unit must be 'deg' or 'rad'")
        self.imu_weight_min = min(1.0, max(0.0, self.imu_weight_min))
        self.imu_weight_max = min(
            1.0, max(self.imu_weight_min, self.imu_weight_max))
        self.imu_weight = min(
            self.imu_weight_max,
            max(self.imu_weight_min, self.imu_weight))
        self.time_scale_min = max(0.05, self.time_scale_min)
        self.time_scale_max = max(
            self.time_scale_min, self.time_scale_max)
        self.motion_rate_scale = min(
            self.time_scale_max,
            max(self.time_scale_min, self.motion_rate_scale))
        self.time_scale_alpha = min(
            1.0, max(0.001, self.time_scale_alpha))
        self.time_scale_window = max(2.0, self.time_scale_window)
        self.time_scale_min_rotation = max(
            1.0e-5, self.time_scale_min_rotation)
        self.bias_alpha = min(1.0, max(0.0, self.bias_alpha))
        if self.use_sim_time:
            self.motion_rate_scale = 1.0
            self.adaptive_time_scale = False
        if self.require_sim_time:
            if not bool(rospy.get_param("/use_sim_time", False)):
                raise RuntimeError(
                    "pure odometry requires /use_sim_time=true")
            clock_deadline = time.monotonic() + 5.0
            while (
                    rospy.Time.now() == rospy.Time()
                    and time.monotonic() < clock_deadline
                    and not rospy.is_shutdown()):
                time.sleep(0.05)
            if rospy.Time.now() == rospy.Time():
                raise RuntimeError(
                    "pure odometry requires a valid MORAI /clock")

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.covariance = np.diag([1.0e-6, 1.0e-6, 1.0e-6])
        self.filtered_speed = 0.0
        self.filtered_yaw_rate = 0.0
        self.previous_filtered_speed = 0.0
        self.previous_filtered_yaw_rate = 0.0
        self.gyro_z = 0.0
        self.gyro_bias = 0.0
        self.imu_history = deque(maxlen=400)
        self.imu_yaw_history = deque(maxlen=400)
        self.vehicle_model_history = deque(maxlen=400)
        self.time_scale_history = deque(maxlen=2000)
        self.previous_imu_stamp = None
        self.previous_imu_yaw = None
        self.previous_imu_gyro = None
        self.previous_raw_imu_yaw = None
        self.unwrapped_imu_yaw = None
        self.orientation_reference_imu_yaw = None
        self.orientation_reference_odometry_yaw = None
        self.last_orientation_sample_yaw = None
        self.last_orientation_sample_stamp = None
        self.orientation_consistency_count = 0
        self.orientation_invalid_streak = 0
        self.orientation_reanchor_pending = False
        self.latest_time_scale_candidate = None
        self.time_scale_updates = 0
        self.last_imu_arrival = None
        self.last_vehicle_arrival = None
        self.last_stamp_progress_arrival = None
        self.last_publish_arrival = None
        self.last_vehicle_stamp = None
        self.stationary_since_stamp = None
        self.latest_steering = 0.0
        self.latest_raw_wheel_angle = 0.0
        self.latest_model_yaw_rate = 0.0
        self.latest_imu_yaw_rate = 0.0
        self.latest_orientation_yaw_rate = None
        self.latest_model_speed = 0.0
        self.latest_model_steering = 0.0
        self.latest_imu_weight = 0.0
        self.latest_yaw_rate_nis = 0.0
        self.latest_model_disagreement = 0.0
        self.latest_imu_stamp_skew = None
        self.imu_stamp_skew_history = deque(maxlen=300)
        self.vehicle_messages = 0
        self.imu_messages = 0
        self.published = 0
        self.rejected_dt = 0
        self.imu_outliers = 0
        self.duplicate_vehicle_stamps = 0
        self.stationary_updates = 0
        self.time_resets = 0
        self.imu_stale_fallbacks = 0
        self.imu_extrapolations = 0
        self.model_disagreements = 0
        self.recovered_time_gaps = 0
        self.unobserved_time_sec = 0.0
        self.invalid_steering = 0
        self.orientation_primary_updates = 0
        self.orientation_stale_fallbacks = 0
        self.orientation_invalid_fallbacks = 0
        self.orientation_reference_resets = 0
        self.model_delay_warmups = 0

        self.publisher = rospy.Publisher(
            self.output_topic, Odometry, queue_size=30)
        self.diagnostics_publisher = rospy.Publisher(
            "/pure_odometry/diagnostics", String, queue_size=2)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.imu_subscriber = rospy.Subscriber(
            self.imu_topic, Imu, self.imu_callback,
            queue_size=100, tcp_nodelay=True)
        self.vehicle_subscriber = rospy.Subscriber(
            self.vehicle_topic, EgoVehicleStatus, self.vehicle_callback,
            queue_size=100, tcp_nodelay=True)
        self.diagnostics_timer = rospy.Timer(
            rospy.Duration(1.0), self.publish_diagnostics)

        rospy.logwarn(
            "[Pure odometry] %s에서 velocity/wheel_angle만 사용함; "
            "position/heading은 읽지 않음", self.vehicle_topic)
        rospy.loginfo(
            "[Pure odometry] vehicle=%s imu=%s output=%s "
            "model_wheelbase=%.4fm delay=%.3fs "
            "time_scale=%s(%.4f)",
            self.vehicle_topic, self.imu_topic, self.output_topic,
            self.model_wheelbase, self.model_input_delay,
            "imu_adaptive" if self.adaptive_time_scale else "fixed",
            self.motion_rate_scale)

    def imu_callback(self, message):
        arrival = time.monotonic()
        gyro_z = float(message.angular_velocity.z)
        gyro_valid = math.isfinite(gyro_z)
        with self.lock:
            if gyro_valid:
                self.gyro_z = self.imu_sign * gyro_z
            stamp = message.header.stamp
            stamp_sec = (
                stamp.to_sec() if stamp != rospy.Time()
                else rospy.Time.now().to_sec())
            quaternion = message.orientation
            quaternion_values = (
                float(quaternion.x), float(quaternion.y),
                float(quaternion.z), float(quaternion.w))
            quaternion_finite = all(
                math.isfinite(value) for value in quaternion_values)
            quaternion_norm = (
                math.sqrt(sum(value * value for value in quaternion_values))
                if quaternion_finite else 0.0)
            imu_yaw = None
            if (
                    quaternion_norm > 1.0e-6
                    and message.orientation_covariance[0] != -1.0):
                normalized = [
                    value / quaternion_norm for value in quaternion_values]
                imu_yaw = euler_from_quaternion(normalized)[2]

            if (
                    self.adaptive_time_scale
                    and not self.use_sim_time
                    and gyro_valid
                    and imu_yaw is not None
                    and self.previous_imu_stamp is not None
                    and self.previous_imu_yaw is not None
                    and self.previous_imu_gyro is not None):
                wall_dt = stamp_sec - self.previous_imu_stamp
                if 0.0 < wall_dt <= 0.5:
                    yaw_increment = abs(wrap_angle(
                        imu_yaw - self.previous_imu_yaw))
                    gyro_increment = abs(
                        0.5 * (self.previous_imu_gyro + self.gyro_z)
                        * wall_dt)
                    self.time_scale_history.append((
                        stamp_sec, yaw_increment, gyro_increment))
                    cutoff = stamp_sec - self.time_scale_window
                    while (
                            self.time_scale_history
                            and self.time_scale_history[0][0] < cutoff):
                        self.time_scale_history.popleft()
                    yaw_sum = sum(
                        sample[1] for sample in self.time_scale_history)
                    gyro_sum = sum(
                        sample[2] for sample in self.time_scale_history)
                    if gyro_sum >= self.time_scale_min_rotation:
                        candidate = yaw_sum / gyro_sum
                        if self.time_scale_min <= candidate <= self.time_scale_max:
                            self.latest_time_scale_candidate = candidate
                            # yaw_sum/gyro_sum is already a robust trailing
                            # window estimate.  A second EMA added several
                            # seconds of avoidable lag when MORAI's physical
                            # rate changed, so apply the bounded window
                            # estimate directly.
                            self.motion_rate_scale = candidate
                            self.time_scale_updates += 1
            if imu_yaw is not None and gyro_valid:
                self.previous_imu_stamp = stamp_sec
                self.previous_imu_yaw = imu_yaw
                self.previous_imu_gyro = self.gyro_z
            if (
                    self.imu_history
                    and stamp_sec < self.imu_history[-1][0] - 1.0):
                # MORAI resets simulation time when the time mode changes.
                self.imu_history.clear()
                self.imu_yaw_history.clear()
                self.time_scale_history.clear()
                self.previous_imu_stamp = None
                self.previous_imu_yaw = None
                self.previous_imu_gyro = None
                self.previous_raw_imu_yaw = None
                self.unwrapped_imu_yaw = None
                self.orientation_reference_imu_yaw = None
                self.orientation_reference_odometry_yaw = None
                self.last_orientation_sample_yaw = None
                self.last_orientation_sample_stamp = None
                self.orientation_consistency_count = 0
                self.orientation_invalid_streak = 0
                self.orientation_reanchor_pending = False
            if gyro_valid:
                if not self.imu_history or stamp_sec > self.imu_history[-1][0]:
                    self.imu_history.append((stamp_sec, self.gyro_z))
                elif stamp_sec == self.imu_history[-1][0]:
                    self.imu_history[-1] = (stamp_sec, self.gyro_z)
            if imu_yaw is not None:
                if not self.imu_yaw_history:
                    self.unwrapped_imu_yaw = imu_yaw
                    self.previous_raw_imu_yaw = imu_yaw
                    self.imu_yaw_history.append((stamp_sec, imu_yaw))
                elif stamp_sec > self.imu_yaw_history[-1][0]:
                    self.unwrapped_imu_yaw += wrap_angle(
                        imu_yaw - self.previous_raw_imu_yaw)
                    self.previous_raw_imu_yaw = imu_yaw
                    self.imu_yaw_history.append(
                        (stamp_sec, self.unwrapped_imu_yaw))
                elif stamp_sec == self.imu_yaw_history[-1][0]:
                    self.imu_yaw_history[-1] = (
                        stamp_sec, self.unwrapped_imu_yaw)
            self.last_imu_arrival = arrival
            self.imu_messages += 1

    def gyro_at_stamp(self, stamp_sec):
        """Linearly interpolate gyro-z at the vehicle source timestamp."""
        if not self.imu_history:
            return None, math.inf, True
        samples = list(self.imu_history)
        if stamp_sec <= samples[0][0]:
            return samples[0][1], samples[0][0] - stamp_sec, True
        if stamp_sec >= samples[-1][0]:
            return samples[-1][1], stamp_sec - samples[-1][0], True
        low = 0
        high = len(samples) - 1
        while high - low > 1:
            middle = (low + high) // 2
            if samples[middle][0] <= stamp_sec:
                low = middle
            else:
                high = middle
        first_stamp, first_value = samples[low]
        second_stamp, second_value = samples[high]
        fraction = (
            (stamp_sec - first_stamp)
            / max(second_stamp - first_stamp, 1.0e-9))
        value = first_value + fraction * (second_value - first_value)
        age = min(
            stamp_sec - first_stamp,
            second_stamp - stamp_sec)
        return value, max(0.0, age), False

    def yaw_at_stamp(self, stamp_sec):
        """Interpolate or briefly extrapolate unwrapped IMU orientation."""
        if len(self.imu_yaw_history) < 2:
            return None, math.inf, False
        samples = list(self.imu_yaw_history)
        if stamp_sec < samples[0][0]:
            return None, math.inf, False
        if stamp_sec >= samples[-1][0]:
            age = stamp_sec - samples[-1][0]
            yaw = samples[-1][1]
            if age > 0.0:
                yaw += (
                    (self.gyro_z - self.gyro_bias)
                    * age * self.motion_rate_scale)
            return yaw, max(0.0, age), True
        low = 0
        high = len(samples) - 1
        while high - low > 1:
            middle = (low + high) // 2
            if samples[middle][0] <= stamp_sec:
                low = middle
            else:
                high = middle
        first_stamp, first_yaw = samples[low]
        second_stamp, second_yaw = samples[high]
        interval = max(second_stamp - first_stamp, 1.0e-9)
        fraction = (stamp_sec - first_stamp) / interval
        yaw = first_yaw + fraction * (second_yaw - first_yaw)
        age = min(stamp_sec - first_stamp, second_stamp - stamp_sec)
        return yaw, max(0.0, age), True

    def model_input_at_stamp(self, stamp_sec):
        """Interpolate delayed speed/steering inputs for the bicycle model."""
        if not self.vehicle_model_history:
            return self.filtered_speed, self.latest_steering, True
        samples = list(self.vehicle_model_history)
        if stamp_sec <= samples[0][0]:
            return samples[0][1], samples[0][2], True
        if stamp_sec >= samples[-1][0]:
            return samples[-1][1], samples[-1][2], False
        low = 0
        high = len(samples) - 1
        while high - low > 1:
            middle = (low + high) // 2
            if samples[middle][0] <= stamp_sec:
                low = middle
            else:
                high = middle
        first = samples[low]
        second = samples[high]
        fraction = (
            (stamp_sec - first[0])
            / max(second[0] - first[0], 1.0e-9))
        speed = first[1] + fraction * (second[1] - first[1])
        steering = first[2] + fraction * (second[2] - first[2])
        return speed, steering, False

    def reset_for_time_jump(self, stamp_sec, arrival):
        """Reset local dead reckoning when MORAI simulation time restarts."""
        # A clock epoch change does not guarantee that the vehicle pose reset.
        # Preserve continuous local odometry and inflate uncertainty instead.
        self.covariance += np.diag([
            1.0, 1.0, math.radians(5.0)**2])
        self.filtered_speed = 0.0
        self.filtered_yaw_rate = 0.0
        self.previous_filtered_speed = 0.0
        self.previous_filtered_yaw_rate = 0.0
        self.last_vehicle_stamp = stamp_sec
        self.last_publish_arrival = arrival
        self.last_stamp_progress_arrival = arrival
        self.stationary_since_stamp = stamp_sec
        self.vehicle_model_history.clear()
        self.orientation_reference_imu_yaw = None
        self.orientation_reference_odometry_yaw = None
        self.last_orientation_sample_yaw = None
        self.last_orientation_sample_stamp = None
        self.orientation_consistency_count = 0
        self.orientation_invalid_streak = 0
        self.orientation_reanchor_pending = False
        self.time_resets += 1
        rospy.logwarn("[Pure odometry] simulation time reset detected")

    def vehicle_callback(self, message):
        # Strict input boundary: do not access message.position or
        # message.heading here.  Only wheel-odometry-equivalent channels are
        # copied out of the simulator status message.
        arrival = time.monotonic()
        velocity_x = float(message.velocity.x)
        raw_wheel_angle = float(message.wheel_angle)
        steering = self.steering_sign * raw_wheel_angle
        if self.wheel_angle_unit == "deg":
            steering = math.radians(steering)
        if not all(math.isfinite(value) for value in (
                velocity_x, steering)):
            return
        if abs(steering) > self.max_steering + math.radians(5.0):
            self.invalid_steering += 1
            return
        steering = min(self.max_steering, max(-self.max_steering, steering))
        # EgoVehicleStatus velocity.x is the vehicle-forward speed.  Do not
        # use velocity.y: lateral simulator state is not wheel-equivalent.
        measured_speed = velocity_x
        stamp = message.header.stamp
        stamp_sec = (
            stamp.to_sec() if stamp != rospy.Time()
            else rospy.Time.now().to_sec())

        with self.lock:
            self.vehicle_messages += 1
            self.last_vehicle_arrival = arrival
            self.latest_raw_wheel_angle = raw_wheel_angle
            if self.last_vehicle_stamp is None:
                self.last_vehicle_stamp = stamp_sec
                self.last_publish_arrival = arrival
                self.last_stamp_progress_arrival = arrival
                self.filtered_speed = (
                    measured_speed)
                self.previous_filtered_speed = self.filtered_speed
                self.latest_steering = steering
                self.vehicle_model_history.append(
                    (stamp_sec, self.filtered_speed, steering))
                self.publish_odometry(message.header.stamp, 0.0)
                return

            previous_vehicle_stamp = self.last_vehicle_stamp
            raw_delta_time = stamp_sec - previous_vehicle_stamp
            if raw_delta_time < -1.0:
                self.reset_for_time_jump(stamp_sec, arrival)
                self.filtered_speed = measured_speed
                self.previous_filtered_speed = measured_speed
                self.latest_steering = steering
                self.publish_odometry(message.header.stamp, 0.0)
                return
            if raw_delta_time <= 0.0:
                self.duplicate_vehicle_stamps += 1
                return
            self.last_vehicle_stamp = stamp_sec
            self.last_publish_arrival = arrival
            self.last_stamp_progress_arrival = arrival
            delta_time = raw_delta_time * self.motion_rate_scale
            if delta_time > self.max_recoverable_gap:
                self.rejected_dt += 1
                self.unobserved_time_sec += delta_time
                uncertainty_distance = max(
                    abs(self.filtered_speed), abs(measured_speed)) * delta_time
                self.covariance += np.diag([
                    max(1.0, uncertainty_distance**2),
                    max(1.0, uncertainty_distance**2),
                    max(math.radians(5.0)**2, (0.2 * delta_time)**2),
                ])
                self.filtered_speed = measured_speed
                self.previous_filtered_speed = measured_speed
                self.filtered_yaw_rate = 0.0
                self.previous_filtered_yaw_rate = 0.0
                self.publish_odometry(message.header.stamp, 0.0)
                return
            if delta_time > self.max_dt:
                self.recovered_time_gaps += 1

            self.previous_filtered_speed = self.filtered_speed
            speed_alpha = first_order_alpha(
                delta_time, self.speed_filter_tau)
            self.filtered_speed += speed_alpha * (
                measured_speed - self.filtered_speed)
            self.latest_steering = steering
            if (
                    self.vehicle_model_history
                    and stamp_sec < self.vehicle_model_history[-1][0] - 1.0):
                self.vehicle_model_history.clear()
            if (
                    not self.vehicle_model_history
                    or stamp_sec > self.vehicle_model_history[-1][0]):
                self.vehicle_model_history.append(
                    (stamp_sec, self.filtered_speed, steering))
            elif stamp_sec == self.vehicle_model_history[-1][0]:
                self.vehicle_model_history[-1] = (
                    stamp_sec, self.filtered_speed, steering)
            model_speed, model_steering, model_warmup = (
                self.model_input_at_stamp(
                    stamp_sec - self.model_input_delay))
            if model_warmup:
                self.model_delay_warmups += 1
            self.latest_model_speed = model_speed
            self.latest_model_steering = model_steering
            denominator = (
                self.model_wheelbase
                + self.understeer_coefficient * model_speed**2)
            model_yaw_rate = (
                model_speed / denominator * math.tan(model_steering))
            self.latest_model_yaw_rate = model_yaw_rate

            interpolated_gyro, raw_imu_stamp_age, extrapolated = (
                self.gyro_at_stamp(stamp_sec))
            imu_stamp_age = raw_imu_stamp_age * self.motion_rate_scale
            self.latest_imu_stamp_skew = imu_stamp_age
            if math.isfinite(imu_stamp_age):
                self.imu_stamp_skew_history.append(imu_stamp_age)
            if extrapolated:
                self.imu_extrapolations += 1
            imu_fresh = (
                interpolated_gyro is not None
                and imu_stamp_age <= self.imu_timeout)
            corrected_gyro = (
                0.0 if interpolated_gyro is None
                else interpolated_gyro - self.gyro_bias)
            self.latest_imu_yaw_rate = corrected_gyro

            if abs(self.filtered_speed) < self.stationary_speed:
                if self.stationary_since_stamp is None:
                    self.stationary_since_stamp = stamp_sec
                stationary_duration = max(
                    0.0,
                    (stamp_sec - self.stationary_since_stamp)
                    * self.motion_rate_scale)
                if (
                        imu_fresh
                        and stationary_duration >= self.stationary_bias_hold
                        and abs(interpolated_gyro) < self.stationary_gyro):
                    bias_alpha = first_order_alpha(delta_time, 2.0)
                    self.gyro_bias += bias_alpha * (
                        interpolated_gyro - self.gyro_bias)
                fused_yaw_rate = 0.0
                self.filtered_speed = 0.0
                self.latest_imu_weight = 0.0
                self.latest_model_disagreement = 0.0
                self.stationary_updates += 1
            elif not imu_fresh:
                self.stationary_since_stamp = None
                fused_yaw_rate = model_yaw_rate
                self.latest_imu_weight = 0.0
                self.imu_stale_fallbacks += 1
            else:
                self.stationary_since_stamp = None
                innovation = corrected_gyro - model_yaw_rate
                model_noise = (
                    self.model_yaw_rate_noise
                    + 0.15 * abs(steering)
                    + 0.0015 * self.filtered_speed**2)
                innovation_variance = (
                    model_noise**2
                    + self.gyro_yaw_rate_noise**2)
                nis = innovation**2 / max(innovation_variance, 1.0e-9)
                self.latest_yaw_rate_nis = nis
                self.latest_model_disagreement = min(
                    1.0,
                    math.sqrt(max(nis, 0.0) / max(self.nis_hard_gate, 1.0e-9)))
                if abs(corrected_gyro) > self.max_abs_gyro:
                    fused_yaw_rate = model_yaw_rate
                    self.latest_imu_weight = 0.0
                    self.imu_outliers += 1
                else:
                    turn_fraction = min(1.0, abs(steering) / 0.20)
                    adaptive_weight = self.imu_weight + (
                        self.imu_weight_max - self.imu_weight
                    ) * turn_fraction
                    if nis > self.nis_gate:
                        # A physically bounded gyro is the direct yaw-rate
                        # observation; high NIS primarily marks tyre/model
                        # disagreement. Trust gyro and inflate process noise.
                        adaptive_weight = self.imu_weight_max
                        self.model_disagreements += 1
                    adaptive_weight = min(
                        self.imu_weight_max,
                        max(self.imu_weight_min, adaptive_weight))
                    fused_yaw_rate = (
                        model_yaw_rate
                        + adaptive_weight * innovation)
                    self.latest_imu_weight = adaptive_weight
                    if (
                            abs(self.filtered_speed) > 3.0
                            and abs(model_yaw_rate) < 0.02
                            and abs(steering) < math.radians(0.5)):
                        # Straight-line non-holonomic bias observation.  Do
                        # not learn bias in a turn: tyre-model error would be
                        # mistaken for gyro bias.
                        raw_innovation = (
                            interpolated_gyro
                            - model_yaw_rate
                            - self.gyro_bias)
                        correction = self.moving_bias_gain * (
                            raw_innovation * delta_time)
                        correction = min(1.0e-4, max(-1.0e-4, correction))
                        self.gyro_bias += correction

            self.previous_filtered_yaw_rate = self.filtered_yaw_rate
            yaw_alpha = first_order_alpha(
                delta_time, self.yaw_rate_filter_tau)
            self.filtered_yaw_rate += yaw_alpha * (
                fused_yaw_rate - self.filtered_yaw_rate)
            if self.filtered_speed == 0.0:
                self.filtered_yaw_rate = 0.0

            orientation_increment = None
            self.latest_orientation_yaw_rate = None
            if self.orientation_primary:
                orientation_yaw, orientation_age, orientation_available = (
                    self.yaw_at_stamp(stamp_sec))
                orientation_fresh = bool(
                    orientation_available
                    and orientation_age * self.motion_rate_scale
                    <= self.orientation_timeout)
                if not orientation_fresh:
                    self.orientation_stale_fallbacks += 1
                else:
                    sensor_rate = None
                    if (
                            self.last_orientation_sample_yaw is not None
                            and self.last_orientation_sample_stamp is not None):
                        orientation_dt = (
                            (stamp_sec - self.last_orientation_sample_stamp)
                            * self.motion_rate_scale)
                        if orientation_dt > 1.0e-6:
                            sensor_rate = (
                                orientation_yaw
                                - self.last_orientation_sample_yaw
                            ) / orientation_dt
                    self.last_orientation_sample_yaw = orientation_yaw
                    self.last_orientation_sample_stamp = stamp_sec
                    rate_valid = bool(
                        sensor_rate is not None
                        and math.isfinite(sensor_rate)
                        and abs(sensor_rate) <= self.max_orientation_rate)
                    gyro_consistent = bool(
                        rate_valid
                        and (
                            not imu_fresh
                            or abs(sensor_rate - corrected_gyro)
                            <= self.orientation_gyro_gate))
                    if rate_valid and gyro_consistent:
                        self.orientation_consistency_count += 1
                        self.orientation_invalid_streak = 0
                    else:
                        self.orientation_consistency_count = 0
                        self.orientation_invalid_streak += 1
                        if self.orientation_invalid_streak >= 3:
                            self.orientation_reanchor_pending = True
                        self.orientation_invalid_fallbacks += 1

                    if (
                            self.orientation_consistency_count >= 5
                            and (
                                self.orientation_reference_imu_yaw is None
                                or self.orientation_reanchor_pending)):
                        self.orientation_reference_imu_yaw = orientation_yaw
                        self.orientation_reference_odometry_yaw = self.yaw
                        if self.orientation_reanchor_pending:
                            self.orientation_reference_resets += 1
                        self.orientation_reanchor_pending = False

                    if (
                            rate_valid
                            and gyro_consistent
                            and self.orientation_reference_imu_yaw is not None):
                        orientation_target = (
                            self.orientation_reference_odometry_yaw
                            + orientation_yaw
                            - self.orientation_reference_imu_yaw)
                        orientation_increment = wrap_angle(
                            orientation_target - self.yaw)
                        self.latest_orientation_yaw_rate = sensor_rate
                        self.filtered_yaw_rate = sensor_rate
                        self.orientation_primary_updates += 1

            mean_yaw_rate = 0.5 * (
                self.previous_filtered_yaw_rate + self.filtered_yaw_rate)
            mean_speed = 0.5 * (
                self.previous_filtered_speed + self.filtered_speed)
            yaw_increment = (
                orientation_increment
                if orientation_increment is not None
                else mean_yaw_rate * delta_time)
            yaw_mid = self.yaw + 0.5 * yaw_increment
            distance = mean_speed * delta_time
            self.x += distance * math.cos(yaw_mid)
            self.y += distance * math.sin(yaw_mid)
            self.yaw = wrap_angle(self.yaw + yaw_increment)
            covariance_steps = max(
                1, int(math.ceil(delta_time / self.max_dt)))
            covariance_dt = delta_time / covariance_steps
            for _ in range(covariance_steps):
                self.propagate_covariance(covariance_dt, yaw_mid)
            self.publish_odometry(message.header.stamp, delta_time)

    def propagate_covariance(self, delta_time, yaw_mid):
        speed = self.filtered_speed
        state_jacobian = np.eye(3)
        state_jacobian[0, 2] = (
            -speed * math.sin(yaw_mid) * delta_time)
        state_jacobian[1, 2] = (
            speed * math.cos(yaw_mid) * delta_time)
        noise_jacobian = np.asarray([
            [math.cos(yaw_mid) * delta_time,
             -0.5 * speed * math.sin(yaw_mid) * delta_time**2],
            [math.sin(yaw_mid) * delta_time,
             0.5 * speed * math.cos(yaw_mid) * delta_time**2],
            [0.0, delta_time],
        ])
        yaw_uncertainty_scale = (
            1.0
            + min(5.0, math.sqrt(max(self.latest_yaw_rate_nis, 0.0)))
            + 3.0 * self.latest_model_disagreement)
        process_noise = np.diag([
            self.speed_noise**2,
            (self.yaw_rate_noise * yaw_uncertainty_scale)**2,
        ])
        self.covariance = (
            state_jacobian @ self.covariance @ state_jacobian.T
            + noise_jacobian @ process_noise @ noise_jacobian.T)
        self.covariance = 0.5 * (
            self.covariance + self.covariance.T)

    def publish_odometry(self, source_stamp, _delta_time):
        stamp = source_stamp if source_stamp != rospy.Time() else rospy.Time.now()
        quaternion = quaternion_from_euler(0.0, 0.0, self.yaw)
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation.x = quaternion[0]
        message.pose.pose.orientation.y = quaternion[1]
        message.pose.pose.orientation.z = quaternion[2]
        message.pose.pose.orientation.w = quaternion[3]
        message.twist.twist.linear.x = self.filtered_speed
        message.twist.twist.angular.z = self.filtered_yaw_rate

        pose_covariance = np.zeros((6, 6), dtype=float)
        pose_covariance[0, 0] = self.covariance[0, 0]
        pose_covariance[0, 1] = self.covariance[0, 1]
        pose_covariance[0, 5] = self.covariance[0, 2]
        pose_covariance[1, 0] = self.covariance[1, 0]
        pose_covariance[1, 1] = self.covariance[1, 1]
        pose_covariance[1, 5] = self.covariance[1, 2]
        pose_covariance[5, 0] = self.covariance[2, 0]
        pose_covariance[5, 1] = self.covariance[2, 1]
        pose_covariance[5, 5] = self.covariance[2, 2]
        pose_covariance[2, 2] = 1.0e3
        pose_covariance[3, 3] = 1.0e3
        pose_covariance[4, 4] = 1.0e3
        message.pose.covariance = pose_covariance.reshape(-1).tolist()
        message.twist.covariance[0] = self.speed_noise**2
        message.twist.covariance[7] = 10.0
        message.twist.covariance[14] = 1.0e3
        message.twist.covariance[21] = 1.0e3
        message.twist.covariance[28] = 1.0e3
        message.twist.covariance[35] = self.yaw_rate_noise**2
        self.publisher.publish(message)
        self.published += 1

        if self.publish_tf:
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = message.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def publish_diagnostics(self, _event):
        with self.lock:
            now = time.monotonic()
            imu_age = None if self.last_imu_arrival is None else max(
                0.0, now - self.last_imu_arrival)
            vehicle_age = None if self.last_vehicle_arrival is None else max(
                0.0, now - self.last_vehicle_arrival)
            stamp_progress_age = (
                None if self.last_stamp_progress_arrival is None else max(
                    0.0, now - self.last_stamp_progress_arrival))
            clock_stalled = bool(
                vehicle_age is not None and vehicle_age < 0.5
                and stamp_progress_age is not None
                and stamp_progress_age > 0.5)
            if vehicle_age is None or vehicle_age >= 0.5:
                mode = "waiting"
            elif clock_stalled:
                mode = "clock_stalled"
            elif (
                    abs(self.filtered_speed) >= self.stationary_speed
                    and self.latest_imu_weight <= 0.0):
                mode = "degraded_model_only"
            else:
                mode = "tracking"
            imu_skew_p95 = (
                None if not self.imu_stamp_skew_history else float(
                    np.percentile(self.imu_stamp_skew_history, 95.0)))
            payload = {
                "mode": mode,
                "x_m": self.x,
                "y_m": self.y,
                "yaw_deg": math.degrees(self.yaw),
                "speed_mps": self.filtered_speed,
                "wheel_angle_raw": self.latest_raw_wheel_angle,
                "wheel_angle_unit": self.wheel_angle_unit,
                "steering_rad": self.latest_steering,
                "gyro_raw_radps": self.gyro_z,
                "gyro_bias_radps": self.gyro_bias,
                "motion_rate_scale": self.motion_rate_scale,
                "time_scale_mode": (
                    "simulation_clock" if self.use_sim_time
                    else "imu_adaptive" if self.adaptive_time_scale
                    else "fixed"),
                "time_scale_candidate": self.latest_time_scale_candidate,
                "time_scale_updates": self.time_scale_updates,
                "uses_sim_time": self.use_sim_time,
                "understeer_coefficient_s2pm": self.understeer_coefficient,
                "physical_wheelbase_m": self.wheelbase,
                "model_wheelbase_m": self.model_wheelbase,
                "model_input_delay_source_s": self.model_input_delay,
                "imu_weight_base": self.imu_weight,
                "imu_weight_applied": self.latest_imu_weight,
                "yaw_rate_nis": self.latest_yaw_rate_nis,
                "model_disagreement_score": self.latest_model_disagreement,
                "imu_yaw_rate_radps": self.latest_imu_yaw_rate,
                "orientation_yaw_rate_radps": (
                    self.latest_orientation_yaw_rate),
                "model_yaw_rate_radps": self.latest_model_yaw_rate,
                "model_speed_mps": self.latest_model_speed,
                "model_steering_rad": self.latest_model_steering,
                "fused_yaw_rate_radps": self.filtered_yaw_rate,
                "imu_age_sec": imu_age,
                "imu_stamp_skew_sec": self.latest_imu_stamp_skew,
                "imu_stamp_skew_p95_sec": imu_skew_p95,
                "vehicle_age_sec": vehicle_age,
                "stamp_progress_age_sec": stamp_progress_age,
                "clock_stalled": clock_stalled,
                "vehicle_messages": self.vehicle_messages,
                "imu_messages": self.imu_messages,
                "published": self.published,
                "rejected_dt": self.rejected_dt,
                "duplicate_vehicle_stamps": self.duplicate_vehicle_stamps,
                "imu_outliers": self.imu_outliers,
                "stationary_updates": self.stationary_updates,
                "imu_stale_fallbacks": self.imu_stale_fallbacks,
                "imu_extrapolations": self.imu_extrapolations,
                "model_disagreements": self.model_disagreements,
                "recovered_time_gaps": self.recovered_time_gaps,
                "unobserved_time_sec": self.unobserved_time_sec,
                "invalid_steering": self.invalid_steering,
                "orientation_primary_updates": (
                    self.orientation_primary_updates),
                "orientation_stale_fallbacks": (
                    self.orientation_stale_fallbacks),
                "orientation_invalid_fallbacks": (
                    self.orientation_invalid_fallbacks),
                "orientation_reference_ready": (
                    self.orientation_reference_imu_yaw is not None),
                "orientation_consistency_count": (
                    self.orientation_consistency_count),
                "orientation_invalid_streak": (
                    self.orientation_invalid_streak),
                "orientation_reference_resets": (
                    self.orientation_reference_resets),
                "model_delay_warmups": self.model_delay_warmups,
                "time_resets": self.time_resets,
                "uses_lidar": False,
                "uses_gps": False,
                "uses_ego_position": False,
                "uses_ego_heading": False,
                "vehicle_fields_used": ["velocity.x", "wheel_angle"],
            }
        self.diagnostics_publisher.publish(
            String(data=json.dumps(payload, sort_keys=True)))


if __name__ == "__main__":
    try:
        PureVehicleOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
