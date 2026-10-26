#!/usr/bin/env python3
"""Project-side MORAI competition UDP gateway.

The node does not modify MORAI or host network settings.  It receives the two
competition publishers, exposes their permitted ROS messages, and sends only
Ego Ctrl Cmd packets with ctrl_mode=2 and cmd_type=1. Gear is limited to the
competition vehicle's documented R=2 and D=4 values.
"""

import json
import math
import socket
import threading

import rospy
from morai_msgs.msg import CollisionData, CtrlCmd, EgoVehicleStatus, ObjectStatus
from std_msgs.msg import Bool, Int32, String

from morai_launch.competition_udp_protocol import (
    PacketError,
    control_watchdog_action,
    normalize_ctrl_steer,
    pack_ctrl_cmd,
    parse_collision_data,
    parse_vehicle_status,
    vehicle_status_si_values,
)


def _required_port(name):
    value = int(rospy.get_param(name, 0))
    if value < 1 or value > 65535:
        raise ValueError("{} must be in 1..65535".format(name))
    return value


def _valid_ipv4(value, name, allow_any=False):
    value = str(value).strip()
    if not value:
        raise ValueError("{} is required".format(name))
    try:
        socket.inet_aton(value)
    except OSError as error:
        raise ValueError("{} must be an IPv4 address".format(name)) from error
    if not allow_any and value == "0.0.0.0":
        raise ValueError("{} cannot be 0.0.0.0".format(name))
    return value


class CompetitionUdpBridge:
    def __init__(self):
        rospy.init_node("competition_udp_bridge")

        self.simulator_ip = _valid_ipv4(
            rospy.get_param("~simulator_ip", ""), "~simulator_ip")
        self.bind_ip = _valid_ipv4(
            rospy.get_param("~bind_ip", "0.0.0.0"), "~bind_ip",
            allow_any=True)
        self.status_port = _required_port("~status_port")
        self.collision_port = _required_port("~collision_port")
        self.control_port = _required_port("~control_port")
        self.control_bind_port = int(rospy.get_param("~control_bind_port", 0))
        if not 0 <= self.control_bind_port <= 65535:
            raise ValueError("control_bind_port must be in 0..65535")
        if self.status_port == self.collision_port:
            raise ValueError("status_port and collision_port must differ")

        self.command_topic = str(rospy.get_param(
            "~command_topic", "/ctrl_cmd"))
        self.gear_topic = str(rospy.get_param(
            "~gear_topic", "/competition/gear_cmd"))
        self.status_topic = str(rospy.get_param(
            "~status_topic", "/Competition_topic"))
        self.status_topic_alias = str(rospy.get_param(
            "~status_topic_alias", "/competition_topic"))
        self.collision_topic = str(rospy.get_param(
            "~collision_topic", "/CollisionData"))
        self.send_rate_hz = max(
            10.0, float(rospy.get_param("~send_rate_hz", 50.0)))
        self.command_timeout_s = max(
            0.05, float(rospy.get_param("~command_timeout_s", 0.25)))
        self.status_timeout_s = max(
            0.1, float(rospy.get_param("~status_timeout_s", 0.5)))
        self.control_release_timeout_s = max(
            self.command_timeout_s + 0.05,
            self.status_timeout_s + 0.05,
            float(rospy.get_param("~control_release_timeout_s", 1.0)))
        self.physical_max_steer_deg = float(rospy.get_param(
            "~physical_max_steer_deg", 35.0))
        if (not math.isfinite(self.physical_max_steer_deg) or
                self.physical_max_steer_deg <= 0.0):
            raise ValueError(
                "~physical_max_steer_deg must be a positive number")
        self.physical_max_steer_rad = math.radians(
            self.physical_max_steer_deg)
        self.enable_control_output = bool(rospy.get_param(
            "~enable_control_output", False))
        self.publish_udp_pose_fields = bool(rospy.get_param(
            "~publish_udp_pose_fields", False))
        if self.publish_udp_pose_fields:
            raise ValueError(
                "competition status does not provide position fields")
        self.max_status_clock_skew_s = max(0.0, float(rospy.get_param(
            "~max_status_clock_skew_s", 10.0)))
        self.timestamp_source = str(rospy.get_param("~timestamp_source", "auto"))
        if self.timestamp_source not in ("auto", "arrival"):
            raise ValueError("timestamp_source must be auto or arrival")

        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest_command = None
        self.latest_command_time = None
        self.latest_gear = 4
        self.latest_status_time = None
        self.valid_status_packets = 0
        self.valid_collision_packets = 0
        self.invalid_status_packets = 0
        self.invalid_collision_packets = 0
        self.status_timestamp_fallbacks = 0
        self.latest_speed_mps = 0.0
        self.latest_status_packet_size = None
        self.control_started = False
        self.collision_object_ids = []

        self.status_socket = self._make_receive_socket(self.status_port)
        self.collision_socket = self._make_receive_socket(self.collision_port)
        self.control_socket = (
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if self.enable_control_output else None)
        if self.control_socket is not None and self.control_bind_port:
            self.control_socket.bind((self.bind_ip, self.control_bind_port))

        self.status_pub = rospy.Publisher(
            self.status_topic, EgoVehicleStatus, queue_size=10)
        self.status_alias_pub = (
            rospy.Publisher(
                self.status_topic_alias, EgoVehicleStatus, queue_size=10)
            if self.status_topic_alias and
            self.status_topic_alias != self.status_topic else None)
        self.collision_pub = rospy.Publisher(
            self.collision_topic, CollisionData, queue_size=10)
        self.collision_stop_pub = rospy.Publisher(
            "/competition_udp/collision_stop", Bool, queue_size=1, latch=True)
        self.collision_stop_pub.publish(Bool(data=False))
        self.ready_pub = rospy.Publisher(
            "/competition_udp/ready", Bool, queue_size=1, latch=True)
        self.diagnostics_pub = rospy.Publisher(
            "/competition_udp/status", String, queue_size=1, latch=True)
        self.command_sub = None
        self.gear_sub = None
        if self.enable_control_output:
            self.command_sub = rospy.Subscriber(
                self.command_topic, CtrlCmd, self._command_callback,
                queue_size=1)
            self.gear_sub = rospy.Subscriber(
                self.gear_topic, Int32, self._gear_callback, queue_size=1)

        self.threads = [
            threading.Thread(
                target=self._status_loop, name="morai-status-udp",
                daemon=True),
            threading.Thread(
                target=self._collision_loop, name="morai-collision-udp",
                daemon=True),
        ]
        for thread in self.threads:
            thread.start()

        self.send_timer = None
        if self.enable_control_output:
            self.send_timer = rospy.Timer(
                rospy.Duration(1.0 / self.send_rate_hz), self._send_command)
        self.health_timer = rospy.Timer(
            rospy.Duration(0.5), self._publish_health)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "[CompetitionUDP] bind=%s status=%d collision=%d -> %s:%d; "
            "control_output=%s CtrlMode=2 Gear=R/D CmdType=1",
            self.bind_ip, self.status_port, self.collision_port,
            self.simulator_ip, self.control_port,
            self.enable_control_output)

    def _make_receive_socket(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
        sock.settimeout(0.2)
        sock.bind((self.bind_ip, port))
        return sock

    def _source_is_simulator(self, address):
        if address[0] == self.simulator_ip:
            return True
        rospy.logwarn_throttle(
            5.0, "[CompetitionUDP] unexpected UDP source %s; expected %s",
            address[0], self.simulator_ip)
        return False

    def _status_loop(self):
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            try:
                packet, address = self.status_socket.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if not self.stop_event.is_set():
                    rospy.logerr("[CompetitionUDP] status socket closed")
                break
            if not self._source_is_simulator(address):
                continue
            try:
                status = parse_vehicle_status(packet)
            except PacketError as error:
                with self.lock:
                    self.invalid_status_packets += 1
                rospy.logwarn_throttle(
                    5.0, "[CompetitionUDP] invalid vehicle status: %s", error)
                continue
            self._publish_vehicle_status(status)
            with self.lock:
                self.latest_status_time = rospy.Time.now()
                self.valid_status_packets += 1

    def _collision_loop(self):
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            try:
                packet, address = self.collision_socket.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if not self.stop_event.is_set():
                    rospy.logerr("[CompetitionUDP] collision socket closed")
                break
            if not self._source_is_simulator(address):
                continue
            try:
                collision = parse_collision_data(packet)
            except PacketError as error:
                with self.lock:
                    self.invalid_collision_packets += 1
                rospy.logwarn_throttle(
                    5.0, "[CompetitionUDP] invalid CollisionData: %s", error)
                continue
            self._publish_collision(collision)
            with self.lock:
                self.valid_collision_packets += 1

    def _status_stamp(self, status, arrival):
        if self.timestamp_source == "arrival":
            return arrival
        sec = int(status["sec"])
        nsec = int(status["nsec"])
        if sec <= 0 or nsec < 0 or nsec >= 1000000000:
            self.status_timestamp_fallbacks += 1
            return arrival
        wire_stamp = rospy.Time(sec, nsec)
        if (self.max_status_clock_skew_s > 0.0 and
                abs((arrival - wire_stamp).to_sec()) >
                self.max_status_clock_skew_s):
            self.status_timestamp_fallbacks += 1
            return arrival
        return wire_stamp

    def _publish_vehicle_status(self, status):
        arrival = rospy.Time.now()
        message = EgoVehicleStatus()
        message.header.stamp = self._status_stamp(status, arrival)
        message.header.frame_id = "base_link"
        # The UDP field at this position is map_data_id, not ego unique_id.
        # Do not publish a false identity into the ROS compatibility message.
        message.unique_id = 0
        message.accel = float(status["accel"])
        message.brake = float(status["brake"])
        if hasattr(message, "gear"):
            message.gear = int(status["gear"])
        if hasattr(message, "ctrl_mode"):
            message.ctrl_mode = int(status["ctrl_mode"])
        # Competition Vehicle Status provides only longitudinal velocity.
        # Explicitly zero every unavailable ROS compatibility field so no
        # downstream component can consume simulator ground-truth placeholders.
        si_values = vehicle_status_si_values(status)
        speed_mps = si_values["signed_velocity_mps"]
        message.velocity.x = speed_mps
        message.velocity.y = 0.0
        message.velocity.z = 0.0
        (message.angular_velocity.x,
         message.angular_velocity.y,
         message.angular_velocity.z) = status["angular_velocity"]
        message.acceleration.x = 0.0
        message.acceleration.y = 0.0
        message.acceleration.z = 0.0
        steer_deg = si_values["steer_deg"]
        if hasattr(message, "front_steer"):
            message.front_steer = steer_deg
        if hasattr(message, "front_steer_angle"):
            message.front_steer_angle = steer_deg
        if hasattr(message, "wheel_angle"):
            message.wheel_angle = steer_deg
        self.status_pub.publish(message)
        if self.status_alias_pub is not None:
            self.status_alias_pub.publish(message)
        self.latest_speed_mps = speed_mps
        self.latest_status_packet_size = int(status["packet_size"])

    def _publish_collision(self, collision):
        message = CollisionData()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "ego_vehicle"
        records = collision["records"]
        if records:
            with self.lock:
                self.collision_object_ids = sorted(set(
                    self.collision_object_ids + [int(r["object_id"]) for r in records]))
            # Record contact without overriding the current perception/path.
            rospy.logwarn_throttle(5.0, "[CompetitionUDP] contact recorded; object IDs=%s",
                                   self.collision_object_ids)
        if records:
            offsets = records[0]["global_offset"]
            message.global_offset_x = offsets[0]
            message.global_offset_y = offsets[1]
            message.global_offset_z = offsets[2]
        for record in records:
            obj = ObjectStatus()
            obj.unique_id = int(record["object_id"])
            obj.type = int(record["object_type"])
            obj.name = "collision_object_{}".format(record["object_id"])
            obj.position.x, obj.position.y, obj.position.z = record["position"]
            message.collision_object.append(obj)
        self.collision_pub.publish(message)

    def _command_callback(self, message):
        if int(message.longlCmdType) != 1:
            rospy.logwarn_throttle(
                5.0,
                "[CompetitionUDP] /ctrl_cmd longlCmdType=%d; competition "
                "wire output remains CmdType=1",
                message.longlCmdType)
        if hasattr(message, "steering"):
            steering = float(message.steering)
        elif hasattr(message, "front_steer"):
            steering = float(message.front_steer)
        else:
            rospy.logerr_throttle(
                5.0, "[CompetitionUDP] CtrlCmd steering field missing")
            return
        with self.lock:
            self.latest_command = (
                float(message.accel), float(message.brake),
                steering)
            self.latest_command_time = rospy.Time.now()

    def _gear_callback(self, message):
        gear = int(message.data)
        if gear not in (2, 4):
            rospy.logwarn_throttle(
                5.0, "[CompetitionUDP] rejected gear=%d; only R=2/D=4", gear)
            return
        with self.lock:
            self.latest_gear = gear

    def _send_packet(self, command, gear=None):
        if not self.enable_control_output or self.control_socket is None:
            raise PacketError("competition control output is disabled")
        if gear is None:
            with self.lock:
                gear = self.latest_gear
        # ROS CtrlCmd.steering is a front-wheel angle in radians. MORAI's raw
        # UDP EgoCtrlCmd uses a normalized steering command in [-1, 1].
        normalized_steer = normalize_ctrl_steer(
            command[2], self.physical_max_steer_rad)
        packet = pack_ctrl_cmd(
            accel=command[0], brake=command[1], steer=normalized_steer,
            ctrl_mode=2, gear=gear, cmd_type=1)
        self.control_socket.sendto(
            packet, (self.simulator_ip, self.control_port))

    def _send_command(self, _event):
        if self.stop_event.is_set():
            return
        now = rospy.Time.now()
        with self.lock:
            command = self.latest_command
            command_time = self.latest_command_time
            status_time = self.latest_status_time
            gear = self.latest_gear

        status_age_s = (None if status_time is None else
                        (now - status_time).to_sec())
        command_age_s = (None if command_time is None else
                         (now - command_time).to_sec())
        action = control_watchdog_action(
            self.control_started, status_age_s, command_age_s,
            self.status_timeout_s, self.command_timeout_s,
            self.control_release_timeout_s)

        if action == "wait":
            return
        if action == "release":
            self.control_started = False
            with self.lock:
                self.latest_command = None
                self.latest_command_time = None
            rospy.loginfo(
                "[CompetitionUDP] stale control released; MORAI mode switch enabled")
            return
        if not self.control_started:
            self.control_started = True
            rospy.loginfo("[CompetitionUDP] validated status; control started")

        output = command if action == "command" else (0.0, 1.0, 0.0)
        try:
            self._send_packet(output, gear=gear)
        except (OSError, PacketError) as error:
            rospy.logerr_throttle(
                2.0, "[CompetitionUDP] control send failed: %s", error)

    def _publish_health(self, _event):
        now = rospy.Time.now()
        with self.lock:
            status_time = self.latest_status_time
            command_time = self.latest_command_time
            gear = self.latest_gear
            counters = {
                "valid_status_packets": self.valid_status_packets,
                "valid_collision_packets": self.valid_collision_packets,
                "invalid_status_packets": self.invalid_status_packets,
                "invalid_collision_packets": self.invalid_collision_packets,
                "collision_stop_latched": False,
                "collision_object_ids": list(self.collision_object_ids),
            }
        status_age = None if status_time is None else (
            now - status_time).to_sec()
        command_age = None if command_time is None else (
            now - command_time).to_sec()
        ready = (
            status_age is not None and status_age <= self.status_timeout_s)
        details = {
            "ready": ready,
            "control_started": self.control_started,
            "control_release_timeout_s": self.control_release_timeout_s,
            "simulator_ip": self.simulator_ip,
            "status_age_s": status_age,
            "command_age_s": command_age,
                "command_gear": gear,
                "physical_max_steer_deg": self.physical_max_steer_deg,
                "control_output_enabled": self.enable_control_output,
                "publish_udp_pose_fields": self.publish_udp_pose_fields,
                "status_speed_mps": self.latest_speed_mps,
                "status_packet_size": self.latest_status_packet_size,
                "status_timestamp_fallbacks":
                    self.status_timestamp_fallbacks,
            }
        details.update(counters)
        self.ready_pub.publish(Bool(data=ready))
        self.diagnostics_pub.publish(String(data=json.dumps(
            details, sort_keys=True, separators=(",", ":"))))
        if not ready:
            rospy.logwarn_throttle(
                5.0,
                "[CompetitionUDP] waiting for valid #MoraiInfo$ on %s:%d",
                self.bind_ip, self.status_port)

    def shutdown(self):
        if self.stop_event.is_set():
            return
        self.stop_event.set()
        if self.send_timer is not None:
            self.send_timer.shutdown()
        self.health_timer.shutdown()
        if self.control_started:
            for _unused in range(5):
                try:
                    self._send_packet((0.0, 1.0, 0.0))
                except (OSError, PacketError):
                    break
        self.status_socket.close()
        self.collision_socket.close()
        if self.control_socket is not None:
            self.control_socket.close()


if __name__ == "__main__":
    try:
        CompetitionUdpBridge()
        rospy.spin()
    except (OSError, ValueError) as error:
        rospy.logfatal("[CompetitionUDP] configuration/socket error: %s", error)
        raise SystemExit(2)
    except rospy.ROSInterruptException:
        pass
