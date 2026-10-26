#!/usr/bin/env python3
"""Sensor / pipeline health for the competition launch, as one JSON String at 2 Hz.

/competition/sensor_health = {"time": ..., "items": [{"key", "name", "group", "hz", "expected_hz",
"age_s", "state": "ok"|"warn"|"bad"|"off", "detail"}]}

Rates are counted from raw messages (rospy.AnyMsg), so the LiDAR cloud is never deserialized.
Only competition inputs are watched; /Ego_topic is not.
"""
import collections
import json
import threading
import time

import rospy
from std_msgs.msg import Bool, String

# key, display name, group, topic, expected Hz for the competition launch.
TOPICS = [
    ("gps_raw", "GPS 수신 (UDP 1234)", "센서", "/gps_raw", 10.0),
    ("gps", "GPS → 측위 (차단 게이트 통과)", "센서", "/gps", 10.0),
    ("imu", "IMU (UDP 1236)", "센서", "/imu/data", 20.0),
    ("lidar", "LiDAR (UDP 2368)", "센서", "/lidar/points", 15.0),
    ("vehicle", "차량 상태 (Competition_topic)", "센서", "/Competition_topic", 30.0),
    ("objects", "카메라·LiDAR 융합 객체", "인지", "/perception/objects", 15.0),
    ("lidar_vehicles", "LiDAR 차량 추론", "인지", "/perception/lidar_vehicles", 15.0),
    ("odometry", "휠 오도메트리", "측위", "/odometry/pure", 20.0),
    ("localization", "측위 출력 (ego_status)", "측위", "/localization/ego_status", 40.0),
    ("path", "Student 경로", "주행", "/student/path_status", 15.0),
    ("control", "제어 명령 (ctrl_cmd)", "주행", "/ctrl_cmd", 20.0),
]


class Health:
    def __init__(self):
        self.lock = threading.Lock()
        self.stamps = {key: collections.deque(maxlen=400) for key, *_ in TOPICS}
        self.gps = {}
        self.blackout = None
        self.eskf = {}
        self.cameras = {}
        self.cameras_at = 0.0
        self.lidar_vehicles = {}
        self.lidar_vehicles_at = 0.0
        from morai_msgs.msg import GPSMessage
        # rospy shares one transport/deserializer per topic. Register counting
        # and inspection together so AnyMsg cannot replace a typed callback.
        inspected = {"gps_raw": (GPSMessage, self.on_gps),
                     "lidar_vehicles": (String, self.on_lidar_vehicles)}
        for key, _, _, topic, _ in TOPICS:
            message_type, handler = inspected.get(key, (rospy.AnyMsg, None))
            rospy.Subscriber(topic, message_type, self.count(key, handler), queue_size=100, tcp_nodelay=True)
        rospy.Subscriber("/gps_blackout/active", Bool, self.on_blackout, queue_size=5)
        rospy.Subscriber("/eskf/diagnostics", String, self.on_eskf, queue_size=5)
        rospy.Subscriber("/student/camera_rates", String, self.on_cameras, queue_size=5)
        self.pub = rospy.Publisher("/competition/sensor_health", String, queue_size=1, latch=True)
        rospy.Timer(rospy.Duration(0.5), self.publish)

    def count(self, key, handler=None):
        def callback(message):
            with self.lock:
                self.stamps[key].append(time.monotonic())
            if handler is not None:
                handler(message)
        return callback

    def on_gps(self, msg):
        with self.lock:
            self.gps = {"lat": msg.latitude, "lon": msg.longitude, "status": msg.status,
                        "valid": abs(msg.latitude) > 1e-6 or abs(msg.longitude) > 1e-6, "at": time.monotonic()}

    def on_blackout(self, msg):
        self.blackout = bool(msg.data)

    def on_eskf(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.eskf = {"mode": data.get("mode"), "gps_age_s": data.get("gps_age_sec"),
                         "time_scale": data.get("odometry_time_scale"), "at": time.monotonic()}

    def on_cameras(self, msg):
        try:
            self.cameras, self.cameras_at = json.loads(msg.data), time.monotonic()
        except ValueError:
            pass

    def on_lidar_vehicles(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or not isinstance(data.get("vehicles"), list):
                return
            with self.lock:
                self.lidar_vehicles = data
                self.lidar_vehicles_at = time.monotonic()
        except (ValueError, TypeError):
            pass

    @staticmethod
    def rate(stamps, now):
        recent = [t for t in stamps if now - t <= 2.0]
        return len(recent) / 2.0, (now - stamps[-1]) if stamps else None

    def publish(self, _):
        now = time.monotonic()
        items = []
        with self.lock:
            for key, name, group, topic, expected in TOPICS:
                hz, age = self.rate(self.stamps[key], now)
                state = "ok" if hz >= 0.6 * expected else ("warn" if hz > 0 else "bad")
                detail = topic
                if key == "lidar_vehicles":
                    detector = self.lidar_vehicles
                    fresh = bool(detector) and now - self.lidar_vehicles_at <= 1.0
                    if not fresh:
                        state, detail = "bad", "차량 추론 상태 메시지 없음"
                    elif not detector.get("active"):
                        hz, state = 0.0, "off"
                        detail = "현재 구간 추론 비활성 (고속도로·지정 교차로에서만 활성)"
                    elif not detector.get("ready"):
                        hz, state, detail = 0.0, "warn", "차량 추론 입력·측위 정렬 또는 초기화 대기"
                    else:
                        detail = f"차량 {len(detector['vehicles'])}대 · {topic}"
                if key == "gps" and self.blackout:
                    state, detail = "warn", "터널 구간: GPS 의도적으로 차단 중 (IMU·휠로 추정)"
                if key == "gps_raw" and self.gps and now - self.gps["at"] < 2.0:
                    detail = (f"위도 {self.gps['lat']:.6f}, 경도 {self.gps['lon']:.6f}" if self.gps["valid"]
                              else "무효 GPS (위경도 0)")
                    if not self.gps["valid"]:
                        state = "warn"
                items.append({"key": key, "name": name, "group": group, "hz": round(hz, 1),
                              "expected_hz": expected, "age_s": None if age is None else round(age, 2),
                              "state": state, "detail": detail})
            fresh = now - self.cameras_at < 3.0
            for camera, hz in sorted(self.cameras.items() if fresh else ()):
                hz = float(hz) if fresh else 0.0
                items.append({"key": "camera_" + camera, "name": f"카메라 {camera}", "group": "센서",
                              "hz": round(hz, 1), "expected_hz": 10.0, "age_s": None,
                              "state": "ok" if hz >= 6 else ("warn" if hz > 0 else "bad"),
                              "detail": "알파마요 UDP 수신"})
            if not self.cameras or not fresh:
                items.append({"key": "cameras", "name": "카메라 (3대)", "group": "센서", "hz": 0.0,
                              "expected_hz": 10.0, "age_s": None, "state": "off",
                              "detail": "알파마요가 실행 중일 때만 보임"})
            eskf = self.eskf if self.eskf and now - self.eskf["at"] < 3.0 else None
            if eskf:
                mode = eskf["mode"]
                state = "ok" if mode == "tracking" else ("warn" if mode in ("odometry_aided", "initializing") else "bad")
                if self.blackout and mode != "tracking":
                    state = "warn"
                gps_age = eskf["gps_age_s"]
                detail = f"모드 {mode} · 마지막 GPS {gps_age:.1f}초 전" if gps_age is not None else f"모드 {mode}"
                scale = eskf.get("time_scale")
                if scale is not None and not 0.8 <= float(scale) <= 1.2:
                    state, detail = "bad", detail + f" · 시간배율 {float(scale):.2f} (비정상)"
                items.append({"key": "eskf", "name": "측위 필터 상태 (ESKF)", "group": "측위", "hz": None,
                              "expected_hz": None, "age_s": None, "state": state, "detail": detail})
            else:
                items.append({"key": "eskf", "name": "측위 필터 상태 (ESKF)", "group": "측위", "hz": None,
                              "expected_hz": None, "age_s": None, "state": "bad", "detail": "진단 메시지 없음"})
            items.append({"key": "blackout", "name": "GPS 차단 게이트 (터널)", "group": "측위", "hz": None,
                          "expected_hz": None, "age_s": None,
                          "state": "off" if self.blackout is None else ("warn" if self.blackout else "ok"),
                          "detail": "게이트 꺼짐" if self.blackout is None else
                          ("터널 안: GPS 차단 중" if self.blackout else "터널 밖: GPS 통과")})
        self.pub.publish(String(data=json.dumps({"time": time.time(), "items": items}, ensure_ascii=False)))


def main():
    rospy.init_node("sensor_health")
    Health()
    rospy.spin()


if __name__ == "__main__":
    main()
