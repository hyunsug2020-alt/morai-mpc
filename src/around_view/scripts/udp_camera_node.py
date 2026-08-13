#!/usr/bin/env python3
"""Publish MORAI UDP camera streams as ROS Image topics."""

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from around_view import UdpCameraReceiver


DEFAULT_CAMERAS = {
    "front": {
        "port": 9092,
        "topic": "/around_view/camera/front/image_raw",
        "frame_id": "around_view_front_camera",
    },
    "left": {
        "port": 9094,
        "topic": "/around_view/camera/left/image_raw",
        "frame_id": "around_view_left_camera",
    },
    "right": {
        "port": 9093,
        "topic": "/around_view/camera/right/image_raw",
        "frame_id": "around_view_right_camera",
    },
    "rear": {
        "port": 9095,
        "topic": "/around_view/camera/rear/image_raw",
        "frame_id": "around_view_rear_camera",
    },
}


class UdpCameraNode:
    def __init__(self):
        self.cameras = rospy.get_param("~cameras", DEFAULT_CAMERAS)
        udp_config = rospy.get_param("~udp", {})
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.show_windows = bool(rospy.get_param("~show_windows", True))
        self.bridge = CvBridge()
        self.gui_available = self.show_windows

        ports = []
        self.camera_by_port = {}
        self.publishers = {}
        self.last_sequences = {}

        for name, config in self.cameras.items():
            port = int(config["port"])
            if port in self.camera_by_port:
                raise ValueError("Duplicate UDP camera port: {}".format(port))
            ports.append(port)
            self.camera_by_port[port] = (name, config)
            self.publishers[name] = rospy.Publisher(
                config["topic"], Image, queue_size=1
            )
            self.last_sequences[port] = 0

        self.receiver = UdpCameraReceiver(
            ports=ports,
            bind_host=udp_config.get("bind_host", "0.0.0.0"),
            packet_format=udp_config.get("packet_format", "auto"),
            receive_buffer_bytes=udp_config.get(
                "receive_buffer_bytes", 1024 * 1024
            ),
            socket_timeout=udp_config.get("socket_timeout", 0.5),
            logger=rospy.loginfo,
        )
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.receiver.stop()
        if self.gui_available:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass

    def _show_frame(self, name, port, frame):
        if not self.gui_available:
            return
        try:
            cv2.imshow("{} camera - UDP {}".format(name, port), frame)
        except cv2.error as error:
            rospy.logwarn("OpenCV windows disabled: %s", error)
            self.gui_available = False

    def run(self):
        self.receiver.start()
        rate = rospy.Rate(self.publish_rate)

        rospy.loginfo(
            "UDP camera node started for %s",
            ", ".join(
                "{}:{}".format(name, config["port"])
                for name, config in self.cameras.items()
            ),
        )

        while not rospy.is_shutdown():
            active_cameras = []

            for port, (name, config) in self.camera_by_port.items():
                frame, received_at, sequence = self.receiver.get_frame(port)
                if frame is None:
                    continue

                active_cameras.append(name)
                if sequence != self.last_sequences[port]:
                    message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    message.header.stamp = rospy.Time.from_sec(received_at)
                    message.header.frame_id = config.get(
                        "frame_id", "{}_camera".format(name)
                    )
                    self.publishers[name].publish(message)
                    self.last_sequences[port] = sequence

                self._show_frame(name, port, frame)

            if len(active_cameras) != len(self.cameras):
                waiting = sorted(set(self.cameras) - set(active_cameras))
                rospy.loginfo_throttle(
                    3.0, "Waiting for UDP cameras: {}".format(", ".join(waiting))
                )

            if self.gui_available:
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    rospy.signal_shutdown("q pressed in camera window")
                    break

            rate.sleep()


def main():
    rospy.init_node("udp_camera")
    node = UdpCameraNode()
    node.run()


if __name__ == "__main__":
    main()
