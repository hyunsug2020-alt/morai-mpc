"""Competition health must distinguish disabled inference from missing inputs."""
import collections
import json
import threading
import unittest
from unittest.mock import Mock, patch

from std_msgs.msg import String

import sensor_health
from sensor_health import Health, TOPICS


class CompetitionPerceptionHealthTests(unittest.TestCase):
    def health(self):
        health = Health.__new__(Health)
        health.lock = threading.Lock()
        health.stamps = {key: collections.deque() for key, *_ in TOPICS}
        health.stamps["lidar_vehicles"].extend([99.9] * 30)
        health.gps, health.eskf, health.cameras = {}, {}, {}
        health.blackout, health.cameras_at = None, 0.0
        health.lidar_vehicles, health.lidar_vehicles_at = {}, 0.0
        health.pub = Mock()
        return health

    def item(self, health):
        with patch("sensor_health.time.monotonic", return_value=100.0):
            health.publish(None)
        data = json.loads(health.pub.publish.call_args.args[0].data)
        return next(item for item in data["items"] if item["key"] == "lidar_vehicles")

    def receive(self, health, active=True, ready=True, vehicles=None):
        with patch("sensor_health.time.monotonic", return_value=99.9):
            health.on_lidar_vehicles(String(data=json.dumps(dict(
                active=active, ready=ready, vehicles=vehicles or []))))

    def test_watch_the_actual_fusion_output_without_simulator_object_truth(self):
        topics = {key: topic for key, _, _, topic, _ in TOPICS}
        self.assertEqual(topics["objects"], "/perception/objects")
        self.assertNotIn("/Object_topic", topics.values())

    def test_inspected_topics_use_one_typed_transport(self):
        from morai_msgs.msg import GPSMessage
        with patch.object(sensor_health.rospy, "Subscriber") as subscriber, \
                patch.object(sensor_health.rospy, "Publisher"), \
                patch.object(sensor_health.rospy, "Timer"):
            health = Health()
        for topic, message_type in (("/gps_raw", GPSMessage),
                                    ("/perception/lidar_vehicles", String)):
            calls = [call for call in subscriber.call_args_list if call.args[0] == topic]
            self.assertEqual(len(calls), 1)
            self.assertIs(calls[0].args[1], message_type)
        callback = next(call.args[2] for call in subscriber.call_args_list
                        if call.args[0] == "/perception/lidar_vehicles")
        with patch("sensor_health.time.monotonic", return_value=99.9):
            callback(String(data='{"active":false,"ready":false,"vehicles":[]}'))
        self.assertEqual(len(health.stamps["lidar_vehicles"]), 1)
        self.assertEqual(self.item(health)["state"], "off")

    def test_inactive_detector_heartbeats_are_not_vehicle_inference(self):
        health = self.health()
        self.receive(health, active=False, ready=False)
        item = self.item(health)
        self.assertEqual(item["state"], "off")
        self.assertEqual(item["hz"], 0.0)
        self.assertIn("현재 구간", item["detail"])

    def test_empty_ready_detection_is_healthy(self):
        health = self.health()
        self.receive(health)
        item = self.item(health)
        self.assertEqual(item["state"], "ok")
        self.assertEqual(item["hz"], 15.0)
        self.assertIn("차량 0대", item["detail"])

    def test_warmup_does_not_claim_successful_inference(self):
        health = self.health()
        self.receive(health, ready=False)
        item = self.item(health)
        self.assertEqual(item["state"], "warn")
        self.assertEqual(item["hz"], 0.0)

    def test_stale_inactive_record_is_a_failure_not_intentional_disable(self):
        health = self.health()
        self.receive(health, active=False, ready=False)
        health.lidar_vehicles_at = 98.0
        self.assertEqual(self.item(health)["state"], "bad")

    def test_malformed_status_cannot_replace_the_last_valid_state(self):
        health = self.health()
        self.receive(health)
        for data in ("[]", '{"vehicles":null}', "broken"):
            health.on_lidar_vehicles(String(data=data))
        self.assertEqual(self.item(health)["state"], "ok")


if __name__ == "__main__":
    unittest.main()
