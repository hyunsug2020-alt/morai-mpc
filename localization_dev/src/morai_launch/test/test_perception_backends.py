#!/usr/bin/env python3
"""Expand launch graphs offline; never start ROS nodes or load model weights."""

import itertools
import unittest
from pathlib import Path

import roslaunch


SOURCE_DIR = Path(__file__).resolve().parents[2]
ENTRIES = (
    "morai_launch/launch/all_object_detection.launch",
    "moraimpc/launch/perception_hdmap_avoidance.launch",
    "morai_launch/launch/competition_autonomy.launch",
    "competition_autonomy_stack/launch/full_scenario.launch",
    "competition_autonomy_stack/launch/local_final.launch",
)
LEGACY_CAMERA = {"camera_vehicle_detector", "camera_human_animal_detector",
                 "camera_obstacle_detector"}
LEGACY_LIDAR = {"lidar_car_detector", "lidar_human_animal_detector",
                "lidar_obstacle_detector"}
ARTIFACTS = {
    "unified_camera_model": "/offline-fixture/camera.pt",
    "unified_lidar_checkpoint": "/offline-fixture/lidar.pth",
    "unified_lidar_model_config": "/offline-fixture/model.yaml",
    "unified_lidar_data_config": "/offline-fixture/data.yaml",
}


def load(entry=ENTRIES[0], camera=False, lidar=False, extra=()):
    config = roslaunch.config.ROSLaunchConfig()
    argv = ["use_unified_camera:=" + str(camera).lower(),
            "use_unified_lidar:=" + str(lidar).lower()]
    argv.extend(name + ":=" + value for name, value in ARTIFACTS.items())
    argv.extend(extra)
    roslaunch.xmlloader.XmlLoader().load(
        str(SOURCE_DIR / entry), config, argv=argv, verbose=False)
    if config.config_errors:
        raise AssertionError(config.config_errors)
    return config


def param(config, suffix):
    # Some entry points place perception under /central; match node-local keys.
    matches = [value.value for key, value in config.params.items()
               if key.endswith("/" + suffix)]
    if len(matches) != 1:
        raise AssertionError("Expected one parameter {}: {}".format(suffix, matches))
    return matches[0]


class PerceptionBackendsTest(unittest.TestCase):
    def test_all_entry_points_forward_each_independent_backend_combination(self):
        for entry, camera, lidar in itertools.product(ENTRIES, (False, True), (False, True)):
            with self.subTest(entry=entry, camera=camera, lidar=lidar):
                config = load(entry, camera, lidar)
                names = [node.name for node in config.nodes]
                self.assertEqual(len(names), len(set(names)))
                expected = {"camera_traffic_light_detector"}
                expected |= {"camera_object_detector"} if camera else LEGACY_CAMERA
                expected |= {"lidar_object_detector"} if lidar else LEGACY_LIDAR
                actual = {node.name for node in config.nodes if node.type in (
                    "camera_vehicle_detector_node.py", "pointpillar_detector.py")}
                self.assertEqual(actual, expected)
                self.assertTrue(all(node.required for node in config.nodes
                                    if node.name in expected))
                self.assertEqual(param(config, "camera_all_detection_fusion/detections_topic"),
                                 "/camera_all_detection/detections")
                self.assertEqual(param(config, "lidar_all_detection_fusion/detections_topic"),
                                 "/lidar_all_detection/detections")
                self.assertTrue(param(config, "lidar_all_detection_fusion/ego_self_filter_enabled"))
                if camera:
                    self.assertEqual(param(config, "camera_object_detector/model_path"),
                                     ARTIFACTS["unified_camera_model"])
                    self.assertEqual(param(config, "camera_object_detector/required_classes"),
                                     ["Car", "Human", "Animal", "Obstacle"])
                    self.assertEqual(param(config, "camera_all_detection_fusion/source_topics/objects"),
                                     "/camera_object_detection/detections")
                if lidar:
                    for key in ("checkpoint", "model_config", "data_config"):
                        self.assertEqual(param(config, "lidar_object_detector/" + key),
                                         ARTIFACTS["unified_lidar_" + key])
                    self.assertTrue(param(config, "lidar_object_detector/strict_checkpoint"))
                    self.assertFalse(param(config, "lidar_object_detector/remove_ground"))
                    self.assertEqual(param(config, "lidar_object_detector/class_filter"),
                                     "Car,Human,Animal,Obstacle")
                    self.assertEqual(param(config, "lidar_all_detection_fusion/sources"),
                                     [{"name": "objects", "topic": "/lidar_all_detection/objects"}])
                if "competition_autonomy_stack" in entry:
                    required = set()
                    for group in ("required_node_suffixes", "required_camera_node_suffixes",
                                  "required_lidar_node_suffixes"):
                        required.update(param(config, "competition_stack_health/" + group))
                    self.assertTrue({"/" + name for name in expected} <= required)
                    disabled = (LEGACY_CAMERA | LEGACY_LIDAR |
                                {"camera_object_detector", "lidar_object_detector"}) - expected
                    self.assertFalse({"/" + name for name in disabled} & required)
                    self.assertIn("/camera_all_detection_fusion", required)
                    self.assertIn("/camera_lidar_safety_fusion", required)
                    self.assertIn("/path_follower_node", required)

    def test_default_launch_keeps_seven_legacy_models(self):
        config = roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(
            str(SOURCE_DIR / ENTRIES[0]), config, verbose=False)
        names = {node.name for node in config.nodes if node.name.endswith("_detector")}
        self.assertEqual(names, LEGACY_CAMERA | LEGACY_LIDAR | {"camera_traffic_light_detector"})

    def test_class_thresholds_survive_shared_inference(self):
        config = load(camera=True, lidar=True, extra=(
            "vehicle_camera_confidence:=0.30", "human_animal_camera_confidence:=0.55",
            "obstacle_camera_confidence:=0.40", "obstacle_lidar_score_threshold:=0.18"))
        self.assertEqual(param(config, "camera_object_detector/confidence"), 0.30)
        self.assertEqual(param(config, "camera_all_detection_fusion/class_confidence_thresholds/human"), 0.55)
        self.assertEqual(param(config, "camera_all_detection_fusion/class_confidence_thresholds/obstacle"), 0.40)
        self.assertEqual(param(config, "lidar_object_detector/score_threshold"), 0.18)
        self.assertEqual(param(config, "lidar_all_detection_fusion/class_score_thresholds/Car"), 0.445)

    def test_ground_preprocessing_is_forwarded_from_top_level(self):
        config = load(ENTRIES[-1], lidar=True, extra=("unified_lidar_remove_ground:=true",))
        self.assertTrue(param(config, "lidar_object_detector/remove_ground"))

    def test_disabled_perception_does_not_start_any_backend(self):
        config = load(ENTRIES[3], camera=True, lidar=True, extra=("start_perception:=false",))
        self.assertFalse(any(node.type in ("camera_vehicle_detector_node.py", "pointpillar_detector.py")
                             for node in config.nodes))

    def test_mode_switch_clears_old_adapter_and_health_parameters(self):
        config = load(ENTRIES[3], camera=True, lidar=True)
        for name in ("camera_all_detection_fusion", "lidar_all_detection_fusion", "competition_stack_health"):
            self.assertTrue(any(ns.rstrip("/").endswith("/" + name)
                                for ns in config.clear_params))


if __name__ == "__main__":
    unittest.main()
