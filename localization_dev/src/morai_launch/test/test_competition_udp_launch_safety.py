#!/usr/bin/env python3

import os
import unittest
import xml.etree.ElementTree as ET


PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SOURCE_DIR = os.path.abspath(os.path.join(PACKAGE_DIR, ".."))
REPOSITORY_DIR = os.path.abspath(os.path.join(SOURCE_DIR, ".."))


def _launch_tree(relative_path):
    return ET.parse(os.path.join(SOURCE_DIR, relative_path)).getroot()


def _arg_default(root, name):
    for element in root.findall("arg"):
        if element.get("name") == name:
            return element.get("default")
    raise AssertionError("launch arg not found: {}".format(name))


class CompetitionUdpLaunchSafetyTest(unittest.TestCase):
    def test_final_localization_uses_competition_status_without_control(self):
        root = _launch_tree(
            "morai_launch/launch/competition_localization_final.launch")
        self.assertEqual(_arg_default(root, "start_connections"), "false")
        self.assertEqual(
            _arg_default(root, "vehicle_state_topic"),
            "/Competition_topic")
        connection = next(
            include for include in root.findall("include")
            if "competition_udp_connections.launch" in include.get(
                "file", ""))
        arguments = {
            item.get("name"): item.get("value")
            for item in connection.findall("arg")}
        self.assertEqual(arguments["enable_control_output"], "false")

    def test_final_local_drive_reuses_connection_and_enables_stack(self):
        root = _launch_tree(
            "competition_autonomy_stack/launch/local_final.launch")
        self.assertEqual(_arg_default(root, "start_connections"), "false")
        self.assertEqual(_arg_default(root, "start_control"), "true")
        self.assertEqual(_arg_default(root, "start_perception"), "true")
        self.assertEqual(_arg_default(root, "rviz"), "true")

    def test_competition_status_speed_is_not_double_fused(self):
        root = _launch_tree(
            "morai_launch/launch/gps_odometry_eskf.launch")
        fusion = next(
            include for include in root.findall("group/include")
            if "sensor_fusion.launch" in include.get("file", ""))
        arguments = {
            item.get("name"): item.get("value")
            for item in fusion.findall("arg")}
        self.assertEqual(arguments["wheel_aiding_enabled"], "true")
        self.assertEqual(arguments["odometry_speed_aiding_enabled"], "false")
        self.assertEqual(arguments["odometry_pose_aiding_enabled"], "true")

    def test_connection_only_launch_cannot_transmit_control_by_default(self):
        root = _launch_tree(
            "morai_launch/launch/competition_udp_connections.launch")
        self.assertEqual(_arg_default(root, "enable_control_output"), "false")

        bridge = next(
            node for node in root.findall("node")
            if node.get("type") == "competition_udp_bridge.py")
        parameters = {
            parameter.get("name"): parameter.get("value")
            for parameter in bridge.findall("param")
        }
        self.assertEqual(
            parameters["enable_control_output"],
            "$(arg enable_control_output)")
        self.assertEqual(parameters["publish_udp_pose_fields"], "false")
        self.assertEqual(parameters["status_topic"], "/Competition_topic")
        self.assertEqual(
            parameters["status_topic_alias"], "/competition_topic")
        self.assertEqual(parameters["control_release_timeout_s"], "1.0")

        includes = list(root.findall("include"))
        sensor_include = next(
            include for include in includes
            if "morai_udp.launch" in include.get("file", ""))
        sensor_arguments = {
            argument.get("name"): argument.get("value")
            for argument in sensor_include.findall("arg")
        }
        self.assertEqual(
            sensor_arguments["udp_source_ip"], "$(arg simulator_ip)")
        self.assertEqual(
            sensor_arguments["udp_bind_ip"], "$(arg bind_ip)")

        camera_include = next(
            include for include in includes
            if "camera_view.launch" in include.get("file", ""))
        camera_arguments = {
            argument.get("name"): argument.get("value")
            for argument in camera_include.findall("arg")
        }
        self.assertEqual(
            camera_arguments["source_ip"], "$(arg simulator_ip)")
        self.assertEqual(
            camera_arguments["bind_ip"], "$(arg bind_ip)")

    def test_integrated_udp_output_follows_start_control_only(self):
        root = _launch_tree(
            "morai_launch/launch/competition_autonomy.launch")
        self.assertEqual(_arg_default(root, "transport"), "udp")
        self.assertEqual(_arg_default(root, "physical_max_steer_deg"), "40.0")
        serialized = ET.tostring(root, encoding="unicode")
        self.assertNotIn("morai_connections.launch", serialized)
        self.assertNotIn("rosbridge_websocket", serialized)
        udp_include = next(
            include for include in root.iter("include")
            if "competition_udp_connections.launch" in
            include.get("file", ""))
        arguments = {
            argument.get("name"): argument.get("value")
            for argument in udp_include.findall("arg")
        }
        self.assertEqual(
            arguments["enable_control_output"], "$(arg start_control)")
        self.assertEqual(_arg_default(root, "start_control"), "false")
        self.assertEqual(_arg_default(root, "global_rejoin_max_kmh"), "10.0")

    def test_full_stack_rejoin_default_remains_hard_capped(self):
        root = _launch_tree(
            "competition_autonomy_stack/launch/full_scenario.launch")
        self.assertEqual(_arg_default(root, "transport"), "udp")
        self.assertEqual(_arg_default(root, "physical_max_steer_deg"), "40.0")
        self.assertEqual(_arg_default(root, "start_control"), "false")
        self.assertEqual(_arg_default(root, "foxglove"), "false")
        self.assertEqual(_arg_default(root, "global_rejoin_max_kmh"), "10.0")

    def test_local_profile_uses_same_udp_gateway_on_loopback(self):
        config_path = os.path.join(
            REPOSITORY_DIR, "config", "local_competition_udp.env")
        with open(config_path, "r", encoding="utf-8") as stream:
            config = stream.read()
        self.assertIn("MORAI_SIM_IP=127.0.0.1", config)
        self.assertIn("MORAI_LOCAL_IP=127.0.0.1", config)
        self.assertIn("MORAI_STATUS_PORT=1909", config)
        self.assertIn("MORAI_CONTROL_PORT=7777", config)
        self.assertIn("MORAI_ENABLE_CONTROL_OUTPUT=true", config)
        self.assertIn("MORAI_PHYSICAL_MAX_STEER_DEG=40.0", config)

        sensor_config_path = os.path.join(
            SOURCE_DIR, "morai_control", "config", "morai_sensor.json")
        with open(sensor_config_path, "r", encoding="utf-8") as stream:
            sensor_config = stream.read()
        for destination_port in (9090, 9091, 9092, 9093, 9094, 9095, 2368):
            self.assertIn(
                '"destinationPort": {}'.format(destination_port),
                sensor_config)

        runner_path = os.path.join(
            REPOSITORY_DIR, "run_local_competition_udp.sh")
        with open(runner_path, "r", encoding="utf-8") as stream:
            runner = stream.read()
        self.assertIn('"$execution_scope" local', runner)

    def test_primary_menu_contains_no_websocket_transport(self):
        menu_path = os.path.join(
            REPOSITORY_DIR, "scripts", "morai_mode_menu.sh")
        with open(menu_path, "r", encoding="utf-8") as stream:
            menu = stream.read().lower()
        self.assertNotIn("rosbridge_websocket.launch", menu)
        self.assertNotIn("transport:=websocket", menu)
        self.assertNotIn("configure_local_websocket", menu)
        self.assertIn("run_local_competition_udp.sh", menu)

    def test_local_and_competition_menus_are_connection_only(self):
        menu_path = os.path.join(
            REPOSITORY_DIR, "scripts", "morai_mode_menu.sh")
        with open(menu_path, "r", encoding="utf-8") as stream:
            menu = stream.read()
        self.assertIn('"$local_network_env" connections', menu)
        self.assertNotIn('"$local_network_env" full', menu)
        self.assertIn('"$network_env" connections', menu)
        self.assertNotIn('"$network_env" full', menu)
        self.assertIn("2|competition|udp)", menu)
        self.assertIn('local_morai_profile="LocalCompetitionUDP.json"', menu)
        self.assertIn("verify_local_morai_network_profile", menu)

        runner_path = os.path.join(
            REPOSITORY_DIR, "run_competition_wired.sh")
        with open(runner_path, "r", encoding="utf-8") as stream:
            runner = stream.read()
        self.assertIn(
            'enable_control_output:="$MORAI_ENABLE_CONTROL_OUTPUT"', runner)
        self.assertNotIn("enable_control_output:=false", runner)

    def test_runtime_connection_launches_never_include_rosbridge(self):
        runtime_launches = (
            "morai_launch/launch/competition_udp_connections.launch",
            "morai_launch/launch/competition_autonomy.launch",
            "morai_launch/launch/morai_connections.launch",
            "morai_launch/launch/morai.launch",
            "morai_launch/launch/hdmap_eskf_lane_change_demo.launch",
            "competition_autonomy_stack/launch/full_scenario.launch",
            "competition_autonomy_stack/launch/test_competition.launch",
            "moraimpc/launch/hdmap_policy_editor.launch",
            "moraimpc/launch/hdmap_intersection_area_editor.launch",
        )
        for relative_path in runtime_launches:
            root = _launch_tree(relative_path)
            includes = [
                element.get("file", "") for element in root.iter("include")]
            self.assertFalse(
                any("rosbridge" in item.lower() for item in includes),
                relative_path)

    def test_udp_runner_rejects_registered_ground_truth_topics(self):
        runner_path = os.path.join(
            REPOSITORY_DIR, "run_competition_wired.sh")
        with open(runner_path, "r", encoding="utf-8") as stream:
            runner = stream.read()
        self.assertIn("^/(Ego_topic|Object_topic)$", runner)
        self.assertIn("기존 MORAI ROS/WebSocket 연결", runner)


if __name__ == "__main__":
    unittest.main()
