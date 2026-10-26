"""Regression checks for disconnected MORAI launch and resized preset dialogs."""
import subprocess
import unittest
from unittest.mock import patch

import competition_network as network


class NetworkTests(unittest.TestCase):
    def test_connected_skips_gui(self):
        with patch.object(network, "missing_udp_ports", return_value=[]), \
                patch.object(network, "load_network_gui") as gui:
            network.ensure_network()
            gui.assert_not_called()

    def test_disconnected_restores_then_verifies(self):
        with patch.object(network, "missing_udp_ports", side_effect=[["127.0.0.1:9093"], []]) as ports, \
                patch.object(network, "load_network_gui") as gui, patch.object(network.time, "sleep"):
            network.ensure_network()
            gui.assert_called_once_with()
            self.assertEqual(ports.call_count, 2)

    def test_failed_restore_blocks_start(self):
        with patch.object(network, "missing_udp_ports", return_value=["127.0.0.1:9093"]), \
                patch.object(network, "load_network_gui"), patch.object(network.time, "sleep"):
            with self.assertRaisesRegex(RuntimeError, "네트워크 로드 실패.*9093"):
                network.ensure_network()

    def test_other_process_socket_is_not_a_verified_simulator(self):
        request = {"protocol_settings": {"items": [{"items": [
            {"type": "NETWORK_ITEM_TYPE_HOST_IP", "value": "127.0.0.1"},
            {"type": "NETWORK_ITEM_TYPE_HOST_PORT", "value": "9093"}]}]}}
        for owner, expected in (("python3", ["127.0.0.1:9093"]), ("Simulator.x86_64", [])):
            with self.subTest(owner=owner), \
                    patch.object(network.competition_profiles, "network_request", return_value=request), \
                    patch.object(network.subprocess, "run", return_value=subprocess.CompletedProcess(
                        [], 0, f'UNCONN 0 0 127.0.0.1:9093 0.0.0.0:* users:(("{owner}",pid=1,fd=2))\n')):
                self.assertEqual(network.missing_udp_ports(), expected)

    def test_socket_inspection_failure_blocks_start(self):
        with patch.object(network.subprocess, "run", side_effect=subprocess.CalledProcessError(1, "ss")):
            with self.assertRaises(subprocess.CalledProcessError):
                network.ensure_network()

    def test_minimized_window_is_found_and_activated(self):
        info = "Absolute upper-left X: 81\nAbsolute upper-left Y: 64\nWidth: 1120\nHeight: 1016\n"

        def run(args, **kwargs):
            if args[:2] == ["xdotool", "search"]:
                if "--onlyvisible" in args:
                    raise subprocess.CalledProcessError(1, args)
                return subprocess.CompletedProcess(args, 0, "123\n")
            return subprocess.CompletedProcess(args, 0, info if args[0] == "xwininfo" else "")

        with patch.object(network.subprocess, "run", side_effect=run) as commands, \
                patch.object(network, "missing_udp_ports", return_value=[]), patch.object(network.time, "sleep"):
            network.load_network_gui()
            args = [call.args[0] for call in commands.call_args_list]
            self.assertIn(["xdotool", "windowactivate", "--sync", "123"], args)
            self.assertLess(args.index(["xdotool", "windowactivate", "--sync", "123"]),
                            args.index(["xwininfo", "-id", "123"]))

    def test_ambiguous_simulator_windows_are_rejected_before_input(self):
        with patch.object(network.subprocess, "run", return_value=subprocess.CompletedProcess([], 0, "123\n456\n")) as run:
            with self.assertRaisesRegex(RuntimeError, "하나로 식별"):
                network.load_network_gui()
            self.assertEqual(run.call_count, 1)

    def test_canvas_matches_observed_resized_dialog_buttons(self):
        info = "Absolute upper-left X: 81\nAbsolute upper-left Y: 64\nWidth: 1120\nHeight: 1016\n"
        # Positions measured in MORAI's 1120x1016 client, independent of WM frame offsets.
        self.assertEqual(network.canvas_point(info, -208, 273), (520, 731))
        self.assertEqual(network.canvas_point(info, 0, 130), (641, 648))
        self.assertEqual(network.canvas_point(info, 383, -284), (864, 406))

    def test_canvas_uses_height_limit_and_negative_monitor_origin(self):
        info = "Absolute upper-left X: -1920\nAbsolute upper-left Y: 27\nWidth: 1920\nHeight: 540\n"
        self.assertEqual(network.canvas_point(info, -208, 273), (-1064, 434))

    def test_missing_or_zero_geometry_is_rejected(self):
        for info in ("Width: 1120\nHeight: 1016\n",
                     "Absolute upper-left X: 0\nAbsolute upper-left Y: 0\nWidth: 0\nHeight: 10\n"):
            with self.subTest(info=info), self.assertRaises(RuntimeError):
                network.canvas_point(info, 0, 0)


if __name__ == "__main__":
    unittest.main()
