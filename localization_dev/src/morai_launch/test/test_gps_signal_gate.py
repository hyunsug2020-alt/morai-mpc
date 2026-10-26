#!/usr/bin/env python3

import importlib.util
import os
import unittest


MODULE_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "scripts", "gps_signal_gate.py"))
SPEC = importlib.util.spec_from_file_location("gps_signal_gate", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class GpsGateStateTest(unittest.TestCase):
    def test_disconnect_drops_and_connect_forwards(self):
        state = MODULE.GpsGateState(enabled=True)
        self.assertTrue(state.account_message())
        self.assertTrue(state.set_enabled(False))
        self.assertFalse(state.account_message())
        self.assertFalse(state.set_enabled(False))
        self.assertTrue(state.set_enabled(True))
        self.assertTrue(state.account_message())
        self.assertEqual(state.input_messages, 3)
        self.assertEqual(state.output_messages, 2)
        self.assertEqual(state.dropped_messages, 1)
        self.assertEqual(state.transitions, 2)


if __name__ == "__main__":
    unittest.main()
