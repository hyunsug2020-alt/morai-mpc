#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Controllable GPS pass-through for repeatable localization outage tests."""

import json
import threading

import rospy
from morai_msgs.msg import GPSMessage
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


class GpsGateState:
    def __init__(self, enabled=True):
        self.enabled = bool(enabled)
        self.input_messages = 0
        self.output_messages = 0
        self.dropped_messages = 0
        self.transitions = 0

    def set_enabled(self, enabled):
        enabled = bool(enabled)
        changed = enabled != self.enabled
        if changed:
            self.enabled = enabled
            self.transitions += 1
        return changed

    def account_message(self):
        self.input_messages += 1
        if self.enabled:
            self.output_messages += 1
            return True
        self.dropped_messages += 1
        return False


class GpsSignalGate:
    def __init__(self):
        rospy.init_node("gps_signal_gate")
        self.lock = threading.RLock()
        self.input_topic = str(rospy.get_param("~input_topic", "/gps"))
        self.output_topic = str(rospy.get_param(
            "~output_topic", "/gps_gated"))
        if self.input_topic == self.output_topic:
            raise ValueError("GPS gate input_topic and output_topic must differ")
        self.state = GpsGateState(rospy.get_param("~enabled", True))
        self.last_input_time = None
        self.last_output_time = None
        self.last_source_stamp = None

        self.publisher = rospy.Publisher(
            self.output_topic, GPSMessage, queue_size=20)
        self.diagnostics_publisher = rospy.Publisher(
            rospy.get_param("~diagnostics_topic", "/gps_gate/diagnostics"),
            String, queue_size=2, latch=True)
        self.subscriber = rospy.Subscriber(
            self.input_topic, GPSMessage, self._gps_callback,
            queue_size=20, tcp_nodelay=True)
        self.set_service = rospy.Service(
            rospy.get_param("~set_enabled_service", "/gps_gate/set_enabled"),
            SetBool, self._set_enabled)
        self.disconnect_service = rospy.Service(
            rospy.get_param("~disconnect_service", "/gps_gate/disconnect"),
            Trigger, self._disconnect)
        self.connect_service = rospy.Service(
            rospy.get_param("~connect_service", "/gps_gate/connect"),
            Trigger, self._connect)
        self.timer = rospy.Timer(rospy.Duration(0.5), self._timer_callback)
        rospy.loginfo(
            "[GPSGate] %s -> %s initial=%s services=/gps_gate/{disconnect,connect}",
            self.input_topic, self.output_topic,
            "connected" if self.state.enabled else "disconnected")

    def _gps_callback(self, message):
        now = rospy.Time.now().to_sec()
        with self.lock:
            self.last_input_time = now
            self.last_source_stamp = message.header.stamp.to_sec()
            if self.state.account_message():
                self.publisher.publish(message)
                self.last_output_time = now

    def _change(self, enabled):
        with self.lock:
            changed = self.state.set_enabled(enabled)
            state = "connected" if self.state.enabled else "disconnected"
            message = "GPS {}{}".format(
                state, "" if changed else " (already)")
            rospy.logwarn("[GPSGate] %s", message)
            self._publish_diagnostics_locked()
            return changed, message

    def _set_enabled(self, request):
        _, message = self._change(request.data)
        return SetBoolResponse(success=True, message=message)

    def _disconnect(self, _request):
        _, message = self._change(False)
        return TriggerResponse(success=True, message=message)

    def _connect(self, _request):
        _, message = self._change(True)
        return TriggerResponse(success=True, message=message)

    @staticmethod
    def _age(now, stamp):
        return None if stamp is None else max(0.0, now - stamp)

    def _document_locked(self):
        now = rospy.Time.now().to_sec()
        return {
            "stamp": now,
            "enabled": self.state.enabled,
            "state": "connected" if self.state.enabled else "disconnected",
            "input_topic": self.input_topic,
            "output_topic": self.output_topic,
            "input_messages": self.state.input_messages,
            "output_messages": self.state.output_messages,
            "dropped_messages": self.state.dropped_messages,
            "transitions": self.state.transitions,
            "last_input_age_sec": self._age(now, self.last_input_time),
            "last_output_age_sec": self._age(now, self.last_output_time),
            "last_source_stamp": self.last_source_stamp,
        }

    def _publish_diagnostics_locked(self):
        self.diagnostics_publisher.publish(String(data=json.dumps(
            self._document_locked(), separators=(",", ":"))))

    def _timer_callback(self, _event):
        with self.lock:
            self._publish_diagnostics_locked()


if __name__ == "__main__":
    try:
        GpsSignalGate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
