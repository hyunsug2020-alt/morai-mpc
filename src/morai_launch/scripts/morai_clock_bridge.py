#!/usr/bin/env python3
"""Publish MORAI simulation time from the Ego status header.

MORAI's Real Time mode stamps ROS messages with wall time even when the
simulator runs below real time.  In Simulation Time mode the Ego status header
contains the authoritative simulation clock, but this simulator release does
not publish ``/clock`` itself.  This bridge publishes only that header stamp;
no vehicle state enters the clock path.
"""

import rospy
from morai_msgs.msg import EgoVehicleStatus
from rosgraph_msgs.msg import Clock


class MoraiClockBridge:
    def __init__(self):
        rospy.init_node("morai_clock_bridge")
        self.input_topic = rospy.get_param("~input_topic", "/Ego_topic")
        self.max_initial_stamp = float(rospy.get_param(
            "~max_initial_stamp_s", 86400.0))
        self.publisher = rospy.Publisher("/clock", Clock, queue_size=20)
        self.last_stamp = None
        self.rejected_wall_stamps = 0
        self.subscriber = rospy.Subscriber(
            self.input_topic, EgoVehicleStatus, self.callback,
            queue_size=100, tcp_nodelay=True)
        rospy.loginfo(
            "[MORAI clock] %s header -> /clock", self.input_topic)

    def callback(self, message):
        stamp = message.header.stamp
        stamp_sec = stamp.to_sec()
        if stamp == rospy.Time():
            rospy.logwarn_throttle(
                5.0, "[MORAI clock] zero simulation timestamp rejected")
            return
        if self.last_stamp is None and stamp_sec > self.max_initial_stamp:
            self.rejected_wall_stamps += 1
            rospy.logerr_throttle(
                5.0,
                "[MORAI clock] wall-clock stamp %.3f rejected; select "
                "Time Manager > Simulation Time Mode > External Sync Off",
                stamp_sec)
            return
        if self.last_stamp is not None and stamp < self.last_stamp:
            rospy.logwarn(
                "[MORAI clock] simulation time reset: %.3f -> %.3f",
                self.last_stamp.to_sec(), stamp_sec)
        self.last_stamp = stamp
        self.publisher.publish(Clock(clock=stamp))


if __name__ == "__main__":
    try:
        MoraiClockBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
