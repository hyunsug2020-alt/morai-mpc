#!/usr/bin/env python3
import rospy
from morai_msgs.msg import EventInfo

rospy.init_node('morai_mode_init')
pub = rospy.Publisher('/InsnControl', EventInfo, queue_size=10)
rospy.sleep(1.0)

ev = EventInfo()
ev.ctrl_mode = 3  # 자율주행 모드
ev.gear = 4       # D단

for _ in range(10):
    pub.publish(ev)
    rospy.sleep(0.1)

rospy.loginfo("[ModeInit] MORAI 자율주행 모드 설정 완료 (ctrl_mode=3, gear=4)")
rospy.spin()
