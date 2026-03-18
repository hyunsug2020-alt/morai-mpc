#!/usr/bin/env python3
import rospy
from morai_msgs.msg import CtrlCmd, EventInfo


def main():
    rospy.init_node('ego_drive_node')

    pub_ctrl  = rospy.Publisher('/ctrl_cmd_0', CtrlCmd,   queue_size=10)
    pub_event = rospy.Publisher('/InsnControl', EventInfo, queue_size=10)

    rospy.sleep(1.0)

    # 자율주행 모드 설정
    ev = EventInfo()
    ev.ctrl_mode = 3
    ev.gear      = 4
    for _ in range(10):
        pub_event.publish(ev)
        rospy.sleep(0.1)
    rospy.loginfo("완료! 주행 시작")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cmd = CtrlCmd()
        cmd.longlCmdType = 2
        cmd.velocity = 20.0
        cmd.steering = 0.0
        pub_ctrl.publish(cmd)
        rospy.loginfo("주행 중... v=10 km/h")
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
