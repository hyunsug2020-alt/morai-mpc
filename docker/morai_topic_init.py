#!/usr/bin/env python3
import rospy
from morai_msgs.msg import (EgoVehicleStatus, CollisionData, 
                            VehicleCollisionData, MoraiSimProcStatus, 
                            IntersectionStatus, ObjectStatusList, 
                            CtrlCmd, EventInfo)

def init_node():
    rospy.init_node('morai_topic_init')
    
    # 1. MORAI가 쏘는 데이터를 ROS가 받을 준비가 됐다고 광고 (Subscriber)
    rospy.Subscriber('/Ego_topic', EgoVehicleStatus, lambda m: None)
    rospy.Subscriber('/CollisionData', CollisionData, lambda m: None)
    rospy.Subscriber('/VehicleCollisionData', VehicleCollisionData, lambda m: None)
    rospy.Subscriber('/InsnStatus', IntersectionStatus, lambda m: None)
    rospy.Subscriber('/Object_topic', ObjectStatusList, lambda m: None)
    
    # 2. ROS가 MORAI로 보낼 데이터 주소 확보 (Publisher)
    # Noetic(ROS 1)에서는 토픽 이름 뒤에 _0을 붙이지 않는 것이 기본 설정인 경우가 많습니다.
    pub_ctrl = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
    pub_event = rospy.Publisher('/InsnControl', EventInfo, queue_size=1)
    
    rospy.loginfo("✅ [Noetic] MORAI 토픽 초기화 및 광고 중...")
    rospy.spin()

if __name__ == '__main__':
    try:
        init_node()
    except rospy.ROSInterruptException:
        pass
