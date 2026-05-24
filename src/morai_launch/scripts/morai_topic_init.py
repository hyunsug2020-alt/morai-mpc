#!/usr/bin/env python3
import rospy
from morai_msgs.msg import (EgoVehicleStatus, CollisionData,
    VehicleCollisionData, IntersectionStatus, IntersectionControl,
    ObjectStatusList, CtrlCmd, EventInfo, GPSMessage, ReplayInfo)
from sensor_msgs.msg import Imu

rospy.init_node('morai_topic_init')

# MORAI -> ROS: Subscriber로 토픽 타입 등록
rospy.Subscriber('/Ego_topic',            EgoVehicleStatus,    lambda m: None)
rospy.Subscriber('/CollisionData',        CollisionData,       lambda m: None)
rospy.Subscriber('/VehicleCollisionData', VehicleCollisionData,lambda m: None)
rospy.Subscriber('/InsnStatus',           IntersectionStatus,  lambda m: None)
rospy.Subscriber('/Object_topic',         ObjectStatusList,    lambda m: None)
rospy.Subscriber('/ReplayInfo_topic',     ReplayInfo,          lambda m: None)
rospy.Subscriber('/imu',                  Imu,                 lambda m: None)
rospy.Subscriber('/gps',                  GPSMessage,          lambda m: None)

# ROS -> MORAI
rospy.Publisher('/ctrl_cmd_0',  CtrlCmd,   queue_size=1)
rospy.Publisher('/InsnControl', EventInfo, queue_size=1)

rospy.loginfo("MORAI 토픽 초기화 완료")
rospy.spin()
