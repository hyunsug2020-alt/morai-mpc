#!/usr/bin/env python3
import rospy
from morai_msgs.msg import (EgoVehicleStatus, CollisionData,
    VehicleCollisionData, IntersectionStatus, ObjectStatusList,
    CtrlCmd, EventInfo, GPSMessage)
from sensor_msgs.msg import Imu

rospy.init_node('morai_topic_init')

last_ego_time = None


def ego_callback(_msg):
    global last_ego_time
    last_ego_time = rospy.Time.now()

rospy.Subscriber('/Ego_topic',            EgoVehicleStatus,    ego_callback)
rospy.Subscriber('/CollisionData',        CollisionData,       lambda m: None)
rospy.Subscriber('/VehicleCollisionData', VehicleCollisionData,lambda m: None)
rospy.Subscriber('/InsnStatus',           IntersectionStatus,  lambda m: None)
rospy.Subscriber('/Object_topic',         ObjectStatusList,    lambda m: None)
rospy.Subscriber('/imu',                  Imu,                 lambda m: None)
rospy.Subscriber('/gps',                  GPSMessage,          lambda m: None)

rospy.Publisher('/ctrl_cmd_0',  CtrlCmd,   queue_size=1)
rospy.Publisher('/InsnControl', EventInfo, queue_size=1)

rospy.loginfo("MORAI 토픽 초기화 완료")


def connection_watchdog(_event):
    if last_ego_time is None:
        rospy.logwarn_throttle(
            10.0,
            "/Ego_topic 대기 중: MORAI Network 설정의 ROS Bridge 연결을 확인하세요.")
        return
    age = (rospy.Time.now() - last_ego_time).to_sec()
    if age > 2.0:
        rospy.logwarn_throttle(
            10.0,
            "/Ego_topic %.1f초 끊김: morai.launch는 유지하고 LIO/실험 런치만 재시작하세요.",
            age)


rospy.Timer(rospy.Duration(1.0), connection_watchdog)
rospy.spin()
