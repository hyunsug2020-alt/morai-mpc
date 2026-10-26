#!/usr/bin/env python3
import rospy
from morai_msgs.msg import (EgoVehicleStatus, CollisionData,
    VehicleCollisionData, IntersectionStatus, ObjectStatusList,
    CtrlCmd, EventInfo, GPSMessage)
from sensor_msgs.msg import Imu

rospy.init_node('morai_topic_init')

competition_mode = bool(rospy.get_param('~competition_mode', False))
vehicle_status_topic = (
    '/Competition_topic' if competition_mode else '/Ego_topic')
last_status_time = None


def ego_callback(_msg):
    global last_status_time
    last_status_time = rospy.Time.now()

rospy.Subscriber(vehicle_status_topic,    EgoVehicleStatus,    ego_callback)
rospy.Subscriber('/CollisionData',        CollisionData,       lambda m: None)
rospy.Subscriber('/gps',                  GPSMessage,          lambda m: None)
if not competition_mode:
    rospy.Subscriber('/VehicleCollisionData', VehicleCollisionData,lambda m: None)
    rospy.Subscriber('/InsnStatus',           IntersectionStatus,  lambda m: None)
    rospy.Subscriber('/Object_topic',         ObjectStatusList,    lambda m: None)
    rospy.Subscriber('/imu',                  Imu,                 lambda m: None)

rospy.Publisher('/ctrl_cmd_0',  CtrlCmd,   queue_size=1)
if not competition_mode:
    rospy.Publisher('/InsnControl', EventInfo, queue_size=1)

rospy.loginfo("MORAI 토픽 초기화 완료: vehicle=%s competition_mode=%s",
              vehicle_status_topic, competition_mode)


def connection_watchdog(_event):
    if last_status_time is None:
        rospy.logwarn_throttle(
            10.0,
            "%s 대기 중: MORAI Network 설정을 확인하세요.",
            vehicle_status_topic)
        return
    age = (rospy.Time.now() - last_status_time).to_sec()
    if age > 2.0:
        rospy.logwarn_throttle(
            10.0,
            "%s %.1f초 끊김: 연결 launch는 유지하고 처리 launch만 재시작하세요.",
            vehicle_status_topic, age)


rospy.Timer(rospy.Duration(1.0), connection_watchdog)
rospy.spin()
