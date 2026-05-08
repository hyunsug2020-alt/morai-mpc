#!/usr/bin/env python3
"""MORAI 후진 기어(R) 값 테스트 스크립트 v3.

서비스 방식으로 기어 전환.
"""
import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv

cur_vel = 0.0

def ego_cb(msg):
    global cur_vel
    cur_vel = msg.velocity.x

rospy.init_node('test_reverse_gear')
ctrl_pub = rospy.Publisher('/ctrl_cmd_0', CtrlCmd, queue_size=1)
# 토픽 방식도 병행
gear_pub = rospy.Publisher('/InsnControl', EventInfo, queue_size=10)
ego_sub = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, ego_cb)

# 서비스 대기
srv_name = '/Service_MoraiEventCmd'
rospy.loginfo("서비스 대기: %s", srv_name)
try:
    rospy.wait_for_service(srv_name, timeout=3.0)
    gear_srv = rospy.ServiceProxy(srv_name, MoraiEventCmdSrv)
    use_srv = True
    rospy.loginfo("서비스 연결 성공")
except:
    use_srv = False
    rospy.logwarn("서비스 없음 → 토픽 방식 사용")

rospy.sleep(1.0)

def set_gear(gear_val, label):
    ev = EventInfo()
    ev.option = 2      # gear 변경 적용
    ev.ctrl_mode = 3
    ev.gear = gear_val
    if use_srv:
        try:
            resp = gear_srv(ev)
            rospy.loginfo("기어 전환(서비스): %s (gear=%d) → resp.gear=%d", label, gear_val, resp.response.gear)
        except Exception as e:
            rospy.logwarn("서비스 호출 실패: %s → 토픽 폴백", e)
            for _ in range(20):
                gear_pub.publish(ev)
                rospy.sleep(0.05)
    else:
        for _ in range(20):
            gear_pub.publish(ev)
            rospy.sleep(0.05)
        rospy.loginfo("기어 전환(토픽): %s (gear=%d)", label, gear_val)

def send_cmd(vel, steer, duration):
    cmd = CtrlCmd()
    cmd.longlCmdType = 2
    cmd.velocity = vel
    cmd.steering = steer
    rate = rospy.Rate(20)
    t0 = rospy.Time.now()
    while (rospy.Time.now() - t0).to_sec() < duration and not rospy.is_shutdown():
        ctrl_pub.publish(cmd)
        rospy.loginfo_throttle(0.5, "  vel_cmd=%.1f | ego_vel=%.2f", vel, cur_vel)
        rate.sleep()

def stop(duration=2.0):
    rospy.loginfo("정지 (%.1f초)", duration)
    send_cmd(0.0, 0.0, duration)

# === 테스트 ===
rospy.loginfo("=== 후진 기어 테스트 v3 ===")

set_gear(4, "D")
rospy.loginfo(">> 전진 5km/h, 3초")
send_cmd(5.0, 0.0, 3.0)

stop(3.0)

set_gear(2, "R")
rospy.sleep(1.0)
rospy.loginfo(">> 후진 5km/h, 4초 (gear=2)")
send_cmd(5.0, 0.0, 4.0)

stop(3.0)
set_gear(4, "D")
rospy.loginfo("=== 완료 ===")
