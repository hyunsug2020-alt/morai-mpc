#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from morai_msgs.msg import GPSMessage
from pyproj import Proj

class ESKFNode:
    def __init__(self):
        rospy.init_node('eskf_node')

        # Subscribers
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.gps_sub = rospy.Subscriber('/gps', GPSMessage, self.gps_callback)

        # Publishers
        self.odom_pub = rospy.Publisher('/eskf/odom', Odometry, queue_size=1)
        
        # UTM Projection
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        # State: [x, y, v_x, v_y, theta]
        self.X = np.zeros((5, 1))
        self.P = np.eye(5) * 1.0
        
        # Noise
        self.Q = np.eye(5) * 0.1
        self.R = np.eye(2) * 2.0  # GPS noise

        self.last_time = rospy.get_time()
        self.is_initialized = False

    def gps_callback(self, msg):
        utm_xy = self.proj_UTM(msg.longitude, msg.latitude)
        z = np.array([[utm_xy[0] - msg.eastOffset], [utm_xy[1] - msg.northOffset]])

        if not self.is_initialized:
            self.X[0:2] = z
            self.is_initialized = True
            return

        # Correction step
        H = np.zeros((2, 5))
        H[0, 0] = 1
        H[1, 1] = 1

        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + self.R)
        self.X = self.X + K @ (z - H @ self.X)
        self.P = (np.eye(5) - K @ H) @ self.P

    def imu_callback(self, msg):
        if not self.is_initialized:
            return

        current_time = rospy.get_time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Get orientation from IMU (as raw comparison)
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        _, _, raw_yaw = euler_from_quaternion(q)

        # Simple Prediction (Constant velocity model + yaw update)
        ax = msg.linear_acceleration.x
        omega = msg.angular_velocity.z

        theta = self.X[4, 0]
        self.X[0] += self.X[2] * dt
        self.X[1] += self.X[3] * dt
        self.X[2] += ax * np.cos(theta) * dt
        self.X[3] += ax * np.sin(theta) * dt
        self.X[4] += omega * dt

        # Jacobian F
        F = np.eye(5)
        F[0, 2] = dt
        F[1, 3] = dt
        F[2, 4] = -ax * np.sin(theta) * dt
        F[3, 4] = ax * np.cos(theta) * dt

        self.P = F @ self.P @ F.T + self.Q

        # Publish Odom
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.X[0, 0]
        odom.pose.pose.position.y = self.X[1, 0]
        
        # 속도 정보 추가 (v_x, v_y)
        odom.twist.twist.linear.x = self.X[2, 0]
        odom.twist.twist.linear.y = self.X[3, 0]
        
        q_new = quaternion_from_euler(0, 0, self.X[4, 0])
        odom.pose.pose.orientation.x = q_new[0]
        odom.pose.pose.orientation.y = q_new[1]
        odom.pose.pose.orientation.z = q_new[2]
        odom.pose.pose.orientation.w = q_new[3]
        
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        ESKFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
