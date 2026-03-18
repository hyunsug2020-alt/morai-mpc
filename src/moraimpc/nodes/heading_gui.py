#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import matplotlib.pyplot as plt
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np

class HeadingGUI:
    def __init__(self):
        rospy.init_node('heading_gui')
        
        self.real_yaw = 0.0
        self.eskf_yaw = 0.0
        
        self.real_history = []
        self.eskf_history = []
        self.time_history = []
        self.start_time = rospy.get_time()

        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.real_callback)
        rospy.Subscriber('/eskf/odom', Odometry, self.eskf_callback)

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.real_line, = self.ax.plot([], [], 'r-', label='Real Heading (Ego_topic)')
        self.eskf_line, = self.ax.plot([], [], 'b-', label='ESKF Heading')
        self.acc_text = self.ax.text(0.02, 0.95, '', transform=self.ax.transAxes, fontsize=12, fontweight='bold', color='darkgreen')
        
        self.ax.legend(loc='upper right')
        self.ax.set_title('Localization Performance (Heading Accuracy)')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Heading (rad)')

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_plot()
            rate.sleep()

    def real_callback(self, msg):
        self.real_yaw = msg.heading * (np.pi / 180.0)

    def eskf_callback(self, msg):
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, self.eskf_yaw = euler_from_quaternion(q)

    def update_plot(self):
        current_time = rospy.get_time() - self.start_time
        self.time_history.append(current_time)
        self.real_history.append(self.real_yaw)
        self.eskf_history.append(self.eskf_yaw)

        # Accuracy Calculation
        error = self.real_yaw - self.eskf_yaw
        error = (error + np.pi) % (2 * np.pi) - np.pi
        accuracy = max(0, 100 * (1 - abs(error) / np.pi))
        self.acc_text.set_text(f'Heading Accuracy: {accuracy:.2f}%')

        # Keep last 100 points
        if len(self.time_history) > 100:
            self.time_history.pop(0)
            self.real_history.pop(0)
            self.eskf_history.pop(0)

        self.real_line.set_data(self.time_history, self.real_history)
        self.eskf_line.set_data(self.time_history, self.eskf_history)
        
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

if __name__ == '__main__':
    try:
        HeadingGUI()
    except rospy.ROSInterruptException:
        pass
