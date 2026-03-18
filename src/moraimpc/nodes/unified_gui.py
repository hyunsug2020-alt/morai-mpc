#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String
from tf.transformations import euler_from_quaternion
import numpy as np
import json
import sys

class UnifiedPerformanceGUI:
    def __init__(self, path_file):
        rospy.init_node('unified_performance_gui')
        
        # Data initialization
        self.real_yaw = 0.0
        self.eskf_yaw = 0.0
        self.cur_pos = [0.0, 0.0]
        self.tracking_error = 0.0
        self.solve_time = 0.0
        self.solver_status = "N/A"
        
        # History
        self.real_history = []
        self.eskf_history = []
        self.time_history = []
        self.start_time = rospy.get_time()

        # Load Path
        with open(path_file) as f:
            data = json.load(f)
        self.waypoints = np.array([[wp['x'], wp['y']] for wp in data['waypoints']])

        # Subscribers
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.real_callback)
        rospy.Subscriber('/eskf/odom', Odometry, self.eskf_callback)
        rospy.Subscriber('/mpc_performance', Float32MultiArray, self.mpc_perf_callback)
        rospy.Subscriber('/mpc_status', String, self.mpc_status_callback)

        # Matplotlib Setup
        plt.ion()
        self.fig = plt.figure(figsize=(12, 8))
        gs = gridspec.GridSpec(2, 2, height_ratios=[2, 1])
        
        # 1. Path Plot (Top-left, large)
        self.ax_path = self.fig.add_subplot(gs[0, 0])
        self.ax_path.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'k--', label='Planned Path')
        self.curr_pos_plot, = self.ax_path.plot([], [], 'ro', label='Current Pos (ESKF)')
        self.ax_path.set_title('Global Path & Tracking')
        self.ax_path.set_aspect('equal')
        self.ax_path.legend()

        # 2. Heading Plot (Top-right)
        self.ax_hdg = self.fig.add_subplot(gs[0, 1])
        self.real_line, = self.ax_hdg.plot([], [], 'r-', label='Real Heading')
        self.eskf_line, = self.ax_hdg.plot([], [], 'b-', label='ESKF Heading')
        self.ax_hdg.set_title('Heading Accuracy')
        self.ax_hdg.legend()

        # 3. MPC Performance Text (Bottom)
        self.ax_info = self.fig.add_subplot(gs[1, :])
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(0.05, 0.5, '', fontsize=12, fontweight='bold', family='monospace')

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_plot()
            rate.sleep()

    def real_callback(self, msg):
        self.real_yaw = msg.heading * (np.pi / 180.0)

    def eskf_callback(self, msg):
        self.cur_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, self.eskf_yaw = euler_from_quaternion(q)

    def mpc_perf_callback(self, msg):
        if len(msg.data) >= 2:
            self.tracking_error = msg.data[0]
            self.solve_time = msg.data[1]

    def mpc_status_callback(self, msg):
        self.solver_status = msg.data

    def update_plot(self):
        current_time = rospy.get_time() - self.start_time
        self.time_history.append(current_time)
        self.real_history.append(self.real_yaw)
        self.eskf_history.append(self.eskf_yaw)

        # Update Path Marker
        self.curr_pos_plot.set_data(self.cur_pos[0], self.cur_pos[1])
        
        # Update Heading Plot
        if len(self.time_history) > 100:
            self.time_history.pop(0)
            self.real_history.pop(0)
            self.eskf_history.pop(0)
        self.real_line.set_data(self.time_history, self.real_history)
        self.eskf_line.set_data(self.time_history, self.eskf_history)
        self.ax_hdg.relim()
        self.ax_hdg.autoscale_view()

        # Update Performance Text
        error = self.real_yaw - self.eskf_yaw
        error = (error + np.pi) % (2 * np.pi) - np.pi
        accuracy = max(0, 100 * (1 - abs(error) / np.pi))
        
        info = (f"=== Autonomous Driving Status ===\n"
                f"[ESKF] Heading Accuracy : {accuracy:6.2f} %\n"
                f"[MPC]  Tracking Error   : {self.tracking_error:6.3f} m\n"
                f"[MPC]  Solver Time      : {self.solve_time:6.2f} ms\n"
                f"[MPC]  Solver Status    : {self.solver_status}")
        self.info_text.set_text(info)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

if __name__ == '__main__':
    path_file = sys.argv[1] if len(sys.argv) > 1 else '/home/david/recorded_path.json'
    try:
        UnifiedPerformanceGUI(path_file)
    except rospy.ROSInterruptException:
        pass
