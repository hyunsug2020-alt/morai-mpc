#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cvxpy as cp
import numpy as np
import json
import sys
import time
from nav_msgs.msg import Odometry
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import Float32MultiArray, String
from tf.transformations import euler_from_quaternion

class Robust_LTV_MPC:
    def __init__(self, path_file, target_vel=20.0):
        # Parameters (Based on 5-state model concept)
        self.N = 25 # Increased Horizon
        self.dt = 0.1
        self.L = 2.7
        
        # [x, y, v, yaw]
        self.Q = np.diag([200.0, 200.0, 50.0, 100.0])
        self.R = np.diag([1.0, 10.0]) # [accel, steer]
        
        self.target_vel = target_vel / 3.6 

        with open(path_file) as f:
            data = json.load(f)
        self.waypoints = np.array([[wp['x'], wp['y']] for wp in data['waypoints']])
        self.cur_state = None 
        
        rospy.init_node('robust_mpc_follower')
        self.cmd_pub = rospy.Publisher('/ctrl_cmd_0', CtrlCmd, queue_size=1)
        self.perf_pub = rospy.Publisher('/mpc_performance', Float32MultiArray, queue_size=1)
        self.status_pub = rospy.Publisher('/mpc_status', String, queue_size=1)

        rospy.Subscriber('/eskf/odom', Odometry, self.odom_callback)
        rospy.Timer(rospy.Duration(self.dt), self.control_loop)
        rospy.loginfo("Robust LTV-MPC Follower V2 Started")
        rospy.spin()

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(q)
        self.cur_state = np.array([x, y, v, yaw])

    def get_reference_profile(self):
        dists = np.linalg.norm(self.waypoints - self.cur_state[:2], axis=1)
        idx = np.argmin(dists)
        
        ref_traj = []
        for i in range(self.N + 1):
            k = min(idx + i, len(self.waypoints) - 1)
            wx, wy = self.waypoints[k]
            
            # Yaw Ref
            if k < len(self.waypoints) - 1:
                dy = self.waypoints[k+1, 1] - self.waypoints[k, 1]
                dx = self.waypoints[k+1, 0] - self.waypoints[k, 0]
                yaw_ref = np.arctan2(dy, dx)
            else:
                yaw_ref = self.cur_state[3]
            
            ref_traj.append([wx, wy, self.target_vel, yaw_ref])
        return np.array(ref_traj), dists[idx]

    def control_loop(self, event):
        if self.cur_state is None: return

        ref_traj, tracking_error = self.get_reference_profile()
        start_time = time.time()

        # State: [x, y, v, yaw], Input: [accel, steer]
        x_var = cp.Variable((4, self.N + 1))
        u_var = cp.Variable((2, self.N))
        
        cost = 0
        constraints = [x_var[:, 0] == self.cur_state]
        
        v_0 = max(1.0, self.cur_state[2])
        yaw_0 = self.cur_state[3]

        for t in range(self.N):
            # LTV Model Linearization
            A = np.eye(4)
            A[0, 2] = np.cos(yaw_0) * self.dt
            A[0, 3] = -v_0 * np.sin(yaw_0) * self.dt
            A[1, 2] = np.sin(yaw_0) * self.dt
            A[1, 3] = v_0 * np.cos(yaw_0) * self.dt
            
            B = np.zeros((4, 2))
            B[2, 0] = self.dt
            B[3, 1] = (v_0 / self.L) * self.dt
            
            constraints += [x_var[:, t+1] == A @ x_var[:, t] + B @ u_var[:, t]]
            
            # Yaw difference handling (simple)
            yaw_ref = ref_traj[t+1, 3]
            dyaw = yaw_ref - yaw_0
            dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi
            target_yaw = yaw_0 + dyaw
            
            ref_state = np.array([ref_traj[t+1, 0], ref_traj[t+1, 1], ref_traj[t+1, 2], target_yaw])
            
            cost += cp.quad_form(x_var[:, t+1] - ref_state, self.Q)
            cost += cp.quad_form(u_var[:, t], self.R)
            
            # Physical Constraints
            constraints += [u_var[1, t] <= np.radians(35), u_var[1, t] >= np.radians(-35)]
            constraints += [u_var[0, t] <= 3.0, u_var[0, t] >= -5.0]

        # Use OSQP solver explicitly via CVXPY for speed and robustness
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP, warm_start=True)
        
        solve_time = (time.time() - start_time) * 1000.0

        if prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            accel = u_var[0, 0].value
            steer = u_var[1, 0].value
            cmd_vel = max(0, (self.cur_state[2] + accel * self.dt) * 3.6)
            # MORAI steering: - sign convention
            self.publish_cmd(cmd_vel, -np.degrees(steer))
            
            perf_msg = Float32MultiArray()
            perf_msg.data = [float(tracking_error), float(solve_time), float(prob.value)]
            self.perf_pub.publish(perf_msg)
            self.status_pub.publish(prob.status)
        else:
            rospy.logwarn(f"MPC Infeasible: {prob.status}")

    def publish_cmd(self, vel, steer):
        cmd = CtrlCmd()
        cmd.longlCmdType = 2
        cmd.velocity = vel
        cmd.steering = steer
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    path_file = sys.argv[1] if len(sys.argv) > 1 else '/home/david/recorded_path.json'
    Robust_LTV_MPC(path_file)
