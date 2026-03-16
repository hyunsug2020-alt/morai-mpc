#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MORAI Ioniq 5 전체 스택 런치
==============================
  1. morai_bridge_node    : MORAI 토픽 변환
  2. eskf_node            : GPS+IMU ESKF 헤딩 (2406.06427v3.pdf)
  3. mpc_path_tracker_cpp : LTV-MPC 전진 추종 (Linear_Time-Varying_MPC.pdf)
  4. parking_manager_node : Hybrid A* + RTI-NMPC 주차
  5. iridescence_gui_node : 통합 Iridescence GUI
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg  = get_package_share_directory('moraimpc')
    cfg  = os.path.join(pkg, 'config', 'ioniq5.yaml')

    enable_gui     = DeclareLaunchArgument('enable_gui',     default_value='true')
    enable_parking = DeclareLaunchArgument('enable_parking', default_value='true')
    initial_mode   = DeclareLaunchArgument('initial_mode',   default_value='FORWARD')

    # 1. MORAI 브릿지
    bridge = Node(package='moraimpc', executable='morai_bridge_node',
                  name='morai_bridge_node', output='screen',
                  parameters=[cfg, {'wheelbase':3.0,'max_steer_deg':35.0,'use_accel_cmd':True}])

    # 2. ESKF
    eskf = Node(package='moraimpc', executable='eskf_node',
                name='eskf_node', output='screen',
                parameters=[cfg, {
                    'imu_topic':               '/handsfree/imu',
                    'gps_fix_topic':           '/gnss_rover/fix',
                    'gps_vel_topic':           '/ublox_gps/fix_velocity',
                    'dual_heading_topic':      '/morai/heading_deg',
                    'dual_heading_valid_topic':'/morai/heading_valid',
                    'dual_heading_accuracy_topic':'/morai/heading_accuracy_deg',
                }])

    # 3. LTV-MPC (bisa 패키지, Ioniq5 파라미터)
    ltv = Node(package='bisa', executable='mpc_path_tracker_cpp',
               name='ltv_mpc_tracker', output='screen',
               parameters=[{
                   'use_ltv_mpc':True, 'prediction_horizon':20, 'time_step':0.2,
                   'wheelbase':3.0, 'weight_position':20.0, 'weight_heading':8.0,
                   'weight_curvature':1.5, 'weight_input':1.0,
                   'max_velocity':30.0, 'min_velocity':0.5,
                   'kappa_min_delta':-0.234, 'kappa_max_delta':0.234,
                   'kappa_blend_alpha':0.15, 'lateral_bound':0.30,
                   'w_lateral_slack_lin':500.0, 'w_lateral_slack_quad':5000.0,
                   'publish_accel_cmd':True,
               }],
               remappings=[('/Ego_pose','/Ego_pose'),('/local_path','/local_path'),('/Accel','/Accel')])

    # 4. 주차 매니저
    parking = Node(package='moraimpc', executable='parking_manager_node',
                   name='parking_manager_node', output='screen',
                   parameters=[cfg, {'initial_mode': LaunchConfiguration('initial_mode')}],
                   condition=IfCondition(LaunchConfiguration('enable_parking')))

    # 5. Iridescence GUI
    gui = Node(package='moraimpc', executable='iridescence_gui_node',
               name='iridescence_gui_node', output='screen',
               condition=IfCondition(LaunchConfiguration('enable_gui')))

    return LaunchDescription([
        enable_gui, enable_parking, initial_mode,
        bridge, eskf, ltv, parking, gui,
    ])
