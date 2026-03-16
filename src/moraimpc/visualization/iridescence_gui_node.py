#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MORAI Ioniq5 - Iridescence 통합 GUI
=====================================
Iridescence 3D 뷰 + ImGui 패널:
  좌상단: 차량 상태 (속도, 방향, 모드)
  좌중간: IMU (roll/pitch/yaw, gyro, accel)
  우상단: GPS (정확도, Fix)
  우중간: ESKF 헤딩 (융합 + 불확도)
  우하단: 주차 진행

3D 씬:
  - 차량 콘 (위치 + 방향)
  - GPS 궤적 (초록선)
  - 전진 경로 (노란선)
  - 주차 경로 (빨간선, Hybrid A*)
  - MPC 예측 궤적 (파란선)
"""
from __future__ import annotations
import math, threading
from dataclasses import dataclass, field
from typing import List, Tuple
from copy import copy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64, String

try:
    from pyridescence import guik, glk
    import imgui
    _HAS_IR = True
except ImportError:
    _HAS_IR = False


@dataclass
class Snap:
    imu_stamp:float=0.; roll:float=0.; pitch:float=0.; yaw_imu:float=0.
    gx:float=0.; gy:float=0.; gz:float=0.; ax:float=0.; ay:float=0.; az:float=0.
    gps_stamp:float=0.; lat:float=0.; lon:float=0.; hacc:float=99.; gps_st:int=-1
    eskf_stamp:float=0.; eskf_hdg:float=0.; eskf_cov:float=99.
    ox:float=0.; oy:float=0.; oyaw:float=0.; ovx:float=0.
    park_state:str='FORWARD'
    lp:List[Tuple]=field(default_factory=list)
    pp:List[Tuple]=field(default_factory=list)
    mp:List[Tuple]=field(default_factory=list)
    trail:List[Tuple]=field(default_factory=list)


class IriNode(Node):
    STALE=1.5; TRAIL=500

    def __init__(self):
        super().__init__('iridescence_gui_node')
        self._s=Snap(); self._lk=threading.Lock()
        sq=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST,depth=10)
        self.create_subscription(Imu,        '/handsfree/imu',       self._imu, sq)
        self.create_subscription(NavSatFix,  '/gnss_rover/fix',      self._gps, sq)
        self.create_subscription(Odometry,   '/odometry/filtered',   self._odom, sq)
        self.create_subscription(PoseStamped,'/Ego_pose',            self._ego, sq)
        self.create_subscription(Float64,    '/eskf/heading_deg',    self._ehdg, 10)
        self.create_subscription(Float64,    '/eskf/covariance_trace',self._ecov, 10)
        self.create_subscription(String,     '/parking_state',       self._pst, 10)
        self.create_subscription(Path,       '/local_path',          self._lpath, 10)
        self.create_subscription(Path,       '/parking_path',        self._ppath, 10)
        self.create_subscription(Path,       '/mpc_predicted_path',  self._mpath, 10)

    def _now(self): return self.get_clock().now().nanoseconds*1e-9

    def _qe(self,x,y,z,w):
        r=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        p=math.asin(max(-1.,min(1.,2*(w*y-z*x))))
        yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        return r,p,yaw

    def _imu(self,m:Imu):
        r,p,y=self._qe(m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w)
        with self._lk:
            s=self._s; s.imu_stamp=self._now()
            s.roll=math.degrees(r); s.pitch=math.degrees(p); s.yaw_imu=math.degrees(y)
            s.gx=m.angular_velocity.x; s.gy=m.angular_velocity.y; s.gz=m.angular_velocity.z
            s.ax=m.linear_acceleration.x; s.ay=m.linear_acceleration.y; s.az=m.linear_acceleration.z

    def _gps(self,m:NavSatFix):
        h=99.;
        if len(m.position_covariance)>=1 and m.position_covariance[0]>0:
            h=math.sqrt(m.position_covariance[0])
        with self._lk:
            s=self._s; s.gps_stamp=self._now()
            s.lat=m.latitude; s.lon=m.longitude; s.hacc=h; s.gps_st=m.status.status

    def _odom(self,m:Odometry):
        _,_,y=self._qe(m.pose.pose.orientation.x,m.pose.pose.orientation.y,
                        m.pose.pose.orientation.z,m.pose.pose.orientation.w)
        with self._lk:
            s=self._s; s.ox=m.pose.pose.position.x; s.oy=m.pose.pose.position.y
            s.oyaw=y; s.ovx=m.twist.twist.linear.x
            if not s.trail or math.hypot(s.ox-s.trail[-1][0],s.oy-s.trail[-1][1])>0.5:
                s.trail.append((s.ox,s.oy))
                if len(s.trail)>self.TRAIL: s.trail.pop(0)

    def _ego(self,m:PoseStamped):
        _,_,y=self._qe(m.pose.orientation.x,m.pose.orientation.y,
                        m.pose.orientation.z,m.pose.orientation.w)
        with self._lk: self._s.ox=m.pose.position.x; self._s.oy=m.pose.position.y; self._s.oyaw=y

    def _ehdg(self,m:Float64):
        with self._lk: self._s.eskf_stamp=self._now(); self._s.eskf_hdg=float(m.data)
    def _ecov(self,m:Float64):
        with self._lk: self._s.eskf_cov=float(m.data)
    def _pst(self,m:String):
        with self._lk: self._s.park_state=m.data
    def _xy(self,m:Path): return [(p.pose.position.x,p.pose.position.y) for p in m.poses]
    def _lpath(self,m:Path):
        with self._lk: self._s.lp=self._xy(m)
    def _ppath(self,m:Path):
        with self._lk: self._s.pp=self._xy(m)
    def _mpath(self,m:Path):
        with self._lk: self._s.mp=self._xy(m)
    def snap(self):
        with self._lk: return copy(self._s)
    def fresh(self,t): n=self._now(); return t>0 and (n-t)<=self.STALE


def _gui(node:IriNode):
    if not _HAS_IR:
        print('[WARN] pyridescence 없음'); return

    v=guik.LightViewer.instance(resolution=[1600,900],title='MORAI MPC - Ioniq 5 SUV')
    v.enable_vsync()
    v.update_drawable('axis',glk.primitives.coordinate_system(),guik.VertexColor(np.eye(4,dtype=np.float32)))

    while v.spin_once():
        s=node.snap()

        # 차량
        yaw=s.oyaw; T=np.eye(4,dtype=np.float32)
        c,ss=math.cos(yaw),math.sin(yaw)
        T[0,0]=c;T[0,1]=-ss;T[1,0]=ss;T[1,1]=c;T[0,3]=s.ox;T[1,3]=s.oy;T[2,3]=0.5
        v.update_drawable('car',glk.primitives.cone(),guik.FlatColor(0.2,0.6,1.,1.,T))

        # 궤적
        if len(s.trail)>=2:
            pts=np.array([[x,y,0.1] for x,y in s.trail],dtype=np.float32)
            v.update_drawable('trail',glk.ThinLines(pts),guik.FlatColor(0.,1.,0.4,.8))
        if len(s.lp)>=2:
            pts=np.array([[x,y,.2] for x,y in s.lp],dtype=np.float32)
            v.update_drawable('lpath',glk.ThinLines(pts),guik.FlatColor(1.,.8,0.,.9))
        if len(s.pp)>=2:
            pts=np.array([[x,y,.3] for x,y in s.pp],dtype=np.float32)
            v.update_drawable('ppath',glk.ThinLines(pts),guik.FlatColor(1.,.3,.3,1.))
        if len(s.mp)>=2:
            pts=np.array([[x,y,.4] for x,y in s.mp],dtype=np.float32)
            v.update_drawable('mpath',glk.ThinLines(pts),guik.FlatColor(.3,.7,1.,.7))

        # ImGui
        _ui(s, node)


def _ui(s:Snap, node:IriNode):
    # ── 차량 상태 ──
    imgui.set_next_window_pos((10,10),imgui.ONCE)
    imgui.set_next_window_size((300,220),imgui.ONCE)
    imgui.begin('Ioniq 5 상태')
    mc={'FORWARD':(0.2,.9,.2,1.),'REVERSE':(1.,.5,0.,1.),'PARKING':(1.,.3,.3,1.),
        'APPROACH':(1.,.8,0.,1.),'PLAN':(.5,.5,1.,1.),'DONE':(0.2,1.,.2,1.)}.get(s.park_state,(.6,.6,.6,1.))
    imgui.text_colored(f'모드: {s.park_state}',*mc)
    imgui.separator()
    spd=abs(s.ovx); dirstr='전진' if s.ovx>=0 else '후진'
    imgui.text(f'속도:  {s.ovx:+.2f} m/s  ({spd*3.6:.1f} km/h) [{dirstr}]')
    imgui.text(f'위치: ({s.ox:.2f}, {s.oy:.2f}) m')
    imgui.text(f'방향:  {math.degrees(s.oyaw):.1f}°')
    imgui.separator()
    ef=node.fresh(s.eskf_stamp)
    ec=(0.2,.9,.2,1.) if ef else (.9,.3,.3,1.)
    imgui.text_colored(f'ESKF 헤딩: {s.eskf_hdg:.2f}°',*ec)
    if s.eskf_cov>0:
        sg=math.degrees(math.sqrt(s.eskf_cov/15.+1e-10))
        imgui.text(f'불확도: ±{sg:.2f}°')
    imgui.end()

    # ── IMU ──
    imgui.set_next_window_pos((10,240),imgui.ONCE)
    imgui.set_next_window_size((300,220),imgui.ONCE)
    imgui.begin('IMU')
    ic=(0.2,.9,.2,1.) if node.fresh(s.imu_stamp) else (.9,.3,.3,1.)
    imgui.text_colored('● LIVE' if node.fresh(s.imu_stamp) else '○ STALE',*ic)
    imgui.text(f'Roll:  {s.roll:+7.2f}°')
    imgui.text(f'Pitch: {s.pitch:+7.2f}°')
    imgui.text(f'Yaw:   {s.yaw_imu:+7.2f}°')
    imgui.separator()
    imgui.text(f'Gyro  x={s.gx:+.3f}  y={s.gy:+.3f}  z={s.gz:+.3f} rad/s')
    imgui.text(f'Accel x={s.ax:+.3f}  y={s.ay:+.3f}  z={s.az:+.3f} m/s²')
    imgui.end()

    # ── GPS ──
    imgui.set_next_window_pos((320,10),imgui.ONCE)
    imgui.set_next_window_size((280,160),imgui.ONCE)
    imgui.begin('GPS')
    gf=node.fresh(s.gps_stamp)
    gc=(0.2,.9,.2,1.) if gf else (.9,.3,.3,1.)
    imgui.text_colored('● LIVE' if gf else '○ STALE',*gc)
    fs={-1:'NO FIX',0:'FIX',1:'SBAS+FIX',2:'GBAS+FIX'}.get(s.gps_st,'?')
    fc=(0.2,.9,.2,1.) if s.gps_st>=0 else (.9,.3,.3,1.)
    imgui.text_colored(f'Fix: {fs}',*fc)
    ac=(0.2,.9,.2,1.) if s.hacc<0.1 else (1.,.8,0.,1.) if s.hacc<0.5 else (.9,.3,.3,1.)
    imgui.text_colored(f'수평 정확도: {s.hacc:.3f} m',*ac)
    imgui.text(f'위도: {s.lat:.7f}°')
    imgui.text(f'경도: {s.lon:.7f}°')
    imgui.end()

    # ── 주차 진행 ──
    if s.park_state not in('FORWARD','IDLE'):
        imgui.set_next_window_pos((320,180),imgui.ONCE)
        imgui.set_next_window_size((280,120),imgui.ONCE)
        imgui.begin('주차')
        imgui.text(f'상태: {s.park_state}')
        if s.park_state=='DONE':
            imgui.text_colored('✓ 주차 완료!',0.2,1.,0.2,1.)
        imgui.end()


def main(args=None):
    rclpy.init(args=args)
    node=IriNode()
    ex=MultiThreadedExecutor(); ex.add_node(node)
    t=threading.Thread(target=ex.spin,daemon=True); t.start()
    if _HAS_IR:
        try: _gui(node)
        except Exception as e: node.get_logger().error(f'GUI 오류: {e}')
    else:
        node.get_logger().warn('pyridescence 없음 - ROS만 실행'); t.join()
    ex.shutdown(); node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
