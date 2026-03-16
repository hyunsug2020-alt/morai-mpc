#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MORAI Ioniq 5 주차 매니저
==========================
논문 기반:
  1. [하이브리드의 근본 논문]    Hybrid A* 기초
  2. [하이브리드의 구조적 설계]  Guided Hybrid A* (비지빌리티+웨이포인트)
  3. [하이브리드와 mpc의 연결]  계층: Hybrid A* → RTI-NMPC
  4. [2410.12170v1.pdf]        RTI-NMPC 후진 주차

상태 머신:
  FORWARD  → LTV-MPC 전진 (mpc_path_tracker_cpp 이 담당)
  APPROACH → 주차구역 접근, 감속
  PLAN     → Hybrid A* 경로 계획
  REVERSE  → RTI-NMPC 후진 주차
  FINE     → 미세 보정
  DONE     → 완료
"""
from __future__ import annotations
import math, time, sys, os
from enum import Enum, auto
from typing import Optional, List, Tuple
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Pose, Accel, Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'navigation'))
try:
    from hybrid_astar import HybridAStarPlanner, HybridAStarConfig, Obstacle
    _HA = True
except ImportError:
    _HA = False


class S(Enum):
    FORWARD = auto(); APPROACH = auto(); PLAN = auto()
    REVERSE = auto(); FINE = auto(); DONE = auto(); IDLE = auto()

S_NAME = {S.FORWARD:'FORWARD', S.APPROACH:'APPROACH', S.PLAN:'PLAN',
          S.REVERSE:'REVERSE', S.FINE:'FINE_TUNE', S.DONE:'DONE', S.IDLE:'IDLE'}


# ── RTI-NMPC (Ioniq 5, 논문 2410.12170v1.pdf) ────────────────
class RTINMPC:
    NX, NU = 5, 2
    def __init__(self, wb=3.0, N=20, Ts=0.1):
        self.wb = wb; self.N = N; self.Ts = Ts
        self.kmax = math.tan(math.radians(35)) / wb
        self.kmin = -self.kmax
        self.vmax = 0.5; self.vmin = -1.5
        self.Q = np.diag([15.,15.,8.,2.,1.])
        self.R = np.diag([0.5, 1.0])
        self.kappa = 0.0; self.v = 0.0
        self.uw = [np.zeros(2) for _ in range(N)]
        self.init = False

    def reset(self):
        self.kappa=0.; self.v=0.; self.uw=[np.zeros(2) for _ in range(self.N)]; self.init=False

    def _f(self, x, u):
        c,s = math.cos(x[2]), math.sin(x[2])
        return np.array([x[3]*c, x[3]*s, x[3]*x[4], u[0], u[1]])

    def _Fx(self, x, u):
        J = np.zeros((5,5))
        J[0,2]=-x[3]*math.sin(x[2]); J[0,3]=math.cos(x[2])
        J[1,2]= x[3]*math.cos(x[2]); J[1,3]=math.sin(x[2])
        J[2,3]=x[4]; J[2,4]=x[3]
        return J

    def _ie(self, xk, uk):
        xn = xk.copy()
        for _ in range(3):
            r = xn - xk - self.Ts*self._f(xn,uk)
            Jr = np.eye(5) - self.Ts*self._Fx(xn,uk)
            xn -= np.linalg.solve(Jr, r)
        xn[2] = math.atan2(math.sin(xn[2]),math.cos(xn[2]))
        xn[4] = np.clip(xn[4], self.kmin, self.kmax)
        xn[3] = np.clip(xn[3], self.vmin, self.vmax)
        return xn

    def compute(self, x0, xref):
        if not self.init:
            self.kappa=x0[4]; self.v=x0[3]; self.init=True
        N,NX,NU = self.N,self.NX,self.NU
        xt=[x0.copy()]
        for k in range(N): xt.append(self._ie(xt[-1],self.uw[k]))

        Ad_list,Bd_list,dk_list=[],[],[]
        for k in range(N):
            Fx=self._Fx(xt[k],self.uw[k]); Fu=np.zeros((5,2)); Fu[3,0]=Fu[4,1]=1.
            M=np.eye(5)-self.Ts*Fx; Ad=np.linalg.inv(M); Bd=Ad@(self.Ts*Fu)
            dk=self._ie(xt[k],self.uw[k])-Ad@xt[k]-Bd@self.uw[k]
            Ad_list.append(Ad); Bd_list.append(Bd); dk_list.append(dk)

        try:
            import osqp, scipy.sparse as sp
            n=N*NU
            Phi=np.zeros(((N+1)*NX,NX)); Gamma=np.zeros(((N+1)*NX,n)); sig=np.zeros((N+1)*NX)
            Phi[:NX]=np.eye(NX); Pk=np.eye(NX); sk=np.zeros(NX)
            for k in range(N):
                Pk=Ad_list[k]@Pk; sk=Ad_list[k]@sk+dk_list[k]
                Phi[(k+1)*NX:(k+2)*NX]=Pk; sig[(k+1)*NX:(k+2)*NX]=sk
                for j in range(k+1):
                    Gamma[(k+1)*NX:(k+2)*NX,j*NU:(j+1)*NU] = \
                        Bd_list[k] if j==k else Ad_list[k]@Gamma[k*NX:(k+1)*NX,j*NU:(j+1)*NU]
            Qb=np.zeros(((N+1)*NX,(N+1)*NX))
            for k in range(N+1): Qb[k*NX:(k+1)*NX,k*NX:(k+1)*NX]=(3. if k==N else 1.)*self.Q
            Rb=np.zeros((n,n))
            for k in range(N): Rb[k*NU:(k+1)*NU,k*NU:(k+1)*NU]=self.R
            Xr=np.zeros((N+1)*NX)
            for k in range(N+1):
                i=min(k,len(xref)-1); Xr[k*NX:(k+1)*NX]=xref[i]
            GQ=Gamma.T@Qb; H=GQ@Gamma+Rb
            H=0.5*(H+H.T)+1e-6*np.eye(n); f=GQ@(Phi@x0+sig-Xr)
            lb=np.tile([-3.,-2.],N); ub=np.tile([1.,2.],N)
            p=osqp.OSQP()
            p.setup(sp.csc_matrix(H),f,sp.eye(n,format='csc'),lb,ub,
                    warm_starting=True,verbose=False,max_iter=300,eps_abs=1e-4,eps_rel=1e-4)
            r=p.solve()
            if r.info.status in('solved','solved_inaccurate'):
                u=r.x
                for k in range(N-1): self.uw[k]=u[(k+1)*NU:(k+2)*NU]
                self.uw[-1]=u[(N-1)*NU:]
                self.v    =np.clip(self.v    +self.Ts*u[0], self.vmin, self.vmax)
                self.kappa=np.clip(self.kappa+self.Ts*u[1], self.kmin, self.kmax)
        except Exception:
            dx=xref[0][0]-x0[0]; dy=xref[0][1]-x0[1]
            tpsi=math.atan2(dy,dx)
            epsi=math.atan2(math.sin(tpsi-x0[2]),math.cos(tpsi-x0[2]))
            self.v=-0.3; self.kappa=np.clip(epsi*0.5,self.kmin,self.kmax)

        return float(self.v), float(self.v*self.kappa)


# ── 주차 매니저 노드 ─────────────────────────────────────────
class ParkingManagerNode(Node):

    def __init__(self):
        super().__init__('parking_manager_node')

        for n,d in [('trigger_dist',20.),('done_dist',0.3),('done_angle_deg',5.),
                    ('approach_speed',3.),('timeout_sec',180.),('rate_hz',20.)]:
            self.declare_parameter(n, d)
        p=lambda n: self.get_parameter(n).value

        self.trig  = float(p('trigger_dist'))
        self.ddist = float(p('done_dist'))
        self.dang  = math.radians(float(p('done_angle_deg')))
        self.tmo   = float(p('timeout_sec'))

        self.state: S = S.FORWARD
        self.pose: Optional[Pose] = None
        self.yaw = 0.; self.cv = 0.
        self.goal: Optional[Tuple[float,float,float]] = None
        self.path: List[Tuple[float,float,float]] = []
        self.t_park: Optional[float] = None

        self.rti = RTINMPC()
        if _HA:
            self.ha = HybridAStarPlanner(HybridAStarConfig(
                xy_resolution=0.20, theta_resolution=math.radians(5),
                wheelbase=3.0, max_steer=math.radians(35),
                vehicle_length=4.635, vehicle_width=1.890, use_reed_shepp=True))
        else:
            self.ha = None

        sq=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST,depth=10)

        self.create_subscription(PoseStamped,'/Ego_pose',           self._pose_cb, sq)
        self.create_subscription(Odometry,   '/odometry/filtered',  self._odom_cb, sq)
        self.create_subscription(PoseStamped,'/parking_goal',       self._goal_cb, 10)
        self.create_subscription(String,     '/drive_mode_cmd',     self._mode_cb, 10)

        self.accel_pub  = self.create_publisher(Accel,      '/Accel',          10)
        self.path_pub   = self.create_publisher(Path,        '/parking_path',   10)
        self.state_pub  = self.create_publisher(String,      '/parking_state',  10)
        self.mk_pub     = self.create_publisher(MarkerArray, '/parking_markers',10)

        hz = float(p('rate_hz'))
        self.create_wall_timer(1.0/hz, self._loop)
        self.get_logger().info('주차 매니저 시작 (Ioniq5, Hybrid A*+RTI-NMPC)')

    # ── 콜백 ──
    def _pose_cb(self, m: PoseStamped):
        self.pose=m.pose; q=m.pose.orientation
        n=math.sqrt(q.x**2+q.y**2+q.z**2+q.w**2)
        if n>1e-6: self.yaw=math.atan2(2*(q.w/n*q.z/n),1-2*(q.z/n)**2)

    def _odom_cb(self, m: Odometry):
        self.cv=float(m.twist.twist.linear.x)
        if self.pose is None: self.pose=m.pose.pose

    def _goal_cb(self, m: PoseStamped):
        q=m.pose.orientation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y**2+q.z**2))
        self.goal=(m.pose.position.x, m.pose.position.y, yaw)
        self.get_logger().info(f'주차 목표: ({self.goal[0]:.2f},{self.goal[1]:.2f},{math.degrees(self.goal[2]):.1f}°)')
        if self.state==S.FORWARD: self.state=S.APPROACH

    def _mode_cb(self, m: String):
        c=m.data.upper()
        if c=='FORWARD': self.state=S.FORWARD; self.rti.reset()
        elif c in('PARKING','PARK') and self.goal: self.state=S.PLAN
        elif c=='IDLE': self.state=S.IDLE

    # ── 루프 ──
    def _loop(self):
        if self.pose is None: return
        ss=String(); ss.data=S_NAME.get(self.state,'?'); self.state_pub.publish(ss)
        self._markers()
        {S.FORWARD:self._fwd, S.APPROACH:self._approach,
         S.PLAN:self._plan, S.REVERSE:self._reverse,
         S.FINE:self._fine, S.DONE:self._stop,
         S.IDLE:lambda:None}.get(self.state, lambda:None)()

    def _fwd(self):
        if not self.goal: return
        d=math.hypot(self.goal[0]-self.pose.position.x, self.goal[1]-self.pose.position.y)
        if d<self.trig: self.get_logger().info(f'주차구역 {d:.1f}m → APPROACH'); self.state=S.APPROACH

    def _approach(self):
        if not self.goal: self.state=S.FORWARD; return
        d=math.hypot(self.goal[0]-self.pose.position.x, self.goal[1]-self.pose.position.y)
        if d<5.0: self.state=S.PLAN

    def _plan(self):
        if not self.goal or not self.pose: self.state=S.FORWARD; return
        sx,sy,sth=self.pose.position.x,self.pose.position.y,self.yaw
        gx,gy,gth=self.goal
        self.get_logger().info('Hybrid A* 경로 계획 중...')
        t0=time.time()
        raw=None
        if self.ha:
            raw=self.ha.plan(sx,sy,sth,gx,gy,gth,
                             map_origin_x=min(sx,gx)-15, map_origin_y=min(sy,gy)-15)
        dt=(time.time()-t0)*1000
        self.get_logger().info(f'Hybrid A* 완료 {dt:.0f}ms, 경로={len(raw) if raw else 0}점')
        if not raw:
            self.get_logger().warn('Hybrid A* 실패 → 직선 fallback')
            raw=[(sx+i/20*(gx-sx), sy+i/20*(gy-sy), sth+i/20*(gth-sth), -1) for i in range(21)]
        if self.ha:
            sm=HybridAStarPlanner.smooth_path(raw)
            self.path=HybridAStarPlanner.path_to_ros_waypoints(sm, ds=0.3)
        else:
            self.path=[(x,y,th) for x,y,th,*_ in raw]
        self._pub_path(); self.rti.reset()
        self.t_park=time.time(); self.state=S.REVERSE

    def _reverse(self):
        if not self.path or not self.pose: return
        if self.t_park and time.time()-self.t_park>self.tmo:
            self.get_logger().warn('주차 타임아웃'); self.state=S.FORWARD; return
        if self._done_check(factor=2): self.state=S.FINE; return
        x0=np.array([self.pose.position.x,self.pose.position.y,
                     self.yaw,self.cv,self.rti.kappa])
        cl=self._closest(x0)
        xr=[np.array([self.path[min(cl+k,len(self.path)-1)][0],
                       self.path[min(cl+k,len(self.path)-1)][1],
                       self.path[min(cl+k,len(self.path)-1)][2],
                       -0.5, 0.0]) for k in range(self.rti.N+1)]
        vc,wc=self.rti.compute(x0,xr)
        a=Accel(); a.linear.x=max(-3., min(1., (vc-self.cv)*3.)); a.angular.z=wc
        self.accel_pub.publish(a)

    def _fine(self):
        if not self.goal: self.state=S.DONE; return
        gx,gy,gth=self.goal
        d=math.hypot(gx-self.pose.position.x, gy-self.pose.position.y)
        da=abs(math.atan2(math.sin(gth-self.yaw),math.cos(gth-self.yaw)))
        if d<self.ddist and da<self.dang:
            self.get_logger().info(f'주차 완료! d={d*100:.0f}cm Δθ={math.degrees(da):.1f}°')
            self._stop(); self.state=S.DONE
        else:
            a=Accel(); a.linear.x=-0.1 if d>0.1 else 0.
            a.angular.z=math.atan2(math.sin(gth-self.yaw),math.cos(gth-self.yaw))*0.3
            self.accel_pub.publish(a)

    def _stop(self):
        a=Accel(); a.linear.x=-1. if abs(self.cv)>0.05 else 0.; self.accel_pub.publish(a)

    def _done_check(self, factor=1):
        if not self.goal or not self.pose: return False
        gx,gy,gth=self.goal
        d=math.hypot(gx-self.pose.position.x, gy-self.pose.position.y)
        da=abs(math.atan2(math.sin(gth-self.yaw),math.cos(gth-self.yaw)))
        return d<self.ddist*factor and da<self.dang*factor

    def _closest(self, x0):
        if not self.path: return 0
        bi,bd=0,float('inf')
        for i,(wx,wy,_) in enumerate(self.path):
            d=(wx-x0[0])**2+(wy-x0[1])**2
            if d<bd: bd,bi=d,i
        return bi

    def _pub_path(self):
        pm=Path(); pm.header.stamp=self.get_clock().now().to_msg(); pm.header.frame_id='map'
        for x,y,th in self.path:
            ps=PoseStamped(); ps.header=pm.header
            ps.pose.position.x=x; ps.pose.position.y=y
            ps.pose.orientation.z=math.sin(th/2); ps.pose.orientation.w=math.cos(th/2)
            pm.poses.append(ps)
        self.path_pub.publish(pm)

    def _markers(self):
        ma=MarkerArray(); now=self.get_clock().now().to_msg()
        if self.goal:
            m=Marker(); m.header.frame_id='map'; m.header.stamp=now
            m.ns='goal'; m.id=0; m.type=Marker.ARROW; m.action=Marker.ADD
            gx,gy,gth=self.goal; m.pose.position.x=gx; m.pose.position.y=gy
            m.pose.orientation.z=math.sin(gth/2); m.pose.orientation.w=math.cos(gth/2)
            m.scale.x=2.; m.scale.y=0.4; m.scale.z=0.4
            m.color.g=1.; m.color.a=0.9; ma.markers.append(m)
        if len(self.path)>=2:
            m2=Marker(); m2.header.frame_id='map'; m2.header.stamp=now
            m2.ns='path'; m2.id=1; m2.type=Marker.LINE_STRIP; m2.action=Marker.ADD
            m2.scale.x=0.12; m2.color.b=1.; m2.color.a=1.
            for x,y,_ in self.path: p=Point(); p.x=x; p.y=y; p.z=0.1; m2.points.append(p)
            ma.markers.append(m2)
        self.mk_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node=ParkingManagerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
