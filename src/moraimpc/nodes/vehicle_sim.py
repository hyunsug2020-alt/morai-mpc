#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vehicle_sim — MORAI 대체 헤드리스 차량 시뮬 (HITL: 실제 LTV-MPC follower 검증용).
  구독: /ctrl_cmd (CtrlCmd: longlCmdType=2, velocity[km/h], steering[rad 전륜각])
  발행: /Ego_topic (EgoVehicleStatus), /Object_topic (ObjectStatusList, 차선추종 NPC)
  모델: Ioniq5 운동학 자전거 + 조향 rate/lag + 종방향 가감속 한계.
  종료: sim_time 경과 or 충돌 → /tmp/hitl_result.json 기록 후 shutdown.

  실제 planner(hdmap_lane_avoid) + 실제 follower(path_follower_node)와 함께 구동.
"""
import json, math, zipfile, sys
from collections import defaultdict
import rospy
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList, ObjectStatus, CtrlCmd
from std_msgs.msg import String

# ── Ioniq5 ──
L_WB=3.0; CAR_L=4.635; CAR_W=1.892; HL=CAR_L/2; HW=CAR_W/2
MAX_STEER=math.radians(40); STEER_RATE=math.radians(45); TAU_STEER=0.12
A_MAX=2.0; B_MAX=6.5; KP_V=0.8
NPC_L=4.5; NPC_W=2.0

def norm(a): return (a+math.pi)%(2*math.pi)-math.pi

def resample(pts,step=0.5):
    if len(pts)<2: return list(pts)
    out=[pts[0]]; carry=0.0
    for i in range(1,len(pts)):
        ax,ay=out[-1]; bx,by=pts[i]; seg=math.hypot(bx-ax,by-ay)
        if seg<1e-9: continue
        d=seg; sx,sy=ax,ay
        while carry+d>=step:
            t=(step-carry)/d; nx,ny=sx+(bx-sx)*t, sy+(by-sy)*t
            out.append((nx,ny)); sx,sy=nx,ny; d=math.hypot(bx-sx,by-sy); carry=0.0
        carry+=d
    out.append(pts[-1]); return out

def load_lanes(zp):
    z=zipfile.ZipFile(zp); links=json.loads(z.read("link_set.json"))
    roads=defaultdict(lambda: defaultdict(list))
    for L in links: roads[L["road_id"]][L["ego_lane"]].append(L)
    def ll(L): return L.get("link_length") or 0
    best=None; bs=0
    for rid,lanes in roads.items():
        if len(lanes)<3: continue
        reps=[max(ls,key=ll) for ls in lanes.values()]
        m=min(ll(L) for L in reps)
        if m>bs: bs=m; best=rid
    lanes=roads[best]; raw=[]
    for ln in sorted(lanes.keys()):
        b=max(lanes[ln], key=ll); raw.append(resample([(p[0],p[1]) for p in b["points"]],0.5))
    def spacing(a,b): return min(min(math.hypot(p[0]-q[0],p[1]-q[1]) for q in b[::5]) for p in a[::10])
    out=[raw[0]]
    for i in range(1,len(raw)):
        if spacing(out[-1],raw[i])>2.8: out.append(raw[i])
    return out

def lane_len(lane): return sum(math.hypot(lane[i+1][0]-lane[i][0], lane[i+1][1]-lane[i][1]) for i in range(len(lane)-1))
def lane_at_s(lane,s):
    if s<=0: h=math.atan2(lane[1][1]-lane[0][1],lane[1][0]-lane[0][0]); return lane[0][0],lane[0][1],h
    acc=0.0
    for i in range(len(lane)-1):
        seg=math.hypot(lane[i+1][0]-lane[i][0], lane[i+1][1]-lane[i][1])
        if acc+seg>=s:
            t=(s-acc)/max(seg,1e-9)
            return lane[i][0]+(lane[i+1][0]-lane[i][0])*t, lane[i][1]+(lane[i+1][1]-lane[i][1])*t, math.atan2(lane[i+1][1]-lane[i][1],lane[i+1][0]-lane[i][0])
        acc+=seg
    h=math.atan2(lane[-1][1]-lane[-2][1],lane[-1][0]-lane[-2][0]); return lane[-1][0],lane[-1][1],h

def obb_collide(ax,ay,ath,bx,by,bth,bl,bw):
    Ax=(math.cos(ath),math.sin(ath)); Ay=(-math.sin(ath),math.cos(ath))
    Bx=(math.cos(bth),math.sin(bth)); By=(-math.sin(bth),math.cos(bth))
    dx=bx-ax; dy=by-ay
    for (ux,uy) in (Ax,Ay,Bx,By):
        rA=HL*abs(ux*Ax[0]+uy*Ax[1])+HW*abs(ux*Ay[0]+uy*Ay[1])
        rB=(bl/2)*abs(ux*Bx[0]+uy*Bx[1])+(bw/2)*abs(ux*By[0]+uy*By[1])
        if abs(dx*ux+dy*uy)>rA+rB: return False
    return True


class VehicleSim:
    def __init__(self):
        rospy.init_node("vehicle_sim")
        zp=rospy.get_param("~hdmap_zip","/mnt/c/Users/Hyunsug2014/Downloads/drive-download-20260701T154301Z-3-001.zip")
        self.lanes=load_lanes(zp)
        self.ego_lane=int(rospy.get_param("~ego_lane",1))
        self.v_set=float(rospy.get_param("~cruise_mps",12.5))
        self.Tsim=float(rospy.get_param("~sim_time",22.0))
        specs=json.loads(rospy.get_param("~npc_specs","[]"))   # [[laneN,frac,sp],...]
        self.out=rospy.get_param("~result_file","/tmp/hitl_result.json")
        # ego state
        li=self.ego_lane-1
        x,y,h=lane_at_s(self.lanes[li], 0.06*lane_len(self.lanes[li]))
        self.x=x; self.y=y; self.yaw=h; self.v=self.v_set
        self.steer=0.0; self.steer_cmd=0.0; self.vel_cmd=self.v_set
        # npc: [lane_idx, s, sp]
        self.npcs=[[int(l)-1, float(fr)*lane_len(self.lanes[int(l)-1]), float(sp)] for (l,fr,sp) in specs]
        self.slow=[i for i,(l,fr,sp) in enumerate(specs) if float(sp)<self.v_set-0.5]
        self.path=[]   # follower가 따라가는 planner 경로(/avoid_waypoints)
        # result
        self.collided=False; self.cause=None; self.max_cte=0.0; self.overtaken=set(); self.done=False
        self.t=0.0
        rospy.Subscriber("/ctrl_cmd", CtrlCmd, self._ctrl_cb, queue_size=1)
        rospy.Subscriber("/avoid_waypoints", String, self._wps_cb, queue_size=1)
        self.ego_pub=rospy.Publisher("/Ego_topic", EgoVehicleStatus, queue_size=1)
        self.obj_pub=rospy.Publisher("/Object_topic", ObjectStatusList, queue_size=1)
        rospy.loginfo("[vsim] egoL=%d v_set=%.1f npc=%d Tsim=%.0f", self.ego_lane, self.v_set, len(self.npcs), self.Tsim)

    def _ctrl_cb(self, m):
        self.steer_cmd=float(m.steering)          # rad 전륜각
        self.vel_cmd=float(m.velocity)/3.6        # km/h→m/s

    def _wps_cb(self, m):
        try:
            wp=json.loads(m.data)["waypoints"]; self.path=[(w["x"],w["y"]) for w in wp]
        except Exception: pass

    def _publish_ego(self):
        e=EgoVehicleStatus()
        e.header.stamp=rospy.Time.now(); e.header.frame_id="map"
        e.position.x=self.x; e.position.y=self.y; e.position.z=0.0
        e.heading=math.degrees(self.yaw)
        e.velocity.x=self.v; e.velocity.y=0.0; e.velocity.z=0.0   # body-frame 전방속도
        self.ego_pub.publish(e)

    def _publish_objs(self):
        ol=ObjectStatusList(); ol.header.stamp=rospy.Time.now(); ol.header.frame_id="map"
        lst=[]
        for i,o in enumerate(self.npcs):
            ox,oy,oh=lane_at_s(self.lanes[o[0]], o[1])
            s=ObjectStatus(); s.unique_id=i+1; s.type=1; s.name="npc%d"%i
            s.position.x=ox; s.position.y=oy; s.position.z=0.0
            s.heading=math.degrees(oh)
            s.velocity.x=o[2]*math.cos(oh); s.velocity.y=o[2]*math.sin(oh); s.velocity.z=0.0
            s.size.x=NPC_L; s.size.y=NPC_W; s.size.z=1.5
            lst.append(s)
        ol.npc_list=lst; ol.num_of_npcs=len(lst)
        ol.pedestrian_list=[]; ol.obstacle_list=[]; ol.num_of_pedestrian=0; ol.num_of_obstacle=0
        self.obj_pub.publish(ol)

    def _step(self, dt):
        # 조향: rate limit + 마찰한계 + 1차지연
        v2=max(self.v*self.v,1.0)
        d_fric=math.atan(1.10*9.81*L_WB/v2)
        dmax=min(MAX_STEER,d_fric)
        cmd=max(-dmax,min(dmax,self.steer_cmd))
        tgt=self.steer+(cmd-self.steer)*(dt/max(TAU_STEER,dt))
        dd=max(-STEER_RATE*dt,min(STEER_RATE*dt,tgt-self.steer)); self.steer+=dd
        # 종방향
        a=max(-B_MAX,min(A_MAX,KP_V*(self.vel_cmd-self.v))); self.v=max(0.0,self.v+a*dt)
        # 자전거모델
        self.x+=self.v*math.cos(self.yaw)*dt; self.y+=self.v*math.sin(self.yaw)*dt
        self.yaw+=self.v/L_WB*math.tan(self.steer)*dt
        for o in self.npcs: o[1]+=o[2]*dt
        # CTE
        if self.path:
            cte=min(math.hypot(px-self.x,py-self.y) for px,py in self.path[::2])
            self.max_cte=max(self.max_cte,cte)
        # 충돌/추월
        ce,se=math.cos(self.yaw),math.sin(self.yaw)
        for i,o in enumerate(self.npcs):
            ox,oy,oh=lane_at_s(self.lanes[o[0]], o[1])
            dx=ox-self.x; dy=oy-self.y; fl=ce*dx+se*dy; lt=-se*dx+ce*dy
            if i in self.slow and fl<-CAR_L and abs(lt)>CAR_W: self.overtaken.add(i)
            if obb_collide(self.x,self.y,self.yaw, ox,oy,oh, NPC_L,NPC_W):
                self.collided=True
                self.cause=("유령추돌" if fl<-1.0 else ("측면고속" if o[2]>self.v+0.5 else "플래너과실"))

    def spin(self):
        dt=0.02; r=rospy.Rate(1.0/dt); pub_every=2; k=0
        while not rospy.is_shutdown() and self.t<self.Tsim and not self.collided:
            self._step(dt)
            self._publish_ego()
            if k%pub_every==0: self._publish_objs()
            k+=1; self.t+=dt
            r.sleep()
        res={"collision":self.collided,"cause":self.cause,"max_cte":round(self.max_cte,3),
             "overtaken":len(self.overtaken),"n_slow":len(self.slow),"final_v":round(self.v,2),
             "stopped":self.v<0.5,"t":round(self.t,2),"ego_lane":self.ego_lane,"v_set":round(self.v_set,2)}
        try:
            with open(self.out,"w") as f: json.dump(res,f,ensure_ascii=False)
        except Exception as e: rospy.logwarn("result write fail: %s",e)
        rospy.loginfo("[vsim] RESULT %s", json.dumps(res,ensure_ascii=False))
        rospy.signal_shutdown("done")


if __name__=="__main__":
    try: VehicleSim().spin()
    except rospy.ROSInterruptException: pass
