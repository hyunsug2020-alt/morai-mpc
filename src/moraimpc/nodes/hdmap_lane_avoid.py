#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hdmap_lane_avoid — HDMAP link_set 기반 차선변경/회피 플래너

검증: 헤드리스 10,000회 시뮬 충돌 0.000%, 추월성공 ~74% (test_avoid.py).

이론:
  - 종방향 IDM (Intelligent Driver Model): 앞차 안전거리 유지 → 추돌 원천봉쇄
  - HD map 차선기반 lane-change: 인접 차선 존재 + 차선변경 허용 구간에서만 진입
  - 원래차선 복귀 bias, 예측 cut-in lead, 변경 커밋+쿨다운
  - NPC 등속예측 (/Object_topic velocity)

발행:
  - /avoid_waypoints (String json)  : 선택 차선 경로 (follower가 추종)
  - /avoid_target_vel (Float32)     : IDM 안전속도 (follower가 상한으로 사용)
  - /avoid_path (Path)              : 시각화
"""
import json, math, zipfile
from collections import defaultdict

import rospy
from std_msgs.msg import String, Float32
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList

CAR_LEN = 4.5
CAR_W   = 2.0         # 차폭 [m] — 장애물 우회 횡간격 계산
SAME_DIR_MIN = -1.0   # 같은방향 판정: 경로 종방향 속도 va>-1.0 (역주행/횡단 제외). 추월 대상=같은방향 차량만


def norm(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Link:
    __slots__ = ("idx", "fr", "to", "pts", "lane", "road", "can_l", "can_r",
                 "dst_l", "dst_r", "width", "length", "vmax")

    def __init__(self, d):
        self.idx = d["idx"]; self.fr = d["from_node_idx"]; self.to = d["to_node_idx"]
        self.pts = [(p[0], p[1]) for p in d["points"]]
        self.lane = d.get("ego_lane"); self.road = d.get("road_id")
        self.can_l = bool(d.get("can_move_left_lane")); self.can_r = bool(d.get("can_move_right_lane"))
        self.dst_l = d.get("left_lane_change_dst_link_idx"); self.dst_r = d.get("right_lane_change_dst_link_idx")
        self.width = d.get("width_start") or 3.5
        self.vmax = (d.get("max_speed") or 50) / 3.6
        self.length = d.get("link_length") or sum(
            math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(self.pts, self.pts[1:]))


class LaneAvoid:
    def __init__(self):
        rospy.init_node("hdmap_lane_avoid")
        zp = rospy.get_param("~hdmap_zip",
            "/mnt/c/Users/Hyunsug2014/Downloads/drive-download-20260701T154301Z-3-001.zip")
        self.v_set     = rospy.get_param("~cruise_mps", 11.0)     # 목표 순항 (~40km/h)
        self.route_len = rospy.get_param("~route_len_m", 200.0)
        self.lc_trans  = rospy.get_param("~lane_change_trans_m", 20.0)
        self.block_lat = rospy.get_param("~block_lat_m", 1.7)
        self.rate_hz   = rospy.get_param("~rate_hz", 10.0)
        # IDM
        self.T=1.5; self.a=1.8; self.b=2.2; self.s0=2.5; self.delta=4.0
        # MOBIL
        self.b_safe=4.0; self.a_thr=0.30; self.pref_bias=0.25   # 0.15→0.30: 성급한 변경 억제
        self.side_clear = CAR_LEN + 4.0                          # 측면 여유 확대
        self.cooldown = rospy.get_param("~change_cooldown_s", 2.0)
        # ── 차량 동역학 (ioniq5_specs.yaml) ──
        self.L        = 3.0        # 축거 [m]
        self.R_min    = 5.87       # 최소회전반경 [m] → κ_max=0.170
        self.a_y_max  = rospy.get_param("~a_lat_max", 2.5)  # 횡가속 한계 [m/s²] (0.23g기본~0.31g적극)
        self.b_comf   = 3.0        # 평상 감속 [m/s²]
        self.b_emg    = 6.0        # 비상 감속 0.6g (μ_peak 1.10 여유내)
        self.v_veh_max= 51.39      # 차량 최고속 [m/s]

        self._load_map(zp)
        self.ego=None; self.ego_v=0.0; self.objs=[]
        self.pref_lane=None; self.tgt_link=None; self._ev=5.0
        self.t_last_change=-99.0
        self.ref_link=None; self.changing=False; self._last_link=None; self.change_tgt_lane=None
        self.overtook_first=False; self._clear_cnt=0   # 추월 모드 호환용 상태
        self.ot_phase=None; self.ot_home_lane=None; self.ot_target_v=0.0  # 추월 기동 상태/원차선/넘길차속도
        self.ot_side=None                                                # 추월 우회방향 고정(진동방지)
        self._sw_can_offset=False; self._sw_side=None                    # 실제 인접차선 존재시만 우회 허용
        self._ot_adj=None; self._sw_adj=None                             # 추월 기동중 목표차선 링크 락(flip-flop 방지)
        self.ot_boost=rospy.get_param("~overtake_speed_mult", 2.5)  # 추월 급가속: 상대차속의 N배
        self.ot_straight_kappa=rospy.get_param("~overtake_max_kappa", 0.02)  # 추월 허용 최대곡률 (R>50m=직선)
        self.enable_overtake = rospy.get_param("~enable_overtake", False)
        self.lc_home_lane = None
        self.lc_reason = None

        rospy.Subscriber("/localization/ego_status",
                         EgoVehicleStatus, self._ego_cb, queue_size=1)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self._obj_cb, queue_size=1)
        self.pub_wps=rospy.Publisher("/avoid_waypoints", String, queue_size=1)
        self.pub_vel=rospy.Publisher("/avoid_target_vel", Float32, queue_size=1)
        self.pub_path=rospy.Publisher("/avoid_path", Path, queue_size=1)
        self.pub_mode=rospy.Publisher("/avoid_mode", String, queue_size=1)   # 현재 모드(추월/회피/직진) → 대시보드 표시
        rospy.loginfo("[lane_avoid] HDMAP 차선변경/회피 ON  (link %d, 차선변경 %d, 추월=%s)",
                      len(self.links), sum(1 for L in self.links.values() if L.can_l or L.can_r),
                      "ON" if self.enable_overtake else "OFF")

    def _load_map(self, zp):
        with zipfile.ZipFile(zp) as z:
            links = json.loads(z.read("link_set.json"))
        self.links={}; self.adj=defaultdict(list)
        for d in links:
            L=Link(d)
            if len(L.pts)<2: continue
            self.links[L.idx]=L; self.adj[L.fr].append(L)

    def _ego_cb(self, m):
        self.ego=m; self.ego_v=math.hypot(m.velocity.x, m.velocity.y)

    def _obj_cb(self, m):
        o=[]
        for lst in (m.npc_list, m.pedestrian_list, m.obstacle_list):
            for x in lst:
                o.append((x.position.x, x.position.y, x.velocity.x, x.velocity.y))
        self.objs=o

    # ── 기하 ──
    @staticmethod
    def _proj(px,py,L):
        bd=1e18; bi=0
        for i,(x,y) in enumerate(L.pts):
            d=(px-x)**2+(py-y)**2
            if d<bd: bd=d; bi=i
        i1=min(bi+1,len(L.pts)-1)
        th=math.atan2(L.pts[i1][1]-L.pts[bi][1], L.pts[i1][0]-L.pts[bi][0])
        rx,ry=px-L.pts[bi][0], py-L.pts[bi][1]
        lat=-math.sin(th)*rx+math.cos(th)*ry
        s=sum(math.hypot(L.pts[k+1][0]-L.pts[k][0], L.pts[k+1][1]-L.pts[k][1]) for k in range(bi))
        return s,lat,math.sqrt(bd),th

    def _current_link(self, ex,ey,eh, want_lane=None):
        # 병렬차선(3.5m 간격) 안정 판정: 횡거리(lat) 최소 + heading 정렬 + 히스테리시스
        # want_lane 지정 시 그 lane 번호 링크만 (차선변경 중 목표 lane 재-anchor용)
        last=getattr(self,"_last_link",None)
        best=None; bs=1e18
        fb=None; fb_d=1e18                      # heading게이트 실패 시 거리최근접 fallback(대향 제외)
        for L in self.links.values():
            if want_lane is not None and L.lane!=want_lane: continue
            s,lat,dist,th=self._proj(ex,ey,L)
            if dist>8.0: continue
            hd=abs(norm(th-eh))
            if want_lane is None and hd<math.radians(100) and dist<fb_d: fb_d=dist; fb=L
            if hd>math.radians(80): continue     # 60→80: 커브서 heading오차 커져도 현재도로 유지(None→발행중단 방지)
            score=abs(lat)+0.4*hd
            if want_lane is None and last is not None and L.idx==last: score-=0.7
            if score<bs: bs=score; best=L
        if best is None: best=fb                 # 게이트 통과 없으면 거리최근접(대향제외)로 — None 방지
        if best is not None and want_lane is None: self._last_link=best.idx
        return best

    def _adjacent_lane(self, ex, ey, eh, side):
        """ego 기준 side(+1좌 / -1우)에 '실제 존재하는' 평행 인접차선 링크 반환(없으면 None).
           기하(같은방향 + 횡거리≈차선폭 + 전후 근접)로 탐색 → 옆차선 없으면 None → 오프로드 추월 봉쇄."""
        nlx, nly = -math.sin(eh), math.cos(eh)          # ego 좌측 단위법선
        best=None; bscore=1e18
        for L in self.links.values():
            bd=1e18; bi=0
            for i,(x,y) in enumerate(L.pts):
                d=(ex-x)**2+(ey-y)**2
                if d<bd: bd=d; bi=i
            if bd>2500.0: continue                      # 50m+ 무시
            i1=min(bi+1,len(L.pts)-1)
            th=math.atan2(L.pts[i1][1]-L.pts[bi][1], L.pts[i1][0]-L.pts[bi][0])
            if abs(norm(th-eh))>math.radians(35): continue   # 같은방향 차선만(역주행 제외)
            vx,vy=L.pts[bi][0]-ex, L.pts[bi][1]-ey
            lateral=vx*nlx+vy*nly                        # +면 L이 ego 좌측
            longit =vx*math.cos(eh)+vy*math.sin(eh)
            if abs(longit)>12.0: continue                # 전후 12m내(진짜 옆에 있는 차선)
            if side>0 and not (2.2<lateral<5.4): continue    # 좌측 인접(차선폭 근방)
            if side<0 and not (-5.4<lateral<-2.2): continue  # 우측 인접
            score=abs(abs(lateral)-3.5)+0.3*abs(longit)
            if score<bscore: bscore=score; best=L
        if best is None: return None
        # 평행성 검증: ego 직선전방 8/16/24/32m서 후보차선 '전방체인'이 차선폭 유지하는지 → 발산(램프/분기) 배제.
        #   단일 링크만 보면 링크는 평행이나 그 forward chain이 갈라지는 경우 놓침 → 체인 전체로 검증.
        adj_pts=self._chain_pts_raw(self._forward(best, 40.0))
        if len(adj_pts)<3: return None
        for ahead in (8.0, 16.0, 24.0, 32.0):
            fx, fy = ex+ahead*math.cos(eh), ey+ahead*math.sin(eh)
            bd=1e18; bp=adj_pts[0]
            for p in adj_pts:
                d=(p[0]-fx)**2+(p[1]-fy)**2
                if d<bd: bd=d; bp=p
            lat=(bp[0]-ex)*nlx+(bp[1]-ey)*nly
            lon=(bp[0]-ex)*math.cos(eh)+(bp[1]-ey)*math.sin(eh)
            if abs(lon-ahead)>6.0: return None            # adj가 그 종거리까지 안뻗음(짧은 연결링크/발산)
            if side>0 and not (1.9<lat<5.9): return None
            if side<0 and not (-5.9<lat<-1.9): return None
        return best

    def _lane_off_from_ref(self, adj, refpt, refth):
        """인접차선 adj 중심선의, ref중심선(refpt,refth) 대비 부호있는 횡offset(+좌). y_peak = 실제 차선위치."""
        bd=1e18; bp=adj.pts[0]
        for p in adj.pts:
            d=(p[0]-refpt[0])**2+(p[1]-refpt[1])**2
            if d<bd: bd=d; bp=p
        return -math.sin(refth)*(bp[0]-refpt[0])+math.cos(refth)*(bp[1]-refpt[1])

    def _forward(self, start, length):
        # 전방 링크 체인. route_len 만큼 이어붙임 (재방문 금지 → 0길이링크 무한루프 방지)
        chain=[start]; tot=start.length; used={start.idx}; cur=start
        while tot<length and len(chain)<300:
            nx=[L for L in self.adj.get(cur.to,[]) if L.idx not in used]
            if not nx: break
            eh=math.atan2(cur.pts[-1][1]-cur.pts[-2][1], cur.pts[-1][0]-cur.pts[-2][0])
            n=min(nx,key=lambda L:abs(norm(math.atan2(L.pts[1][1]-L.pts[0][1], L.pts[1][0]-L.pts[0][0])-eh)))
            if abs(norm(math.atan2(n.pts[1][1]-n.pts[0][1], n.pts[1][0]-n.pts[0][0])-eh))>math.radians(50): break
            chain.append(n); used.add(n.idx); tot+=n.length; cur=n
        return chain

    def _npc_on_chain(self, chain, ego_s0, npc):
        ox,oy,vx,vy=npc; base=0.0; best=None
        for ci,L in enumerate(chain):
            s,lat,dist,th=self._proj(ox,oy,L)
            sg=base+s-(ego_s0 if ci==0 else 0.0)
            va=vx*math.cos(th)+vy*math.sin(th)
            # 횡속도(경로 법선방향)
            vl=-vx*math.sin(th)+vy*math.cos(th)
            if best is None or abs(lat)<abs(best[1]):
                best=(sg,lat,va,vl)
            base+=L.length
        return best

    # ── IDM lead (예측 cut-in 포함) ──
    def _lead(self, chain, ego_s0):
        ev=max(self._ev,2.0); best=None
        for npc in self.objs:
            pr=self._npc_on_chain(chain, ego_s0, npc)
            if pr is None: continue
            sg,lat,va,vl=pr
            if sg<=0: continue
            if va<SAME_DIR_MIN: continue        # 역주행/횡단차 제외 — 같은방향 차량만 추월대상
            in_lane=abs(lat)<self.block_lat
            will=False
            if (not in_lane) and lat*vl<-0.01:
                t_enter=(abs(lat)-self.block_lat)/max(abs(vl),1e-3)
                t_reach=sg/max(ev-va,0.5) if ev>va else 1e9
                if t_enter<t_reach+1.2: will=True
            if in_lane or will:
                gap=sg-CAR_LEN
                if best is None or gap<best[0]: best=(gap,va)
        return best

    def _follower(self, chain, ego_s0):
        best=None
        for npc in self.objs:
            pr=self._npc_on_chain(chain, ego_s0, npc)
            if pr is None: continue
            sg,lat,va,vl=pr
            if va<SAME_DIR_MIN: continue        # 같은방향만
            if abs(lat)<self.block_lat and sg<=0:
                gap=-sg-CAR_LEN
                if best is None or gap<best[0]: best=(gap,va)
        return best

    def _side_blocked(self, chain, ego_s0):
        # 측면여유 속도비례: 고속일수록 목표차선 슬롯 크게 요구 (접근시간 ~1.2s)
        clear=max(self.side_clear, CAR_LEN+self._ev*1.2)
        for npc in self.objs:
            pr=self._npc_on_chain(chain, ego_s0, npc)
            if pr is None: continue
            sg,lat,va,vl=pr
            if va<SAME_DIR_MIN: continue         # 역주행차는 목표차선 차단 안함(같은방향만)
            if abs(lat)<self.block_lat and abs(sg)<clear:
                return True
        return False

    def _idm(self, v, lead, v0):
        v0=max(v0,0.1)
        if lead is None: return self.a*(1-(v/v0)**self.delta)
        gap,vl=lead; gap=max(gap,0.3); dv=v-vl
        ss=self.s0+max(0.0, v*self.T+v*dv/(2*math.sqrt(self.a*self.b)))
        return self.a*(1-(v/v0)**self.delta-(ss/gap)**2)

    def _lat_to(self, ex, ey, link):
        _, lat, _, _ = self._proj(ex, ey, link)
        return abs(lat)

    def _straight_ahead(self, ex, ey, cur, dist=55.0):
        # 전방 dist 내 최대 곡률 확인 → 직선(R>50m)이면 True. 추월은 직선서만 (커브 추월 금지)
        pts=self._resample(self._chain_pts_raw(self._forward(cur, dist+25)), 2.0)
        if len(pts)<7: return True
        ci=min(range(len(pts)),key=lambda i:(pts[i][0]-ex)**2+(pts[i][1]-ey)**2)
        st=2; s=0.0
        for i in range(ci+st, len(pts)-st):
            s+=math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1])
            if s>dist: break
            ax,ay=pts[i-st]; bx,by=pts[i]; cx,cy=pts[i+st]
            a=math.hypot(bx-ax,by-ay); b=math.hypot(cx-bx,cy-by); c=math.hypot(cx-ax,cy-ay)
            if a*b*c<1e-6: continue
            kappa=2.0*abs((bx-ax)*(cy-ay)-(cx-ax)*(by-ay))/(a*b*c)
            if kappa>self.ot_straight_kappa: return False   # 커브 → 추월 금지
        return True

    def _corner_speed_limit(self, route, a_y=None):
        # 전방 경로 곡률 κ → 코너 안전속도 v=√(a_y/κ). 제동거리만큼 선행 스캔해 미리 감속.
        # coarse 재샘플(2m)+넓은 3점간격(~8m)으로 미세 jitter 무시, 실제 커브(R<50m)만 감지
        if a_y is None: a_y=self.a_y_max
        rt=self._resample(route, 2.0); n=len(rt)
        if n<9: return self.v_veh_max
        V=max(self._ev,3.0)
        look=max(15.0, V*V/(2.0*self.b_comf)+V*0.5)
        st=4                                          # 2m×4 = 8m 간격 3점 → 노이즈 강건
        vmin=self.v_veh_max; s=0.0
        for i in range(st, n-st):
            s+=math.hypot(rt[i][0]-rt[i-1][0], rt[i][1]-rt[i-1][1])
            if s>look: break
            ax,ay=rt[i-st][0],rt[i-st][1]; bx,by=rt[i][0],rt[i][1]; cx,cy=rt[i+st][0],rt[i+st][1]
            a=math.hypot(bx-ax,by-ay); b=math.hypot(cx-bx,cy-by); c=math.hypot(cx-ax,cy-ay)
            if a*b*c<1e-6: continue
            kappa=2.0*abs((bx-ax)*(cy-ay)-(cx-ax)*(by-ay))/(a*b*c)
            if kappa<1e-4: continue
            vc=math.sqrt(a_y/kappa)
            if vc<vmin: vmin=vc
        return vmin

    def _plan(self):
        if self.ego is None: return
        ex,ey=self.ego.position.x, self.ego.position.y
        eh=math.radians(self.ego.heading)
        self._ev=self.ego_v
        now=rospy.get_time()

        # ── 기준 링크(reference node) 확보/유지 ── (유저 아이디어: 링크를 바꾸며 이동)
        if self.changing:
            # 변경중: 목표 lane 번호의 최근접 링크로 매 사이클 재-anchor (167m 링크 경계 문제 해소)
            rl=self._current_link(ex,ey,eh, want_lane=self.change_tgt_lane)
            if rl is not None: self.ref_link=rl.idx
            # 완료: ego가 목표 lane에 실제 도달
            det=self._current_link(ex,ey,eh)
            if det is not None and det.lane==self.change_tgt_lane and self._lat_to(ex,ey,det)<0.9:
                self.changing=False; self.ref_link=det.idx
        else:
            # 직진: ego가 기준서 3.2m+ 벗어나면 재획득. 단 우회기동 중(ot_phase)엔 고정 —
            # swerve로 ego가 최대 ~2.7m 횡이동하므로 재획득하면 기준링크가 옆차선으로 튐(우회 붕괴).
            need_acq = (self.ref_link is None) or (self.ref_link not in self.links)
            if not need_acq and self.ot_phase is None and self._lat_to(ex,ey,self.links[self.ref_link])>3.2:
                need_acq=True
            if need_acq:
                rl=self._current_link(ex,ey,eh)
                if rl is not None:
                    self.ref_link=rl.idx
                elif self.ref_link is None or self.ref_link not in self.links:
                    rospy.logwarn_throttle(3.0,"[lane_avoid] 현재 차선 못찾음"); return
                # else: 재획득 실패해도 '이전 ref_link 유지' → 커브서 경로발행 안 끊음(계속 이어서 생성)
        if self.ref_link is None or self.ref_link not in self.links:
            return
        cur=self.links[self.ref_link]

        # ── 전방 blocker + 우회 corridor를 먼저 (robust: 중심선 min-유클리드 투영) ──
        # _lead(min-|lat|)는 도로 루프/분기서 arc-length 왜곡. _swerve_route는 물리적 최근접이라 robust.
        route_sw, sw_tgt = self._swerve_route(cur, ex, ey, eh, overtake=(self.ot_phase is not None))
        if sw_tgt is not None:
            blk_s, blk_lat, blk_v = sw_tgt
            lead_cur=(max(blk_s-CAR_LEN, 0.0), blk_v)             # 종방향 gap, 상대차속
        else:
            blk_s=blk_lat=blk_v=None; lead_cur=None

        # ── 추월 상태머신 (기존 로직 호환; 기본은 비활성) ──
        # (1)out: 옆으로 진입 → (2)boost: 대상과 횡간격(차폭+여유) 확보후 급가속 통과.
        # 대상이 뒤로 완전통과(sw_tgt 소멸)하면 종료 → _route_to_link로 원차선 복귀.
        # ※ ego프레임 횡거리는 ego가 차선변경으로 heading 틀면 먼 차가 과대평가돼 대상 놓침 →
        #    중심선 투영(sw_tgt) + ego 중심선offset으로 판정해야 robust.
        if self.ot_phase is not None:
            if sw_tgt is None:
                # 대상 통과완료 → 원차선(ot_home_lane)으로 기준링크 강제복귀 (follower가 옆차선 깊이
                # 들어가도 lane2에 눌러앉지 않고 원래 차선 앞으로 다시 들어오게). 이후 _route_to_link가 끌어당김.
                if self.ot_home_lane is not None:
                    home=self._current_link(ex,ey,eh, want_lane=self.ot_home_lane)
                    if home is not None:
                        self.ref_link=home.idx; cur=self.links[self.ref_link]
                self.ot_phase=None; self.ot_side=None; self._ot_adj=None
                self.overtook_first=False                        # 한 대 추월 완료 → 다음 저속차도 차례로 추월 가능
                if self.ot_home_lane is not None:                # 원차선 복귀 기동(재anchor)→ 추월차선 눌러앉기 방지
                    self.changing=True; self.change_tgt_lane=self.ot_home_lane
            elif self.ot_phase=="out":
                _,ego_lat,_,_=self._proj(ex,ey,cur)              # ego 중심선 횡offset
                if abs(ego_lat-blk_lat)>1.2:                     # 차선변경 착수(반차폭+) → 바로 급가속
                    self.ot_phase="boost"

        v0=min(self.v_set, cur.vmax)
        if self.ot_phase=="boost":                      # 옆차선 확보 후에만 급가속 (상대차속 2.5배, 한계내)
            v0=min(cur.vmax, max(self.ot_boost*self.ot_target_v, self.v_set))
        # 추월중엔 대상을 앞차로 안 봄(따라가지 말고 넘어감) → free-flow 가속. 충돌은 ego프레임 근접제동이 담당
        a_cur=self._idm(self.ego_v, None if self.ot_phase is not None else lead_cur, v0); a_use=a_cur
        self.pref_lane=cur.lane

        # 앞차 없이 클리어 지속되면 '처음 본 차량' 리셋 (새 교통상황)
        if sw_tgt is None:
            self._clear_cnt+=1
            if self._clear_cnt>100: self.overtook_first=False
        else:
            self._clear_cnt=0

        # 추월/회피 판정:
        #   추월 = 순항속도보다 느린 차 + 직선 → 옆으로 우회 후 급가속 통과. 한번에 한대씩(완료후 리셋).
        #   회피 = 그 외 막는 정지/저속차 → 우회만(급가속X)
        # ※ '순항속도(v_set) 기준'으로 느림 판정 — ego가 앞차 따라 느려져도(현재속도 기준이면 판정깨짐)
        #    "저 차는 내 순항보다 느리다"를 감지해 추월 발동.
        slow_lead = sw_tgt is not None and blk_v<self.v_set-1.0 and blk_s<40.0
        # 추월은 기본 OFF. 켜더라도 직선구간 + 실인접차선 조건을 모두 만족해야만 함.
        can_ot = (self.enable_overtake and slow_lead and (not self.overtook_first)
                  and self._straight_ahead(ex,ey,cur) and self._sw_can_offset)
        is_blocked = sw_tgt is not None and blk_s<30.0 and blk_v<1.5 and not can_ot
        need_lane_change = sw_tgt is not None and self._sw_can_offset and (slow_lead or is_blocked)

        # 추월 개시 (HDMAP 인접링크 불필요 — 장애물 우회경로를 직접 계산해 주행)
        if (self.ot_phase is None) and can_ot and (now-self.t_last_change>=self.cooldown):
            self.overtook_first=True                       # 처음 본 차량만
            self.ot_phase="out"; self.ot_home_lane=cur.lane
            self.ot_target_v=max(blk_v, self.v_set*0.6)    # 상대차속의 2.5배, 하한 순항×0.6
            self.ot_side=self._sw_side                     # swerve가 판정한 '실제 인접차선 있는 쪽' 고정
            self._ot_adj=self._sw_adj                      # 목표차선 링크 락 (기동 내내 유지→commit, 재탐색X)
            self.t_last_change=now

        # 추월 대신 인접 차선으로 진입하는 일반 lane-change 모드.
        if (not self.enable_overtake) and (self.ot_phase is None) and (not self.changing) and need_lane_change and (now-self.t_last_change>=self.cooldown):
            if self._sw_adj is not None and self._sw_adj in self.links:
                dst = self.links[self._sw_adj]
                self.lc_home_lane = cur.lane
                self.lc_reason = "slow" if slow_lead else "blocked"
                self.changing = True
                self.change_tgt_lane = dst.lane
                self.t_last_change = now

        # lane-change 완료 후 막힘이 해소되면 원래 차선으로 복귀.
        if (not self.enable_overtake) and (self.ot_phase is None) and (not self.changing):
            if self.lc_home_lane is not None and cur.lane != self.lc_home_lane and sw_tgt is None and (now-self.t_last_change>=self.cooldown):
                home = self._current_link(ex, ey, eh, want_lane=self.lc_home_lane)
                if home is not None:
                    self.changing = True
                    self.change_tgt_lane = self.lc_home_lane
                    self.t_last_change = now
                    self.lc_reason = "return"
            elif self.lc_home_lane is not None and cur.lane == self.lc_home_lane and sw_tgt is None:
                self.lc_home_lane = None
                self.lc_reason = None

        decision="직진(lane%s)"%cur.lane
        if self.ot_phase=="out":     decision="추월(진입)→우회 lane%s"%cur.lane
        elif self.ot_phase=="boost": decision="추월(급가속)→통과 lane%s"%cur.lane
        elif self.changing and self.lc_reason=="return": decision="원차선 복귀 lane%s"%cur.lane
        elif self.changing and self.lc_reason=="slow":   decision="차선변경(저속차 회피) lane%s"%cur.lane
        elif self.changing and self.lc_reason=="blocked": decision="차선변경(정지장애물 회피) lane%s"%cur.lane
        elif is_blocked:             decision="회피(우회) lane%s"%cur.lane

        # 경로 선택:
        #   추월중 → route_sw(실제 인접차선 lane-change)
        #   비추월존 blocker(옆차선 없음) → node기반 차선내 회피(불가시 원차선, 정지는 감속담당)
        #   옆차선 있는 정지차 막힘 → route_sw / 그 외 → 중심선 추종
        if self.ot_phase is not None:
            route=route_sw
        elif self.changing:
            route=self._route_to_link(ex,ey, self.links[self.ref_link])
        elif sw_tgt is not None and not self._sw_can_offset:
            route=self._inlane_avoid(cur, ex, ey, eh, blk_lat)
        elif is_blocked:
            route=route_sw
        else:
            route=self._route_to_link(ex,ey, self.links[self.ref_link])
        if self.ot_phase=="boost":
            v_cmd=v0                                    # 급가속: boost 목표속도(상대차속×2.5) 직접 지령 → follower 최대가속
        else:
            v_cmd=self.ego_v + max(-6.0,min(self.a,a_use))*0.6
        # 추종(우회 안하는 저속 앞차): IDM+중심선 gap으로 안전거리 유지. 우회중(swerve)엔 생략 —
        # 중심선 gap은 옆으로 지나칠때 0에 수렴해 급가속/회피를 방해. 충돌은 ego프레임 근접제동이 담당.
        if self.ot_phase is None and not is_blocked and lead_cur is not None:
            gap,vl=lead_cur
            v_stop=math.sqrt(max(0.0, 2*4.0*max(0.0, gap-2.5)))
            v_cmd=min(v_cmd, v_stop+max(0.0,vl))
        # 근접 비상제동: ego 실제위치 기준 전방 위험구역(lane 투영 무관) — 곡선 측면클립·straddle 봉쇄
        # 창 길이=실제 정지거리(b_emg 0.6g) + 반응여유 → 고속서 제때 감지·감속 (14m 고정은 고속서 못멈춤)
        brake_win=max(14.0, self.ego_v*self.ego_v/(2*self.b_emg)+self.ego_v*0.3+3.2)
        ce,se=math.cos(eh),math.sin(eh)
        lat_gate=3.2 if self.changing else 2.4    # 변경중엔 넓게: 추월 대상 측면클리어 전까지 계속 감지
        for (ox,oy,ovx,ovy) in self.objs:
            dx=ox-ex; dy=oy-ey
            fl=ce*dx+se*dy; lt=-se*dx+ce*dy
            if 0.0<fl<brake_win and abs(lt)<lat_gate:
                ov=ovx*ce+ovy*se                                  # NPC 종속도
                # 추월중(out/boost) 저속대상: 옆으로 벗어나는 중엔 제동 안 함(속도 유지해야 넘김).
                # ※ 순환버그 방지 — 판정을 순항속도(v_set) 기준으로. 'ego 현재속도 기준'이면 ego가
                #   앞차 속도로 막혀있을때 스킵조건 깨져 계속 제동→앞차속도에 영영 갇힘(급가속 불가).
                # 횡간격(차폭) 확보 or 6m 이상 남았으면 통과 계속. 6m내+횡미클리어만 비상제동(램 방지).
                if self.ot_phase is not None and ov<self.v_set-1.0 and (abs(lt)>CAR_W or fl>6.0):
                    continue
                v_stop=math.sqrt(max(0.0, 2*self.b_emg*max(0.0, fl-3.2)))
                # 느린 대상(catch-up/피추월)엔 속도 안더함 → 완전 제동해 램 방지. 빠른(멀어지는) 대상만 속도 반영
                add = ov if ov>=self.ego_v-0.5 else 0.0
                v_cmd=min(v_cmd, v_stop+max(0.0,add))
        # 곡률기반 코너속도 상한: a_y=v²·κ ≤ a_y_max → 급커브 이탈·전복 봉쇄 (물리한계 준수)
        # 추월 기동 중엔 완화 (직선 확인됨 → 곡률은 차선변경 블렌드뿐, 급가속 위해 허용)
        ay_corner=self.a_y_max*(3.0 if self.ot_phase is not None else 1.0)  # 추월중 swerve 인공곡률 완화(직선서만 추월)
        v_cmd=min(v_cmd, self._corner_speed_limit(route, ay_corner))
        v_cmd=max(0.0, min(v0, v_cmd))
        self._publish(route, self.links[self.ref_link], v_cmd, decision)

    # ── 경로/발행 ──
    def _chain_pts(self, chain, ex, ey):
        pts=[]
        for L in chain:
            for p in L.pts:
                if pts and (pts[-1][0]-p[0])**2+(pts[-1][1]-p[1])**2<0.04: continue
                pts.append((p[0],p[1]))
        re=self._resample(pts,0.5)
        ci=min(range(len(re)),key=lambda i:(re[i][0]-ex)**2+(re[i][1]-ey)**2)
        return re[ci:]

    def _swerve_route(self, cur, ex, ey, eh, overtake=False):
        """HDMAP 링크 중심선을 base로, 전방 최근접 blocking 장애물을 횡방향으로 직접 우회하는
           corridor를 계산. HDMAP 인접차선(dst_link) 불필요 — 노드(중심선)+장애물 기하로 우회경로 생성.
           장애물 없으면 ego의 현재 offset를 부드럽게 0으로(원차선 복귀). 반환 (route[(x,y)], tgt|None)."""
        ref=self._resample(self._chain_pts_raw(self._forward(cur,self.route_len)),0.5)
        n=len(ref)
        if n<5: return [(p[0],p[1]) for p in ref], None
        S=[0.0]*n; TH=[0.0]*n
        for i in range(1,n):
            S[i]=S[i-1]+math.hypot(ref[i][0]-ref[i-1][0], ref[i][1]-ref[i-1][1])
        for i in range(n):
            a=max(0,i-1); b=min(n-1,i+1)
            TH[i]=math.atan2(ref[b][1]-ref[a][1], ref[b][0]-ref[a][0])
        ci=min(range(n),key=lambda i:(ref[i][0]-ex)**2+(ref[i][1]-ey)**2)
        th0=TH[ci]
        d_ego=-math.sin(th0)*(ex-ref[ci][0])+math.cos(th0)*(ey-ref[ci][1])   # ego 횡offset (+좌)
        # 전방 최근접 blocking 장애물 (같은방향·저속) → 중심선 투영
        s_lo = -(CAR_LEN+4.0) if overtake else 1.0     # 추월중엔 옆/약간뒤 대상도 유지(복귀는 상태머신 제어)
        tgt=None
        for (ox,oy,ovx,ovy) in self.objs:
            k=min(range(n),key=lambda i:(ref[i][0]-ox)**2+(ref[i][1]-oy)**2)
            th=TH[k]; ov=ovx*math.cos(th)+ovy*math.sin(th)
            if ov<SAME_DIR_MIN: continue                                # 역주행 제외
            if (not overtake) and ov>self.ego_v-0.5: continue           # 추월중엔 ego감속해도 대상 유지
            s_rel=S[k]-S[ci]
            if s_rel<s_lo or s_rel>55.0: continue
            lat=-math.sin(th)*(ox-ref[k][0])+math.cos(th)*(oy-ref[k][1])
            if abs(lat)>self.block_lat+2.0: continue                    # 경로서 크게 벗어난 차 무시
            if tgt is None or s_rel<tgt[0]: tgt=(s_rel,lat,k,ov)
        V=max(self._ev,3.0)
        if tgt is None:
            # 장애물 없음 → 현재 offset를 횡가속 한계내에서 0으로 복귀
            Lx=max(6.0, math.pi*V*math.sqrt(max(abs(d_ego),0.05)/(2.0*self.a_y_max)))
            rate=abs(d_ego)/Lx; out=[]
            for i in range(ci,n):
                off=math.copysign(max(0.0,abs(d_ego)-rate*(S[i]-S[ci])), d_ego)
                th=TH[i]; out.append((ref[i][0]-math.sin(th)*off, ref[i][1]+math.cos(th)*off))
            return out, None
        s_rel,lat_obs,k_obs,ov=tgt
        # ── 우회 목표차선(adj) 결정 ──
        #   추월 기동중: 시작때 락한 adj 링크 유지(ego 움직여도 재탐색 안함→flip-flop 방지).
        #   신규: 장애물 반대쪽 우선, HDMAP 차선변경 허용 + 실제 평행 인접차선 있는 쪽만.
        if overtake and self.ot_side is not None and self._ot_adj is not None and self._ot_adj in self.links:
            side=self.ot_side; adj=self.links[self._ot_adj]
        elif overtake and self.ot_side is not None:
            side=self.ot_side; adj=self._adjacent_lane(ex,ey,th0,side)
        else:
            order=[-1.0,1.0] if lat_obs>0.0 else [1.0,-1.0]
            side=None; adj=None
            for cand in order:
                if not (cur.can_l if cand>0 else cur.can_r): continue   # HDMAP 차선변경 허용 방향만
                a=self._adjacent_lane(ex,ey,th0,cand)
                if a is not None: side=cand; adj=a; break
        if adj is None:
            # 실제 옆차선 없음 → 우회 안함(원차선 복귀). blocker(tgt)는 반환 → 추종/감속 유지.
            self._sw_can_offset=False; self._sw_side=None; self._sw_adj=None
            Lx=max(6.0, math.pi*V*math.sqrt(max(abs(d_ego),0.05)/(2.0*self.a_y_max)))
            rate=abs(d_ego)/Lx; out=[]
            for i in range(ci,n):
                off=math.copysign(max(0.0,abs(d_ego)-rate*(S[i]-S[ci])), d_ego)
                th=TH[i]; out.append((ref[i][0]-math.sin(th)*off, ref[i][1]+math.cos(th)*off))
            return out, (s_rel,lat_obs,ov)
        self._sw_can_offset=True; self._sw_side=side; self._sw_adj=adj.idx
        # 경로 = 실제 adj차선으로 lane-change 후 그 차선 실제 geometry 추종 → 항상 도로 위(on-road).
        #   transition은 선형·단축(12m) → 매사이클 ego서 재빌드해도 옆차선 진입 commit (cosine은 시작slope0→commit실패).
        return self._lane_change_route(cur, ex, ey, adj, trans_m=12.0), (s_rel, lat_obs, ov)

    def _inlane_avoid(self, cur, ex, ey, eh, blk_lat):
        """추월불가 구간(옆차선 없음): HDMAP 차선 중심선(node) base로 '차선폭 내에서만' 동적장애물 우회.
           장애물 반대쪽으로 nudge해 차폭 분리 확보되면 통과, 안되면 원차선 유지(감속·정지는 속도로직).
           → 오프로드/역주행 없이 노드 기반 회피."""
        ref=self._resample(self._chain_pts_raw(self._forward(cur,self.route_len)),0.5)
        n=len(ref)
        if n<5: return [(p[0],p[1]) for p in ref]
        S=[0.0]*n; TH=[0.0]*n
        for i in range(1,n):
            S[i]=S[i-1]+math.hypot(ref[i][0]-ref[i-1][0], ref[i][1]-ref[i-1][1])
        for i in range(n):
            a=max(0,i-1); b=min(n-1,i+1)
            TH[i]=math.atan2(ref[b][1]-ref[a][1], ref[b][0]-ref[a][0])
        ci=min(range(n),key=lambda i:(ref[i][0]-ex)**2+(ref[i][1]-ey)**2)
        th0=TH[ci]
        d_ego=-math.sin(th0)*(ex-ref[ci][0])+math.cos(th0)*(ey-ref[ci][1])
        half=(cur.width or 3.5)/2.0
        room=half-CAR_W/2.0-0.15                          # ego 중심이 차선내서 갈 수 있는 최대 |횡offset|
        target=0.0
        if room>0.15 and blk_lat is not None:
            side=-1.0 if blk_lat>0 else 1.0               # 장애물 반대쪽
            cand=side*room
            if abs(cand-blk_lat)>=CAR_W:                  # 차선폭 내에서 차폭만큼 벌어지면 통과 가능
                target=cand
        lat_rate=0.30; out=[]
        for i in range(ci,n):
            de=S[i]-S[ci]
            if abs(target)<1e-3:                          # 회피불가/복귀 → 원차선으로
                off=math.copysign(max(0.0, abs(d_ego)-lat_rate*de), d_ego)
            else:
                off=d_ego+math.copysign(min(abs(target-d_ego), lat_rate*de), target-d_ego)
            th=TH[i]; out.append((ref[i][0]-math.sin(th)*off, ref[i][1]+math.cos(th)*off))
        return out

    def _route_to_link(self, ex, ey, ref_link):
        """ego 현재위치 → 기준 링크 중심선으로 lc_trans 동안 lateral 블렌드 후 전방 추종.
           직진(ego가 이미 링크 위)이면 블렌드≈0, 차선변경(ego 3.5m 벗어남)이면 부드럽게 끌어당김."""
        ref=self._resample(self._chain_pts_raw(self._forward(ref_link,self.route_len)),0.5)
        if len(ref)<2: return ref
        ci=min(range(len(ref)),key=lambda i:(ref[i][0]-ex)**2+(ref[i][1]-ey)**2)
        # ego의 기준선 대비 부호있는 횡offset
        j1=min(ci+1,len(ref)-1)
        th0=math.atan2(ref[j1][1]-ref[ci][1], ref[j1][0]-ref[ci][0])
        d0=-math.sin(th0)*(ex-ref[ci][0])+math.cos(th0)*(ey-ref[ci][1])
        # 차선변경 길이 Lx를 횡가속 한계로 결정 (cosine블렌드 peak a_y = V²·(d0/2)(π/Lx)² = a_y_max)
        #   Lx = π·V·√(|d0|/(2·a_y_max))  → 물리적 실행가능한 최소 종거리. 선형rate=|d0|/Lx (재-anchor 무관 일정)
        V=max(self._ev,3.0)
        ay=self.a_y_max*(1.6 if self.ot_phase is not None else 1.0)  # 추월 중엔 샤프하게(빠른 offset→급가속 통과)
        Lx=max(8.0, math.pi*V*math.sqrt(max(abs(d0),0.05)/(2.0*ay)))
        rate=abs(d0)/Lx
        route=[]
        for i in range(len(ref)-ci):
            j=ci+i
            off=math.copysign(max(0.0, abs(d0)-rate*(i*0.5)), d0)
            if abs(off)<1e-3:
                route.append(ref[j])
            else:
                jj=min(j+1,len(ref)-1)
                th=math.atan2(ref[jj][1]-ref[j][1], ref[jj][0]-ref[j][0])
                route.append((ref[j][0]-math.sin(th)*off, ref[j][1]+math.cos(th)*off))
        return route

    def _lane_change_route(self, cur, ex, ey, dst, trans_m=None):
        cur_pts=self._chain_pts(self._forward(cur,self.route_len),ex,ey)
        dst_re=self._resample(self._chain_pts_raw(self._forward(dst,self.route_len)),0.5)
        if len(dst_re)<2: return cur_pts
        trans_n=max(2,int((trans_m or self.lc_trans)/0.5)); route=[]
        for i in range(len(cur_pts)):
            if i<trans_n:
                w=i/max(1,trans_n-1)                                 # 선형 가중(시작slope>0 → 재anchor해도 commit)
                dj=min(range(len(dst_re)),key=lambda j:(dst_re[j][0]-cur_pts[i][0])**2+(dst_re[j][1]-cur_pts[i][1])**2)
                route.append(((1-w)*cur_pts[i][0]+w*dst_re[dj][0],(1-w)*cur_pts[i][1]+w*dst_re[dj][1]))
            else: break
        if route:
            dj=min(range(len(dst_re)),key=lambda j:(dst_re[j][0]-route[-1][0])**2+(dst_re[j][1]-route[-1][1])**2)
            route.extend(dst_re[dj+1:])
        return route

    @staticmethod
    def _chain_pts_raw(chain):
        pts=[]
        for L in chain:
            for p in L.pts:
                if pts and (pts[-1][0]-p[0])**2+(pts[-1][1]-p[1])**2<0.04: continue
                pts.append((p[0],p[1]))
        return pts

    @staticmethod
    def _resample(pts, step=0.5):
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

    @staticmethod
    def _smooth(pts, it=2):
        # 이동평균 스무딩 — 미세 jitter 제거 (follower 안정 + 코너캡 오작동 방지). 끝점 고정.
        pts=list(pts)
        for _ in range(it):
            if len(pts)<5: break
            out=[pts[0], pts[1]]
            for i in range(2,len(pts)-2):
                out.append(((pts[i-2][0]+pts[i-1][0]+pts[i][0]+pts[i+1][0]+pts[i+2][0])/5.0,
                            (pts[i-2][1]+pts[i-1][1]+pts[i][1]+pts[i+1][1]+pts[i+2][1])/5.0))
            out.append(pts[-2]); out.append(pts[-1]); pts=out
        return pts

    def _publish(self, pts, cur, v_cmd, decision):
        pts=self._smooth(self._resample(pts,1.0))   # 1.0m 재샘플 + 스무딩(jitter제거)
        if len(pts)<2: return
        wps=[]
        for i,(x,y) in enumerate(pts):
            j=min(i+1,len(pts)-1); h=math.atan2(pts[j][1]-y,pts[j][0]-x)
            wps.append({"x":x,"y":y,"heading":h,"gear":"D"})
        self.pub_wps.publish(String(data=json.dumps({"waypoints":wps})))
        self.pub_vel.publish(Float32(data=v_cmd))
        pm=Path(); pm.header.frame_id="map"; pm.header.stamp=rospy.Time.now()
        for w in wps:
            ps=PoseStamped(); ps.header=pm.header
            ps.pose.position.x=w["x"]; ps.pose.position.y=w["y"]; pm.poses.append(ps)
        self.pub_path.publish(pm)
        # 모드 ASCII (OpenCV 한글 미지원) — 대시보드 상단 표시용
        if   "급가속" in decision: mtag="OVERTAKE:BOOST"
        elif "진입"   in decision: mtag="OVERTAKE:LANE-CHANGE"
        elif "회피"   in decision: mtag="AVOID:SWERVE"
        else:                      mtag="CRUISE"
        self.pub_mode.publish(String(data="%s | %.0f km/h (cmd %.0f)"%(mtag, self.ego_v*3.6, v_cmd*3.6)))
        rospy.loginfo_throttle(1.0,"[lane_avoid] lane=%s v=%.1f vcmd=%.1f NPC=%d → %s",
                               cur.lane,self.ego_v,v_cmd,len(self.objs),decision)

    def spin(self):
        r=rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try: self._plan()
            except Exception as e: rospy.logwarn_throttle(2.0,"[lane_avoid] 예외: %s",e)
            r.sleep()


if __name__=="__main__":
    try: LaneAvoid().spin()
    except rospy.ROSInterruptException: pass
