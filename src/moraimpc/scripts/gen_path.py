#!/usr/bin/env python3
# HDMAP link_set 기반 경로 생성: 현재 차량 위치(live)에서, 차량이 실제 올라탄 차선 링크를 따라 라우팅
import json, math, zipfile
from collections import defaultdict
import rospy
from morai_msgs.msg import EgoVehicleStatus

ZIP = "/mnt/c/Users/Hyunsug2014/Downloads/drive-download-20260701T154301Z-3-001.zip"
TARGET_LEN = 300.0
MAXGAP  = 3.0
MAXTURN = math.radians(55)
OUT = "/home/david/morai-mpc-agent-morai-lio-gps-integration/src/moraimpc/data/hdmap_path.json"

rospy.init_node("gen_path", anonymous=True)
ego = rospy.wait_for_message(
    "/localization/ego_status", EgoVehicleStatus, timeout=5.0)
START = (ego.position.x, ego.position.y)
START_HDG = math.radians(ego.heading)
print("현재 차량: (%.2f, %.2f) heading=%.1f도" % (START[0], START[1], ego.heading))

with zipfile.ZipFile(ZIP) as z:
    links = json.loads(z.read("link_set.json"))
adj = defaultdict(list)
for L in links:
    adj[L["from_node_idx"]].append(L)

def d2(p, q): return (p[0]-q[0])**2 + (p[1]-q[1])**2
def norm(a): return (a + math.pi) % (2*math.pi) - math.pi

# ── 시작 링크 선택: "차량이 올라탄 차선" 우선 ──
#  조건: heading차 < 60도 (역방향/직교 차선 배제)
#  점수: 거리(m)*2 + heading차(rad) — 옆차선(3.5m 차이)보다 내 차선(<1m)이 항상 이김
cands = []
for L in links:
    p = L["points"]
    for i in range(len(p)-1):
        dd = d2(p[i], START)
        if dd > 15**2: continue
        h = math.atan2(p[i+1][1]-p[i][1], p[i+1][0]-p[i][0])
        hd = abs(norm(h - START_HDG))
        if hd > math.radians(60): continue   # 역방향/교차 차선 배제
        dist = math.sqrt(dd)
        cands.append((dist*2.0 + hd, L, i, dist, math.degrees(hd)))
if not cands:
    raise SystemExit("차량 주변 15m에 heading 맞는 링크 없음 — 차량이 도로 위인지 확인")
cands.sort(key=lambda c: c[0])
_, L0, i0, dist0, hd0 = cands[0]
print("시작 링크 %s point %d  (차량거리 %.2fm, heading차 %.1f도)" % (L0["idx"], i0, dist0, hd0))

route = []
def add(pts):
    for p in pts:
        if route and d2(route[-1], p) < 0.04: continue
        route.append([p[0], p[1]])
def plen(pts):
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(pts, pts[1:]))

add(L0["points"][i0:])
cur = L0["to_node_idx"]; used = {L0["idx"]}
stop = "목표거리 도달"
while plen(route) < TARGET_LEN:
    cs = [L for L in adj.get(cur, []) if L["idx"] not in used and L["points"]
          and d2(L["points"][0], route[-1]) < MAXGAP*MAXGAP]
    if not cs: stop = "연결 링크 없음"; break
    lh = math.atan2(route[-1][1]-route[-2][1], route[-1][0]-route[-2][0]) if len(route)>=2 else START_HDG
    def turn(L):
        p = L["points"]; h = math.atan2(p[1][1]-p[0][1], p[1][0]-p[0][0]) if len(p)>=2 else lh
        return abs(norm(h - lh))
    L = min(cs, key=turn)
    if turn(L) > MAXTURN: stop = "급커브 접합부(%.0f도) 종료" % math.degrees(turn(L)); break
    used.add(L["idx"]); add(L["points"]); cur = L["to_node_idx"]

maxgap = max((math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(route, route[1:])), default=0)
maxkink = 0
for i in range(1, len(route)-1):
    h1 = math.atan2(route[i][1]-route[i-1][1], route[i][0]-route[i-1][0])
    h2 = math.atan2(route[i+1][1]-route[i][1], route[i+1][0]-route[i][0])
    maxkink = max(maxkink, abs(norm(h2-h1)))

wps = [{"x": p[0], "y": p[1],
        "heading": math.atan2(route[min(i+1,len(route)-1)][1]-p[1], route[min(i+1,len(route)-1)][0]-p[0]),
        "gear": "D"} for i,p in enumerate(route)]
json.dump({"waypoints": wps}, open(OUT, "w"))
print("wp %d, 길이 %.1fm | 종료: %s" % (len(wps), plen(route), stop))
print("품질: 최대간격 %.2fm, 최대꺾임 %.1f도" % (maxgap, math.degrees(maxkink)))
print("X %.1f~%.1f Y %.1f~%.1f" % (min(w['x'] for w in wps),max(w['x'] for w in wps),min(w['y'] for w in wps),max(w['y'] for w in wps)))
