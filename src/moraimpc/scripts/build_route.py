#!/usr/bin/env python3
# 선택 노드 시퀀스 -> 무방향 최단 도로라우팅(하이브리드) -> 스무싱 -> hdmap_path.json + 경로 HTML
#  세그먼트가 직선대비 과하게 우회하면(교차로 등) 노드 직접연결로 대체.
import json, zipfile, math, heapq
from collections import defaultdict

ZIP = "/mnt/c/Users/Hyunsug2014/Downloads/drive-download-20260701T154301Z-3-001.zip"
OUT_PATH = "/home/coss/catkin_ws/src/moraimpc/data/hdmap_path.json"
OUT_HTML = "/mnt/c/Users/Hyunsug2014/Downloads/hdmap_route.html"
DETOUR = 2.5   # 직선*2.5+20m 초과 라우팅이면 직접연결

SEQ = ["A1256W000793","A1256W000857","A1256W000845","A1256W000573","A1256W000563",
       "A1256W000866","A1256W000094","A1256W000560","A1256W000558","A1256W000549",
       "A1256W000492","A1256W000599","A1256W000524","A1256W000499","A1256W000526",
       "A1256W000430","A1256W000090","A1256W000089","A1256W000691","A1256W000687",
       "A1256W000740","A1256W000780","A1256W000332","A1256W000127","A1256W000134",
       "A1256W000131"]

with zipfile.ZipFile(ZIP) as z:
    nodes = json.loads(z.read('node_set.json'))
    links = json.loads(z.read('link_set.json'))
nbi = {n['idx']: n for n in nodes}

def llen(L):
    p = L['points']; return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(p, p[1:]))

# 무방향 그래프 + 노드쌍->링크(점 복원용)
uadj = defaultdict(list)
pair = defaultdict(list)   # (u,v) -> [(pts, length)]  (v방향으로 정렬된 점열)
for L in links:
    f, t = L['from_node_idx'], L['to_node_idx']; d = llen(L)
    P = [[q[0], q[1]] for q in L['points']]
    uadj[f].append((t, d)); uadj[t].append((f, d))
    pair[(f, t)].append((P, d)); pair[(t, f)].append((P[::-1], d))

def dij(a, b):
    pq = [(0, a)]; dist = {a: 0}; prev = {}
    while pq:
        d, u = heapq.heappop(pq)
        if u == b: break
        if d > dist.get(u, 1e18): continue
        for v, w in uadj[u]:
            nd = d + w
            if nd < dist.get(v, 1e18):
                dist[v] = nd; prev[v] = u; heapq.heappush(pq, (nd, v))
    if b not in dist: return None, 0
    path = [b]
    while path[-1] != a: path.append(prev[path[-1]])
    return path[::-1], dist[b]

def seg_points(u, v):
    cand = pair.get((u, v))
    if not cand: return None
    P, _ = min(cand, key=lambda c: c[1])
    return P

def interp(p, q, step=0.5):
    d = math.hypot(q[0]-p[0], q[1]-p[1]); n = max(1, int(d/step))
    return [[p[0]+(q[0]-p[0])*i/n, p[1]+(q[1]-p[1])*i/n] for i in range(n+1)]

route = []
def add(pts):
    for p in pts:
        if route and (route[-1][0]-p[0])**2 + (route[-1][1]-p[1])**2 < 0.04: continue
        route.append([p[0], p[1]])

print("=== 세그먼트 (도로추종 / 직접연결) ===")
for a, b in zip(SEQ, SEQ[1:]):
    pa, pb = nbi[a]['point'], nbi[b]['point']
    straight = math.hypot(pa[0]-pb[0], pa[1]-pb[1])
    npath, rd = dij(a, b)
    if npath and rd <= DETOUR*straight + 20:
        for u, v in zip(npath, npath[1:]):
            sp = seg_points(u, v)
            if sp: add(sp)
        mode = "도로추종(%dm)" % rd
    else:
        add(interp(pa, pb))
        mode = "직접연결(%dm 우회회피)" % (rd if npath else 0)
    print("  %s->%s  직선%.0fm  %s" % (a[-3:], b[-3:], straight, mode))

raw_len = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(route, route[1:]))

def resample(pts, step=0.5):
    out = [pts[0][:]]; carry = 0.0
    for i in range(1, len(pts)):
        ax, ay = out[-1]; bx, by = pts[i]
        seg = math.hypot(bx-ax, by-ay)
        if seg < 1e-9: continue
        d = seg; sx, sy = ax, ay
        while carry + d >= step:
            t = (step-carry)/d; nx, ny = sx+(bx-sx)*t, sy+(by-sy)*t
            out.append([nx, ny]); sx, sy = nx, ny
            d = math.hypot(bx-sx, by-sy); carry = 0.0
        carry += d
    out.append(pts[-1][:]); return out

def smooth(pts, w):
    n = len(pts); out = []
    for i in range(n):
        lo = max(0, i-w); hi = min(n, i+w+1); seg = pts[lo:hi]
        out.append([sum(p[0] for p in seg)/len(seg), sum(p[1] for p in seg)/len(seg)])
    out[0] = pts[0][:]; out[-1] = pts[-1][:]; return out

def _norm(a): return (a+math.pi) % (2*math.pi) - math.pi
def despike(pts, max_turn=100, iters=80):
    thr = math.radians(max_turn); pts = [p[:] for p in pts]
    for _ in range(iters):
        out = [pts[0]]; removed = False; i = 1
        while i < len(pts)-1:
            a, b, c = out[-1], pts[i], pts[i+1]
            h1 = math.atan2(b[1]-a[1], b[0]-a[0]); h2 = math.atan2(c[1]-b[1], c[0]-b[0])
            if abs(_norm(h2-h1)) > thr:   # 반전 spike apex 제거
                removed = True; i += 1; continue
            out.append(b); i += 1
        out.append(pts[-1]); pts = out
        if not removed: break
    return pts

def chaikin(pts, iters):
    for _ in range(iters):
        out = [pts[0][:]]
        for a, b in zip(pts, pts[1:]):
            out.append([0.75*a[0]+0.25*b[0], 0.75*a[1]+0.25*b[1]])
            out.append([0.25*a[0]+0.75*b[0], 0.25*a[1]+0.75*b[1]])
        out.append(pts[-1][:]); pts = out
    return pts

def maxkink_of(pts):
    m = 0.0
    for i in range(1, len(pts)-1):
        m = max(m, abs(_norm(math.atan2(pts[i+1][1]-pts[i][1], pts[i+1][0]-pts[i][0]) -
                             math.atan2(pts[i][1]-pts[i-1][1], pts[i][0]-pts[i-1][0]))))
    return m

# ── 실제 회전반경 측정 (0.5m 노이즈 대신 ±step*0.5m 기준선으로 3점 Menger κ) ──
def coarse_kappa(pts, i, step):
    n = len(pts)
    a = pts[max(0, i-step)]; b = pts[i]; c = pts[min(n-1, i+step)]
    A = math.hypot(b[0]-a[0], b[1]-a[1]); B = math.hypot(c[0]-b[0], c[1]-b[1])
    C = math.hypot(c[0]-a[0], c[1]-a[1])
    ar = abs((b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1])) / 2.0
    return 0.0 if A*B*C < 1e-9 else 4*ar/(A*B*C)

def max_coarse_kappa(pts, step=6):
    return max((coarse_kappa(pts, i, step) for i in range(len(pts))), default=0.0)

# ── 최소 회전반경 강제: R < R_min 커브를 ±step 창 chord 쪽으로 완화 (양끝 고정) ──
#   차량 최대조향 35°→최소반경 3.86m. R_MIN=5m(steer 28°)로 여유 확보.
def enforce_min_radius(pts, r_min=5.0, step=6, iters=400):
    kt = 1.0 / r_min
    pts = [p[:] for p in pts]; n = len(pts)
    for _ in range(iters):
        ks = [coarse_kappa(pts, i, step) for i in range(n)]
        if max(ks) <= kt: break
        newp = [p[:] for p in pts]
        for i in range(1, n-1):
            if ks[i] > kt:
                lo = max(0, i-step); hi = min(n-1, i+step)
                mx = (pts[lo][0] + pts[hi][0]) / 2.0
                my = (pts[lo][1] + pts[hi][1]) / 2.0
                lam = 0.5 * min(1.0, ks[i]/kt - 1.0)   # 초과분 비례 완화
                newp[i][0] = pts[i][0] + lam*(mx - pts[i][0])
                newp[i][1] = pts[i][1] + lam*(my - pts[i][1])
        newp[0] = pts[0]; newp[-1] = pts[-1]            # 시작/끝 고정
        pts = newp
    return pts

# despike -> 리샘플 -> 반복(미세spike 재제거 + 이동평균) 최대꺾임<10도까지
sm = resample(despike(route, 85), 0.5)
for _ in range(80):
    if math.degrees(maxkink_of(sm)) < 10: break
    sm = despike(sm, 20)        # 남은 국소 spike 물리 제거
    sm = smooth(sm, 6)
sm = resample(sm, 0.5)
sm = smooth(sm, 3)

# ── 최소반경 강제 (차량 물리한계 R>3.86m → 여유 R_MIN=5.5m, steer 26°) ──
#   국소(±1.5m, step=3) + 광역(±3m, step=6) 2단계로 0.5m 스케일 타이트니스까지 완화
def kmax05(pts):
    n=len(pts)
    return max((coarse_kappa(pts,i,1) for i in range(n)), default=0.0)
# 컨트롤러와 동일한 곡률 (±1점 중앙차분 heading의 dh/ds) — 실제 ref κ
def ctrl_kappa_max(pts):
    n=len(pts); h=[0.0]*n
    if n<2: return 0.0
    h[0]=math.atan2(pts[1][1]-pts[0][1], pts[1][0]-pts[0][0])
    for i in range(1,n-1):
        h[i]=math.atan2(pts[i+1][1]-pts[i-1][1], pts[i+1][0]-pts[i-1][0])
    h[-1]=math.atan2(pts[-1][1]-pts[-2][1], pts[-1][0]-pts[-2][0])
    km=0.0
    for i in range(n):
        lo=max(0,i-1); hi=min(n-1,i+1)
        if lo==hi: continue
        dh=_norm(h[hi]-h[lo]); ds=math.hypot(pts[hi][0]-pts[lo][0], pts[hi][1]-pts[lo][1])
        if ds>1e-6: km=max(km, abs(dh/ds))
    return km

R_MIN = 6.0
k_before = ctrl_kappa_max(sm)
sm = enforce_min_radius(sm, r_min=R_MIN, step=6)   # 실제 회전반경(±3m) 완화
sm = resample(sm, 0.5)
for _ in range(6):                                  # 강한 이동평균 반복 — 0.5m 헤딩 노이즈 제거
    sm = smooth(sm, 5)
sm = resample(sm, 0.5)                              # 균일 0.5m 간격 복원

# ── 끝단 kink 트림: 고정 endpoint 접근부 헤어핀 꼬리 제거 (최대 15m) ──
#   실제 커브는 매끄러운데 마지막 몇 점만 endpoint 고정 탓 첨점 → 그 꼬리를 잘라
#   매끄러운 지점에서 종료 (차는 끝 1~2m 못 가도 무방).
def tail_kappa(pts, i):
    lo = max(0, i-1); hi = min(len(pts)-1, i+1)
    if lo == hi: return 0.0
    h1 = math.atan2(pts[i][1]-pts[lo][1], pts[i][0]-pts[lo][0])
    h2 = math.atan2(pts[hi][1]-pts[i][1], pts[hi][0]-pts[i][0])
    ds = math.hypot(pts[hi][0]-pts[lo][0], pts[hi][1]-pts[lo][1])
    return abs(_norm(h2-h1))/ds if ds > 1e-6 else 0.0
kt = 1.0 / R_MIN; trimmed = 0
while len(sm) > 12 and trimmed < 30 and tail_kappa(sm, len(sm)-2) > kt:
    sm.pop(); trimmed += 1
sm = smooth(sm, 3)

k_after = ctrl_kappa_max(sm)
print("최소반경 강제(컨트롤러 ref κ): κ_max %.3f(R=%.1fm) → %.3f(R=%.1fm)  [steer %.0f° → %.0f°, 한계35°], 끝단트림 %d점"
      % (k_before, 1/k_before if k_before>0 else 999,
         k_after,  1/k_after  if k_after>0  else 999,
         math.degrees(math.atan(k_before*2.7)), math.degrees(math.atan(k_after*2.7)), trimmed))

def norm(a): return (a+math.pi) % (2*math.pi) - math.pi
maxgap = max(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(sm, sm[1:]))
maxkink = max((abs(norm(math.atan2(sm[i+1][1]-sm[i][1], sm[i+1][0]-sm[i][0]) -
              math.atan2(sm[i][1]-sm[i-1][1], sm[i][0]-sm[i-1][0]))) for i in range(1, len(sm)-1)), default=0)
tot = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(sm, sm[1:]))

wps = [{"x": p[0], "y": p[1],
        "heading": math.atan2(sm[min(i+1, len(sm)-1)][1]-p[1], sm[min(i+1, len(sm)-1)][0]-p[0]),
        "gear": "D"} for i, p in enumerate(sm)]
json.dump({"waypoints": wps}, open(OUT_PATH, "w"))

# ── 최악 꺾임 위치 진단 ──
kinks = []
for i in range(1, len(sm)-1):
    k = abs(norm(math.atan2(sm[i+1][1]-sm[i][1], sm[i+1][0]-sm[i][0]) -
                 math.atan2(sm[i][1]-sm[i-1][1], sm[i][0]-sm[i-1][0])))
    kinks.append((k, i))
kinks.sort(reverse=True)
print("=== 최악 꺾임 top5 ===")
for k, i in kinks[:5]:
    print("  %.0f도  @(%.1f, %.1f) idx%d/%d" % (math.degrees(k), sm[i][0], sm[i][1], i, len(sm)))

# ── PNG 렌더(Agg, 화면불필요) ──
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(13, 13))
for lk in links:
    p = lk['points']
    ax.plot([q[0] for q in p], [q[1] for q in p], '-', color='#c8d0d8', lw=0.6, zorder=1)
ax.plot([p[0] for p in sm], [p[1] for p in sm], '-', color='#ff9000', lw=2.2, zorder=3, label='route')
for i, s in enumerate(SEQ):
    pt = nbi[s]['point']
    ax.plot(pt[0], pt[1], 'o', color='#20a020', ms=7, zorder=4)
    ax.annotate(str(i), (pt[0], pt[1]), fontsize=8, color='#003000', zorder=5)
for k, i in kinks[:8]:
    if math.degrees(k) > 25:
        ax.plot(sm[i][0], sm[i][1], 'x', color='red', ms=12, mew=3, zorder=6)
ax.plot(sm[0][0], sm[0][1], '*', color='lime', ms=22, mec='k', zorder=7, label='start')
ax.plot(sm[-1][0], sm[-1][1], '*', color='red', ms=22, mec='k', zorder=7, label='end')
ax.set_aspect('equal'); ax.grid(True, alpha=0.3); ax.legend()
ax.set_title('route %.0fm  maxkink %.0f deg  (red x = kink>25deg)' % (tot, math.degrees(maxkink)))
PNG = "/mnt/c/Users/Hyunsug2014/Downloads/route_check.png"
plt.savefig(PNG, dpi=80, bbox_inches='tight'); plt.close()
print("PNG:", PNG)
print("=== 결과 ===")
print("wp %d, 길이 %.1fm" % (len(wps), tot))
print("품질: 최대간격 %.2fm, 최대꺾임 %.1f도  %s" % (maxgap, math.degrees(maxkink),
      "OK" if math.degrees(maxkink) < 15 else "급커브주의"))

# ── HTML ──
Ldata = [[[round(q[0], 2), round(q[1], 2)] for q in lk["points"][::3]] for lk in links]
ROUTE = [[round(p[0], 2), round(p[1], 2)] for p in sm[::2]]
SELPT = [{"x": nbi[s]["point"][0], "y": nbi[s]["point"][1], "ord": i} for i, s in enumerate(SEQ)]
NB = [{"x": n["point"][0], "y": n["point"][1]} for n in nodes if n["idx"] not in SEQ]
bounds = {"nx": min(n["point"][0] for n in nodes), "xx": max(n["point"][0] for n in nodes),
          "ny": min(n["point"][1] for n in nodes), "xy": max(n["point"][1] for n in nodes)}

HTML = r"""<!DOCTYPE html><html lang="ko"><head><meta charset="utf-8"><title>HDMAP 경로</title>
<style>*{box-sizing:border-box}html,body{margin:0;height:100%;font-family:'Malgun Gothic',sans-serif}
#wrap{display:flex;height:100%}#map{flex:1;background:#0d1117;cursor:grab}#map:active{cursor:grabbing}
#side{width:260px;background:#161b22;color:#e6edf3;padding:14px;font-size:14px}b{color:#7ee787}</style></head>
<body><div id="wrap"><svg id="map"></svg><div id="side"><h2>🛣️ 생성 경로</h2><div id="stat"></div>
<hr><div style="color:#8b949e;font-size:12px">노랑=주행경로<br>초록점=선택노드(순번)<br>초록별=시작 빨강별=끝<br>스크롤=줌, 드래그=이동</div></div></div>
<script>
const LINKS=__L__,ROUTE=__R__,SEL=__S__,NB=__NB__,ST=__ST__,BD=__BD__;
const svg=document.getElementById('map'),NS='http://www.w3.org/2000/svg';
document.getElementById('stat').innerHTML='<b>'+ST.n+'</b> wp &nbsp; <b>'+ST.len+'m</b><br>최대간격 '+ST.gap+'m<br>최대꺾임 '+ST.kink+'&deg;<br>노드 '+SEL.length+'개 경유';
const pad=15;let vb={x:BD.nx-pad,y:-BD.xy-pad,w:(BD.xx-BD.nx)+2*pad,h:(BD.xy-BD.ny)+2*pad};
function setVB(){svg.setAttribute('viewBox',vb.x+' '+vb.y+' '+vb.w+' '+vb.h);}
const g=document.createElementNS(NS,'g');svg.appendChild(g);
function poly(pts,st,w,op){const p=document.createElementNS(NS,'polyline');p.setAttribute('points',pts.map(q=>q[0]+','+(-q[1])).join(' '));p.setAttribute('fill','none');p.setAttribute('stroke',st);p.setAttribute('stroke-width',w);p.setAttribute('stroke-opacity',op);p.setAttribute('stroke-linejoin','round');g.appendChild(p);}
function dot(x,y,r,f){const c=document.createElementNS(NS,'circle');c.setAttribute('cx',x);c.setAttribute('cy',-y);c.setAttribute('r',r);c.setAttribute('fill',f);g.appendChild(c);}
for(const l of LINKS)poly(l,'#30475e',0.5,0.6);
for(const n of NB)dot(n.x,n.y,0.7,'#7a2a2a');
poly(ROUTE,'#ffd000',2.6,0.3);poly(ROUTE,'#ffd000',1.2,1);
for(const s of SEL){dot(s.x,s.y,1.8,'#2ecc40');const t=document.createElementNS(NS,'text');t.setAttribute('x',s.x+1.6);t.setAttribute('y',-s.y-1.6);t.setAttribute('font-size','3');t.setAttribute('fill','#fff');t.textContent=s.ord;g.appendChild(t);}
function star(x,y,c){const e=document.createElementNS(NS,'circle');e.setAttribute('cx',x);e.setAttribute('cy',-y);e.setAttribute('r','3.2');e.setAttribute('fill',c);e.setAttribute('stroke','#fff');e.setAttribute('stroke-width','0.6');g.appendChild(e);}
star(ROUTE[0][0],ROUTE[0][1],'#20e020');star(ROUTE[ROUTE.length-1][0],ROUTE[ROUTE.length-1][1],'#e02020');
svg.addEventListener('wheel',e=>{e.preventDefault();const r=svg.getBoundingClientRect();const mx=vb.x+(e.clientX-r.left)/r.width*vb.w,my=vb.y+(e.clientY-r.top)/r.height*vb.h;const f=e.deltaY<0?0.85:1.18;vb.x=mx-(mx-vb.x)*f;vb.y=my-(my-vb.y)*f;vb.w*=f;vb.h*=f;setVB();},{passive:false});
let dr=false,lx,ly;svg.addEventListener('mousedown',e=>{dr=true;lx=e.clientX;ly=e.clientY;});window.addEventListener('mouseup',()=>dr=false);
window.addEventListener('mousemove',e=>{if(!dr)return;const r=svg.getBoundingClientRect();vb.x-=(e.clientX-lx)/r.width*vb.w;vb.y-=(e.clientY-ly)/r.height*vb.h;lx=e.clientX;ly=e.clientY;setVB();});
setVB();
</script></body></html>"""
HTML = (HTML.replace("__L__", json.dumps(Ldata)).replace("__R__", json.dumps(ROUTE))
            .replace("__S__", json.dumps(SELPT)).replace("__NB__", json.dumps(NB))
            .replace("__BD__", json.dumps(bounds))
            .replace("__ST__", json.dumps({"n": len(wps), "len": round(tot, 1),
                     "gap": round(maxgap, 2), "kink": round(math.degrees(maxkink), 1)})))
open(OUT_HTML, "w", encoding="utf-8").write(HTML)
print("경로파일:", OUT_PATH)
print("경로 HTML:", OUT_HTML)
