#!/usr/bin/env python3
# MORAI HDMAP -> 자체완결 인터랙티브 HTML (Chrome에서 열기). 의존성 없음.
#  줌/팬 + 노드 호버(idx 표시) + 노드 클릭시 경로에 순서대로 추가 -> 경로 직접 작성
import json, zipfile

ZIP = "/mnt/c/Users/Hyunsug2014/Downloads/drive-download-20260701T154301Z-3-001.zip"
OUT = "/mnt/c/Users/Hyunsug2014/Downloads/hdmap_viz.html"

with zipfile.ZipFile(ZIP) as z:
    nodes = json.loads(z.read('node_set.json'))
    links = json.loads(z.read('link_set.json'))

# 노드: 순번 부여
N = [{"n": i, "idx": nd["idx"], "x": nd["point"][0], "y": nd["point"][1],
      "stop": bool(nd.get("on_stop_line"))} for i, nd in enumerate(nodes)]
# 링크: 점 서브샘플(2칸마다) + from/to node
L = []
for lk in links:
    p = lk["points"]
    pts = [[round(q[0], 2), round(q[1], 2)] for q in p[::2]]
    if pts and pts[-1] != [round(p[-1][0], 2), round(p[-1][1], 2)]:
        pts.append([round(p[-1][0], 2), round(p[-1][1], 2)])
    L.append({"f": lk["from_node_idx"], "t": lk["to_node_idx"], "p": pts})

# Ego (옵션)
EGO = None
try:
    import rospy
    from morai_msgs.msg import EgoVehicleStatus
    rospy.init_node('hdmap_html', anonymous=True, disable_signals=True)
    e = rospy.wait_for_message(
        '/localization/ego_status', EgoVehicleStatus, timeout=3.0)
    EGO = {"x": e.position.x, "y": e.position.y, "h": e.heading}
    print("Ego (%.1f, %.1f)" % (e.position.x, e.position.y))
except Exception as ex:
    print("Ego 없음(무시):", ex)

HTML = r"""<!DOCTYPE html><html lang="ko"><head><meta charset="utf-8">
<title>MORAI HDMAP</title>
<style>
 *{box-sizing:border-box} html,body{margin:0;height:100%;font-family:'Malgun Gothic',sans-serif}
 #wrap{display:flex;height:100%}
 #map{flex:1;background:#0d1117;cursor:grab} #map:active{cursor:grabbing}
 #side{width:320px;background:#161b22;color:#e6edf3;padding:12px;overflow:auto}
 #side h3{margin:6px 0} .btn{margin:3px;padding:6px 10px;border:0;border-radius:5px;background:#238636;color:#fff;cursor:pointer}
 .btn.g{background:#30363d} textarea{width:100%;height:90px;margin-top:6px;background:#0d1117;color:#7ee787;border:1px solid #30363d;border-radius:5px;font-size:13px}
 ol{padding-left:22px} li{margin:2px 0}
 #tip{position:fixed;background:#000d;color:#fff;padding:4px 8px;border-radius:5px;font-size:13px;pointer-events:none;display:none;z-index:9}
 .hint{color:#8b949e;font-size:12px;line-height:1.5}
</style></head><body><div id="wrap">
<svg id="map"></svg>
<div id="side">
 <h3>🛣️ 경로 만들기</h3>
 <div class="hint">노드(빨강점) <b>클릭</b> → 순서대로 추가<br>스크롤=줌, 드래그=이동, 호버=idx</div>
 <div style="margin:8px 0"><button class="btn" onclick="undo()">↶ 되돌리기</button><button class="btn g" onclick="clr()">전체삭제</button></div>
 <b>선택 순서:</b><ol id="list"></ol>
 <b>노드번호 시퀀스(복사해서 주면 경로생성):</b>
 <textarea id="seq" readonly></textarea>
 <div id="cnt" class="hint"></div>
</div></div><div id="tip"></div>
<script>
const NODES=__NODES__, LINKS=__LINKS__, EGO=__EGO__;
const svg=document.getElementById('map'), tip=document.getElementById('tip');
const SVGNS='http://www.w3.org/2000/svg';
// y 뒤집기(화면 좌표). plotY=-y
let minX=1e9,maxX=-1e9,minY=1e9,maxY=-1e9;
for(const n of NODES){minX=Math.min(minX,n.x);maxX=Math.max(maxX,n.x);minY=Math.min(minY,-n.y);maxY=Math.max(maxY,-n.y);}
const pad=20; let vb={x:minX-pad,y:minY-pad,w:(maxX-minX)+2*pad,h:(maxY-minY)+2*pad};
function setVB(){svg.setAttribute('viewBox',vb.x+' '+vb.y+' '+vb.w+' '+vb.h);}
const g=document.createElementNS(SVGNS,'g'); svg.appendChild(g);
// 링크
for(const lk of LINKS){const pl=document.createElementNS(SVGNS,'polyline');
 pl.setAttribute('points',lk.p.map(q=>q[0]+','+(-q[1])).join(' '));
 pl.setAttribute('fill','none');pl.setAttribute('stroke','#4a90d9');pl.setAttribute('stroke-width','0.6');pl.setAttribute('stroke-opacity','0.55');g.appendChild(pl);}
// Ego
if(EGO){const s=document.createElementNS(SVGNS,'circle');s.setAttribute('cx',EGO.x);s.setAttribute('cy',-EGO.y);s.setAttribute('r','3');s.setAttribute('fill','#20e020');s.setAttribute('stroke','#fff');s.setAttribute('stroke-width','0.5');g.appendChild(s);
 const hr=EGO.h*Math.PI/180;const ln=document.createElementNS(SVGNS,'line');ln.setAttribute('x1',EGO.x);ln.setAttribute('y1',-EGO.y);ln.setAttribute('x2',EGO.x+8*Math.cos(hr));ln.setAttribute('y2',-(EGO.y+8*Math.sin(hr)));ln.setAttribute('stroke','#20e020');ln.setAttribute('stroke-width','1');g.appendChild(ln);}
// 노드 + 번호
const sel=[]; const circ={};
for(const n of NODES){const c=document.createElementNS(SVGNS,'circle');
 c.setAttribute('cx',n.x);c.setAttribute('cy',-n.y);c.setAttribute('r','1.4');
 c.setAttribute('fill',n.stop?'#f0a020':'#e02020');c.setAttribute('stroke','#000');c.setAttribute('stroke-width','0.15');c.style.cursor='pointer';
 c.addEventListener('mouseenter',ev=>{tip.style.display='block';tip.textContent='#'+n.n+'  '+n.idx+(n.stop?'  [정지선]':'');c.setAttribute('r','2.4');});
 c.addEventListener('mousemove',ev=>{tip.style.left=(ev.clientX+12)+'px';tip.style.top=(ev.clientY+12)+'px';});
 c.addEventListener('mouseleave',ev=>{tip.style.display='none';c.setAttribute('r',sel.includes(n.n)?'2.2':'1.4');});
 c.addEventListener('click',ev=>{ev.stopPropagation();toggle(n);});
 g.appendChild(c);circ[n.n]=c;
 const t=document.createElementNS(SVGNS,'text');t.setAttribute('x',n.x+1.2);t.setAttribute('y',-n.y-1.2);t.setAttribute('font-size','2.2');t.setAttribute('fill','#8b949e');t.textContent=n.n;t.style.pointerEvents='none';g.appendChild(t);}
function toggle(n){const i=sel.indexOf(n.n);if(i>=0){sel.splice(i,1);circ[n.n].setAttribute('fill',n.stop?'#f0a020':'#e02020');circ[n.n].setAttribute('r','1.4');}else{sel.push(n.n);circ[n.n].setAttribute('fill','#2ecc40');circ[n.n].setAttribute('r','2.2');}render();}
function undo(){if(sel.length){const m=sel.pop();const nn=NODES[m];circ[m].setAttribute('fill',nn.stop?'#f0a020':'#e02020');circ[m].setAttribute('r','1.4');render();}}
function clr(){while(sel.length){const m=sel.pop();const nn=NODES[m];circ[m].setAttribute('fill',nn.stop?'#f0a020':'#e02020');circ[m].setAttribute('r','1.4');}render();}
function render(){const ol=document.getElementById('list');ol.innerHTML='';for(const m of sel){const li=document.createElement('li');li.textContent='#'+m+'  '+NODES[m].idx;ol.appendChild(li);}
 document.getElementById('seq').value=sel.join(', ');document.getElementById('cnt').textContent=sel.length+'개 선택됨';}
// 줌/팬
svg.addEventListener('wheel',ev=>{ev.preventDefault();const r=svg.getBoundingClientRect();const mx=vb.x+(ev.clientX-r.left)/r.width*vb.w;const my=vb.y+(ev.clientY-r.top)/r.height*vb.h;const f=ev.deltaY<0?0.85:1.18;vb.x=mx-(mx-vb.x)*f;vb.y=my-(my-vb.y)*f;vb.w*=f;vb.h*=f;setVB();},{passive:false});
let drag=false,lx,ly;
svg.addEventListener('mousedown',ev=>{drag=true;lx=ev.clientX;ly=ev.clientY;});
window.addEventListener('mouseup',()=>drag=false);
window.addEventListener('mousemove',ev=>{if(!drag)return;const r=svg.getBoundingClientRect();vb.x-=(ev.clientX-lx)/r.width*vb.w;vb.y-=(ev.clientY-ly)/r.height*vb.h;lx=ev.clientX;ly=ev.clientY;setVB();});
setVB();
</script></body></html>"""

HTML = (HTML.replace("__NODES__", json.dumps(N))
            .replace("__LINKS__", json.dumps(L))
            .replace("__EGO__", json.dumps(EGO)))
open(OUT, "w", encoding="utf-8").write(HTML)
print("저장:", OUT, " (%d nodes, %d links)" % (len(N), len(L)))
