#!/usr/bin/env python3
# MORAI HDMAP 시각화: node_set + link_set 를 인터랙티브 GUI 로 표시 (줌/팬 가능)
#  빨강 점 = 노드(idx 라벨), 파랑 선 = 링크, 초록 별 = Ego 현재위치
import json, zipfile, math
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

ZIP = "/mnt/c/Users/Hyunsug2014/Downloads/drive-download-20260701T154301Z-3-001.zip"

with zipfile.ZipFile(ZIP) as z:
    nodes = json.loads(z.read('node_set.json'))
    links = json.loads(z.read('link_set.json'))
print("node %d, link %d" % (len(nodes), len(links)))

# Ego 위치 (있으면 표시)
ego = None
try:
    import rospy
    from morai_msgs.msg import EgoVehicleStatus
    rospy.init_node('viz_hdmap', anonymous=True, disable_signals=True)
    e = rospy.wait_for_message('/Ego_topic', EgoVehicleStatus, timeout=3.0)
    ego = (e.position.x, e.position.y, e.heading)
    print("Ego (%.1f, %.1f) heading=%.1f" % ego)
except Exception as ex:
    print("Ego 위치 못읽음(무시):", ex)

fig, ax = plt.subplots(figsize=(15, 11))

# 링크 (도로)
for L in links:
    p = L['points']
    ax.plot([pt[0] for pt in p], [pt[1] for pt in p], '-',
            color='#4a90d9', lw=1.0, alpha=0.6, zorder=1)

# 노드 + idx 라벨
nx = [n['point'][0] for n in nodes]
ny = [n['point'][1] for n in nodes]
ax.scatter(nx, ny, s=18, c='#e02020', zorder=3)
for n in nodes:
    ax.annotate(n['idx'], (n['point'][0], n['point'][1]),
                fontsize=4.5, color='#202020', zorder=4,
                xytext=(2, 2), textcoords='offset points')

# Ego
if ego:
    ax.scatter([ego[0]], [ego[1]], s=260, marker='*',
               c='#20e020', edgecolors='black', linewidths=1.2, zorder=5, label='Ego')
    h = math.radians(ego[2])
    ax.annotate('', xy=(ego[0] + 10 * math.cos(h), ego[1] + 10 * math.sin(h)),
                xytext=(ego[0], ego[1]),
                arrowprops=dict(arrowstyle='->', color='#20e020', lw=2.5), zorder=6)
    ax.legend(loc='upper right')

ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_title('MORAI HDMAP  —  red=node(idx), blue=link, green star=Ego   [scroll/drag to zoom]')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
plt.tight_layout()
plt.show()
