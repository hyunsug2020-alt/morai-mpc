#!/usr/bin/env python3
"""RViz view of the competition launch (student_v3_follow_claude.sh), all in frame "map".

  /competition/hdmap    MarkerArray (latched): HD-map lanes, global route, operator rules
                        from drivable_rules.json (blocked / lane-change allow / deny, no-go zones)
  /competition/ego      Marker: localization pose (/localization/ego_status, never /Ego_topic)
  /competition/objects  MarkerArray: /Object_topic boxes + velocity arrows
  TF map -> competition_ego from the localization pose, so RViz can follow the car. (MORAI's own
  rosbridge TF map -> base_link is the simulator truth and is not used here.)
"""
import argparse
import json
import math
import os

import rospy
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

COLORS = {
    "lane": (0.45, 0.50, 0.58, 0.9), "opp": (0.28, 0.31, 0.36, 0.9), "route": (0.22, 0.85, 0.54, 1.0),
    "block": (1.0, 0.30, 0.37, 1.0), "allow": (0.24, 0.78, 1.0, 1.0), "deny": (1.0, 0.65, 0.24, 1.0),
    "zone": (1.0, 0.30, 0.37, 0.25), "ego": (1.0, 1.0, 1.0, 0.95),
    "npc": (0.35, 0.55, 1.0, 0.8), "pedestrian": (1.0, 0.85, 0.2, 0.9), "obstacle": (1.0, 0.3, 0.3, 0.85),
}


def color(name):
    return ColorRGBA(*COLORS[name])


def points(seq, z=0.0):
    return [Point(float(p[0]), float(p[1]), z) for p in seq]


def base_marker(ns, marker_id, kind, scale, rgba):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns, marker.id, marker.type, marker.action = ns, marker_id, kind, Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = scale
    marker.color = rgba
    return marker


class CompetitionViz:
    def __init__(self, args):
        with open(args.links) as source:
            self.links = [l for l in json.load(source) if len(l.get("points") or []) >= 2]
        with open(args.route) as source:
            self.route = [(p["x"], p["y"]) for p in json.load(source)["waypoints"]]
        self.rules_path = args.rules
        self.rules_mtime = None
        self.hdmap_pub = rospy.Publisher("/competition/hdmap", MarkerArray, queue_size=1, latch=True)
        self.ego_pub = rospy.Publisher("/competition/ego", Marker, queue_size=1)
        self.objects_pub = rospy.Publisher("/competition/objects", MarkerArray, queue_size=1)
        self.tf = tf2_ros.TransformBroadcaster()
        self.object_ids = 0
        rospy.Subscriber(args.ego_topic, EgoVehicleStatus, self.on_ego, queue_size=1)
        rospy.Subscriber(args.object_topic, ObjectStatusList, self.on_objects, queue_size=1)
        rospy.Timer(rospy.Duration(2.0), lambda _: self.publish_map())
        self.publish_map(force=True)

    def load_rules(self):
        try:
            with open(self.rules_path) as source:
                rules = json.load(source)
            return rules if isinstance(rules, dict) else {}
        except (OSError, ValueError):
            return {}

    def publish_map(self, force=False):
        try:
            mtime = os.path.getmtime(self.rules_path)
        except OSError:
            mtime = None
        if not force and mtime == self.rules_mtime:
            return
        self.rules_mtime = mtime
        rules = self.load_rules()
        link_rules = rules.get("links") or {}
        lanes = base_marker("lanes", 0, Marker.LINE_LIST, 0.15, color("lane"))
        opp = base_marker("lanes_opposite", 0, Marker.LINE_LIST, 0.12, color("opp"))
        for link in self.links:
            target = opp if link.get("opp_traffic") else lanes
            pts = points(link["points"][::2] + [link["points"][-1]])
            for a, b in zip(pts, pts[1:]):
                target.points += [a, b]
        array = MarkerArray(markers=[lanes, opp])
        route = base_marker("route", 0, Marker.LINE_STRIP, 0.45, color("route"))
        route.points = points(self.route[::2], 0.05)
        array.markers.append(route)
        by_id = {link["idx"]: link for link in self.links}
        for index, (link_id, rule) in enumerate(sorted(link_rules.items())):
            link = by_id.get(link_id)
            if link is None or not isinstance(rule, dict):
                continue
            kind = "block" if rule.get("block") else (
                "deny" if "deny" in (rule.get("left"), rule.get("right")) else "allow")
            marker = base_marker("rules", index, Marker.LINE_STRIP, 0.9, color(kind))
            marker.points = points(link["points"], 0.08)
            array.markers.append(marker)
        for index, zone in enumerate(rules.get("zones") or []):
            polygon = zone.get("polygon") or []
            if len(polygon) < 3:
                continue
            outline = base_marker("zones", index, Marker.LINE_STRIP, 0.4, color("block"))
            outline.points = points(polygon + [polygon[0]], 0.1)
            fill = base_marker("zone_fill", index, Marker.TRIANGLE_LIST, 1.0, color("zone"))
            for a, b in zip(polygon[1:], polygon[2:]):            # fan: exact for convex zones
                fill.points += points([polygon[0], a, b], 0.09)
            label = base_marker("zone_label", index, Marker.TEXT_VIEW_FACING, 2.0, color("block"))
            label.pose.position.x = sum(p[0] for p in polygon) / len(polygon)
            label.pose.position.y = sum(p[1] for p in polygon) / len(polygon)
            label.pose.position.z = 1.0
            label.text = str(zone.get("name") or f"zone {index + 1}")
            array.markers += [outline, fill, label]
        clear = Marker(action=Marker.DELETEALL)
        self.hdmap_pub.publish(MarkerArray(markers=[clear] + array.markers))

    def on_ego(self, msg):
        yaw = math.radians(msg.heading)
        stamp = rospy.Time.now()
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id, transform.child_frame_id = stamp, "map", "competition_ego"
        transform.transform.translation.x, transform.transform.translation.y = msg.position.x, msg.position.y
        transform.transform.rotation.z, transform.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf.sendTransform(transform)
        car = base_marker("ego", 0, Marker.CUBE, 1.0, color("ego"))
        car.header.stamp = stamp
        car.pose.position.x, car.pose.position.y, car.pose.position.z = msg.position.x, msg.position.y, 0.8
        car.pose.orientation.z, car.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        car.scale.x, car.scale.y, car.scale.z = 4.635, 1.892, 1.6
        self.ego_pub.publish(car)

    def on_objects(self, msg):
        markers, count = [], 0
        for kind, items in (("npc", msg.npc_list), ("pedestrian", msg.pedestrian_list), ("obstacle", msg.obstacle_list)):
            for obj in items:
                yaw = math.radians(obj.heading)
                box = base_marker("objects", count, Marker.CUBE, 1.0, color(kind))
                box.pose.position.x, box.pose.position.y = obj.position.x, obj.position.y
                box.pose.position.z = max(obj.size.z, 0.5) / 2
                box.pose.orientation.z, box.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                box.scale.x, box.scale.y, box.scale.z = (max(obj.size.x, 0.4), max(obj.size.y, 0.4), max(obj.size.z, 0.5))
                box.lifetime = rospy.Duration(0.5)
                markers.append(box)
                count += 1
                speed = math.hypot(obj.velocity.x, obj.velocity.y)
                if speed > 0.3:
                    # MORAI object velocity is in the object's own frame; draw it along its heading.
                    arrow = base_marker("objects", count, Marker.ARROW, 1.0, color(kind))
                    arrow.points = points([(obj.position.x, obj.position.y),
                                           (obj.position.x + math.cos(yaw) * speed * 2.0,
                                            obj.position.y + math.sin(yaw) * speed * 2.0)], 1.0)
                    arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.25, 0.6, 0.6
                    arrow.lifetime = rospy.Duration(0.5)
                    markers.append(arrow)
                    count += 1
        self.objects_pub.publish(MarkerArray(markers=markers))


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--links", default=os.path.join(root, "autodrive/student_v3_claude/link_set.json"))
    parser.add_argument("--route", default=os.path.join(root, "autodrive/student_v3_claude/global_path.json"))
    parser.add_argument("--rules", default=os.path.join(root, "data/autodrive/drivable_rules.json"))
    parser.add_argument("--ego-topic", default="/localization/ego_status")
    parser.add_argument("--object-topic", default="/Object_topic")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("competition_viz")
    CompetitionViz(args)
    rospy.spin()


if __name__ == "__main__":
    main()
