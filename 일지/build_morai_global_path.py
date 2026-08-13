#!/usr/bin/env python3
"""hdmap_viz.html의 선택 노드를 MORAI 경로 추종기용 JSON으로 변환함."""

import argparse
import json
import math
import re
import zipfile
from collections import defaultdict
from pathlib import Path


DEFAULT_HTML = Path("/home/bisa/Downloads/hdmap_viz.html")
DEFAULT_HDMAP = Path(
    "/home/bisa/morai-mpc-agent-morai-lio-gps-integration/"
    "src/moraimpc/data/hdmap.zip"
)
DEFAULT_OUTPUT = Path("/home/bisa/Downloads/morai_global_path.json")


def path_length(points):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )


def load_selection(html_path):
    text = html_path.read_text(encoding="utf-8")
    nodes_start = text.index("const NODES=") + len("const NODES=")
    nodes_end = text.index(", LINKS=", nodes_start)
    nodes = json.loads(text[nodes_start:nodes_end])

    match = re.search(r"const sel=\[([^]]*)\]", text)
    if not match:
        raise RuntimeError("HTML에서 선택 노드 배열(const sel)을 찾지 못했음")
    selection = [int(value) for value in match.group(1).split(",") if value.strip()]
    if len(selection) < 2:
        raise RuntimeError("경로 생성에는 선택 노드가 2개 이상 필요함")

    node_by_number = {node["n"]: node for node in nodes}
    missing = [number for number in selection if number not in node_by_number]
    if missing:
        raise RuntimeError(f"존재하지 않는 글로벌 노드 번호: {missing}")
    return selection, node_by_number


def load_links(hdmap_path):
    with zipfile.ZipFile(hdmap_path) as archive:
        links = json.loads(archive.read("link_set.json"))
    links_by_nodes = defaultdict(list)
    for link in links:
        key = (link["from_node_idx"], link["to_node_idx"])
        links_by_nodes[key].append(link)
    return links_by_nodes


def build_route(selection, node_by_number, links_by_nodes):
    route = []
    link_ids = []
    for from_number, to_number in zip(selection, selection[1:]):
        from_node = node_by_number[from_number]
        to_node = node_by_number[to_number]
        key = (from_node["idx"], to_node["idx"])
        candidates = links_by_nodes.get(key, [])
        if not candidates:
            raise RuntimeError(
                f"정방향 HD맵 링크가 없음: {from_number}({key[0]}) -> "
                f"{to_number}({key[1]})"
            )
        link = min(candidates, key=lambda item: path_length(item["points"]))
        link_ids.append(link["idx"])
        for point in link["points"]:
            xy = [float(point[0]), float(point[1])]
            if route and math.hypot(xy[0] - route[-1][0], xy[1] - route[-1][1]) < 1e-6:
                continue
            route.append(xy)
    return route, link_ids


def make_waypoints(route):
    waypoints = []
    for index, point in enumerate(route):
        if index == 0:
            before, after = route[0], route[1]
        elif index == len(route) - 1:
            before, after = route[-2], route[-1]
        else:
            before, after = route[index - 1], route[index + 1]
        heading = math.atan2(after[1] - before[1], after[0] - before[0])
        waypoints.append(
            {"x": point[0], "y": point[1], "heading": heading, "gear": "D"}
        )
    return waypoints


def route_quality(route):
    gaps = [
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(route, route[1:])
    ]
    turns = []
    for a, b, c in zip(route, route[1:], route[2:]):
        h1 = math.atan2(b[1] - a[1], b[0] - a[0])
        h2 = math.atan2(c[1] - b[1], c[0] - b[0])
        delta = (h2 - h1 + math.pi) % (2.0 * math.pi) - math.pi
        turns.append(abs(delta))
    return sum(gaps), max(gaps, default=0.0), math.degrees(max(turns, default=0.0))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--html", type=Path, default=DEFAULT_HTML)
    parser.add_argument("--hdmap", type=Path, default=DEFAULT_HDMAP)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    selection, node_by_number = load_selection(args.html)
    links_by_nodes = load_links(args.hdmap)
    route, link_ids = build_route(selection, node_by_number, links_by_nodes)
    waypoints = make_waypoints(route)
    total_length, max_gap, max_turn = route_quality(route)

    if max_gap > 1.0:
        raise RuntimeError(f"경로 점 간격이 너무 큼: {max_gap:.3f}m")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps({"waypoints": waypoints}, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"출력: {args.output}")
    print(f"글로벌 노드: {len(selection)}개 / HD맵 링크: {len(link_ids)}개")
    print(f"웨이포인트: {len(waypoints)}개 / 경로 길이: {total_length:.1f}m")
    print(f"최대 점 간격: {max_gap:.3f}m / 최대 국소 회전: {max_turn:.2f}도")
    print(f"시작: ({route[0][0]:.3f}, {route[0][1]:.3f})")
    print(f"종료: ({route[-1][0]:.3f}, {route[-1][1]:.3f})")


if __name__ == "__main__":
    main()
