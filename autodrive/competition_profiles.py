"""Authoritative one-PC and two-PC sensor files and competition UDP profiles."""
import argparse
import copy
import hashlib
import json
import shlex
import socket
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PRESETS = ROOT / "morai/final_sensor"
SOURCE_FILES = {
    "bisa_real_sensor_single.json": "619cb7e6609c6af8e0d3d443c990c9a2310364e65e26c7872801d0ce284b9130",
    "bisa_real_sensor_two_pc.json": "21025156a73034822025459a08b0485775c26c18401814228245b191f10209d5",
    "2026_molit_comp_network.json": "d25f643f628e1ade4243aae13328deb14b7a63c962241321e3e35b487580a415",
}
DEFAULT_PROFILE = "two_pc"
PROFILE_NAMES = {"single_pc": "싱글 PC용", "two_pc": "대회용 · PC 2대"}
PROFILE_FILES = {
    "single_pc": {"sensor_file": "bisa_real_sensor_single.json",
                  "network_file": "2026_molit_comp_network_single_pc.json"},
    "two_pc": {"sensor_file": "bisa_real_sensor_two_pc.json",
               "network_file": "2026_molit_comp_network_two_pc.json"},
}
VIEW_IDS = {"front": 1, "left": 3, "right": 2}
LIDAR_HZ = 15.0


def original_bytes(name):
    """Verify registered inputs without rewriting MORAI's saved sensor values."""
    data = (PRESETS / "originals" / name).read_bytes()
    if hashlib.sha256(data).hexdigest() != SOURCE_FILES[name]:
        raise ValueError("센서·네트워크 원본 변경 감지: " + name)
    return data


def originals(name="two_pc"):
    return [json.loads(original_bytes(filename)) for filename in
            (PROFILE_FILES[name]["sensor_file"], "2026_molit_comp_network.json")]


def loopback(value):
    value = copy.deepcopy(value)

    def visit(item):
        if isinstance(item, dict):
            for key in ("hostIP", "destinationIP"):
                if item.get(key) in ("192.168.0.1", "192.168.0.10"):
                    item[key] = "127.0.0.1"
            # Windows simulator PCs may use ports below 1024 (Ego status 910/911); MORAI on
            # Linux runs unprivileged and then fails SetVehicleNetwork, so shift them up.
            for key in ("hostPort", "destinationPort"):
                if isinstance(item.get(key), int) and 0 < item[key] < 1024:
                    item[key] += 9000
            for child in item.values():
                visit(child)
        elif isinstance(item, list):
            for child in item:
                visit(child)

    visit(value)
    return value


def receiver_ports(sensor):
    ports = set()

    def visit(item):
        if isinstance(item, dict):
            if isinstance(item.get("destinationPort"), int):
                ports.add(item["destinationPort"])
            for child in item.values():
                visit(child)
        elif isinstance(item, list):
            for child in item:
                visit(child)

    visit(sensor)
    return ports


def single_pc_network(network, sensor):
    """On one PC every socket shares 127.0.0.1: a simulator-side host port equal to a
    sensor receive port (CollisionData 9091 vs IMU 9091) steals our receiver's port."""
    network = loopback(network)
    taken = receiver_ports(loopback(sensor))

    def visit(item):
        if isinstance(item, dict):
            if isinstance(item.get("hostPort"), int) and item["hostPort"] in taken:
                item["hostPort"] += 1000
            for child in item.values():
                visit(child)
        elif isinstance(item, list):
            for child in item:
                visit(child)

    visit(network)
    return network


def materialize():
    payloads = []
    for name, files in PROFILE_FILES.items():
        sensor, network = originals(name)
        if name == "single_pc":
            network = single_pc_network(network, sensor)
        payloads.extend(((files["sensor_file"], original_bytes(files["sensor_file"])),
                         (files["network_file"],
                          (json.dumps(network, ensure_ascii=False, indent=2) + "\n").encode())))
    for filename, data in payloads:
        path = PRESETS / filename
        if not path.exists() or path.read_bytes() != data:
            temporary = path.with_suffix(".tmp")
            temporary.write_bytes(data)
            temporary.replace(path)


def profile(name):
    if name not in PROFILE_NAMES:
        raise ValueError("주행 구성은 single_pc 또는 two_pc 지정 필요")
    sensor, network = originals(name)
    if name == "single_pc":
        network = single_pc_network(network, sensor)
    sensor_path = PRESETS / PROFILE_FILES[name]["sensor_file"]
    network_path = PRESETS / PROFILE_FILES[name]["network_file"]
    # Never silently accept a stale generated profile or an environment override.
    if (sensor_path.read_bytes() != original_bytes(PROFILE_FILES[name]["sensor_file"]) or
            json.loads(network_path.read_bytes()) != network):
        raise ValueError("선택한 센서·네트워크 설정이 등록된 원본 기준과 다름")
    cameras = {int(c["m_SensorUniqueID"]): c for c in sensor["cameraList"]}
    ego = next(e for e in network["egoNetworkData"] if e["UNIQUEID"] == 0)
    rows = {r["PUBSUB_TYPE"]: r for r in ego["listEgoNetworkInfo"] if r["netType"] == 1}
    control = rows[771]["EgoCtrlConfig"]["udpConfig"]
    return {
        "id": name, "label": PROFILE_NAMES[name], "transport": "udp",
        "sensor_file": str(sensor_path), "network_file": str(network_path),
        "sensor_sha256": hashlib.sha256(sensor_path.read_bytes()).hexdigest(),
        "network_sha256": hashlib.sha256(network_path.read_bytes()).hexdigest(),
        "simulator_ip": control["hostIP"], "local_ip": control["destinationIP"],
        "control_port": int(control["hostPort"]),
        "control_bind_port": int(control["destinationPort"]),
        "status_port": int(rows[295]["config"]["udpConfig"]["destinationPort"]),
        # 261 is the official CollisionData protocol; 787 is a separate publisher.
        "collision_port": int(rows[261]["config"]["udpConfig"]["destinationPort"]),
        "gps_port": int(sensor["GPSList"][0]["gc"]["udpConfig"]["destinationPort"]),
        "imu_port": int(sensor["IMUList"][0]["ic"]["udpConfig"]["destinationPort"]),
        "lidar_port": int(sensor["Lidar3DList"][0]["lc"]["udpConfig"]["destinationPort"]),
        "lidar_hz": LIDAR_HZ, "lidar_rpm": int(LIDAR_HZ*60),
        "lidar_mount": [float(sensor["Lidar3DList"][0]["pos"][k]) for k in ("x", "y", "z")],
        "cameras": {view: {"camera_id": uid,
                           "udp_port": int(cameras[uid]["cc"]["udpConfig"]["destinationPort"])}
                    for view, uid in VIEW_IDS.items()},
        "aux_camera": {"camera_id": 4,
                       "udp_port": int(cameras[4]["cc"]["udpConfig"]["destinationPort"])},
    }


def network_request(name):
    p = profile(name)
    network = json.loads(Path(p["network_file"]).read_bytes())
    ego = next(e for e in network["egoNetworkData"] if e["UNIQUEID"] == 0)
    items = []
    fields = {"hostIP": "HOST_IP", "hostPort": "HOST_PORT",
              "destinationIP": "DESTINATION_IP", "destinationPort": "DESTINATION_PORT"}
    for row in ego["listEgoNetworkInfo"]:
        if row["netType"] != 1:
            continue
        config = row["EgoCtrlConfig"] if row["Division_TYPE"] == 3 else row["config"]
        udp = config["udpConfig"]
        items.append({"protocol": row["PUBSUB_TYPE"], "items": [
            {"type": "NETWORK_ITEM_TYPE_" + kind, "value": str(udp[key])}
            for key, kind in fields.items()]})
    return {"actor_info": {"id": {"value": "Ego"}, "object_type": "OBJECT_TYPE_VEHICLE"},
            "enabled": True, "comm_type": "NETWORK_COMM_TYPE_UDP", "frame_rate": 50,
            "protocol_settings": {"comm_type": "NETWORK_COMM_TYPE_UDP", "items": items},
            "ip_settings": [{"connect_type": "NETWORK_CONNECT_TYPE_PUBSUB", "items": [
                {"type": "NETWORK_ITEM_TYPE_HOST_IP", "value": p["simulator_ip"]},
                {"type": "NETWORK_ITEM_TYPE_DESTINATION_IP", "value": p["local_ip"]}]}]}


def check_local_ip(p):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind((p["local_ip"], 0))
        except OSError as error:
            raise ValueError(p["label"] + ": 주행 PC 유선 IP " + p["local_ip"] +
                             " 필요. 현재 PC에 해당 IP가 없어 시작하지 않음") from error


def shell(p):
    values = {"STUDENT_NETWORK_PROFILE": p["id"], "SENSOR_PRESET": p["sensor_file"],
              "STUDENT_SENSOR_PRESET": p["sensor_file"], "GPS_PORT": p["gps_port"],
              "IMU_PORT": p["imu_port"], "MORAI_SIM_IP": p["simulator_ip"],
              "MORAI_LOCAL_IP": p["local_ip"], "MORAI_STATUS_PORT": p["status_port"],
              "MORAI_COLLISION_PORT": p["collision_port"], "MORAI_CONTROL_PORT": p["control_port"],
              "MORAI_CONTROL_BIND_PORT": p["control_bind_port"],
              "MORAI_LIDAR_RPM": p["lidar_rpm"],
              "SIGNAL_CAMERA_PORT": p["aux_camera"]["udp_port"],
              "STUDENT_LIDAR_MOUNT": ",".join(map(str, p["lidar_mount"])),
              "CAMERA_PORT_PATTERN": "|".join(str(c["udp_port"]) for c in p["cameras"].values())}
    return "\n".join("export " + key + "=" + shlex.quote(str(value)) for key, value in values.items())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("generate", "shell", "check", "json", "request"))
    parser.add_argument("profile", nargs="?", choices=tuple(PROFILE_NAMES))
    args = parser.parse_args()
    if args.operation == "generate":
        materialize()
        return
    p = profile(args.profile)
    if args.operation == "shell":
        print(shell(p))
    elif args.operation == "check":
        check_local_ip(p)
    else:
        print(json.dumps(network_request(args.profile) if args.operation == "request" else p,
                         ensure_ascii=False, indent=2))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, StopIteration) as error:
        raise SystemExit(str(error))
