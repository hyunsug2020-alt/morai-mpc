"""Apply and verify the single-PC sensor/UDP profile before an owned drive."""
import argparse
import json
from pathlib import Path
from websocket import WebSocketException

from runtime.morai_rpc import MoraiRPC

EGO = {"id": {"value": "Ego"}, "object_type": "OBJECT_TYPE_VEHICLE"}


def normalized_items(rpc, value):
    enum = rpc.pool.FindEnumTypeByName("morai_sim_api.actor.PUBSUB_TYPE")
    result = {}
    for row in value.get("items", []):
        key = row["protocol"]
        if isinstance(key, str):
            key = enum.values_by_name[key].number
        result[key] = {item["type"]: item["value"] for item in row.get("items", [])}
    return result


def network_matches(rpc, live, expected):
    if live.get("comm_type") != "NETWORK_COMM_TYPE_UDP":
        return False
    actual = normalized_items(rpc, live)
    wanted = normalized_items(rpc, expected["protocol_settings"])
    return all(all(actual.get(key, {}).get(field) == value for field, value in fields.items())
               for key, fields in wanted.items())


def configure(p, request, rpc, network_verified_on_host=False):
    if p["id"] != "single_pc" or p["simulator_ip"] != "127.0.0.1":
        raise ValueError("local simulator configuration accepts only single_pc")
    # Loading the sensor file preserves its full geometry, rates and fourth camera.
    rpc.call("sensor.Sensor", "LoadSensorFile", {"value": Path(p["sensor_file"]).name}, timeout=20)
    rpc.call("simulation.Simulation", "Resume", timeout=3)
    # Older ROS development presets leave a separate sensor-editor pause. Clear
    # it while their service still exists; UDP-only setups use Simulation.Resume.
    try:
        from runtime.morai_window import resume_sensor_pause
        resume_sensor_pause()
    except (OSError, WebSocketException, RuntimeError):
        # UDP-only setups have no sensor-editor pause service to query.
        pass
    if network_verified_on_host:
        # This MORAI build reports ROS from GetVehicleNetworkSetting and rejects SetVehicleNetwork,
        # even while the UDP preset loaded in its Network Settings window is live. The launcher
        # verified on the host that the simulator holds every requested UDP port.
        print("MORAI UDP network verified on host (simulator owns all requested ports)", flush=True)
    else:
        live = rpc.call("actor.Actor", "GetVehicleNetworkSetting", EGO, timeout=3)
        if not network_matches(rpc, live, request):
            rpc.call("actor.Actor", "SetVehicleNetwork", request, timeout=8)
            live = rpc.call("actor.Actor", "GetVehicleNetworkSetting", EGO, timeout=3)
        if not network_matches(rpc, live, request):
            raise RuntimeError("선택한 UDP 네트워크의 IP/포트 적용 확인 실패; 주행 시작 안 함")
    infos = rpc.call("sensor.Sensor", "GetSensorInfos", EGO, timeout=3)["values"]
    ids = {int(item["sensor_id"]["value"]) for item in infos}
    if ids != {1, 2, 3, 4, 5, 6, 7}:
        raise RuntimeError("final_sensor 센서 ID 확인 실패: " + str(sorted(ids)))
    rpc.call("actor.Actor", "SetVehicleControlMode",
             {"actor_info": EGO, "mode": "VEHICLE_CONTROL_AUTO_MODE"}, timeout=3)
    rpc.call("simulation.Simulation", "Resume", timeout=3)
    print("MORAI single_pc: final_sensor sensors + localhost UDP network verified", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile-file", type=Path, required=True)
    parser.add_argument("--request-file", type=Path, required=True)
    parser.add_argument("--network-verified-on-host", action="store_true")
    args = parser.parse_args()
    configure(json.loads(args.profile_file.read_bytes()), json.loads(args.request_file.read_bytes()), MoraiRPC(),
              args.network_verified_on_host)
