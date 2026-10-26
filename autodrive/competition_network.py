"""Restore the local MORAI UDP preset and verify simulator-owned sockets."""
import json
import os
import re
import subprocess
import time

import competition_profiles


def missing_udp_ports():
    request = competition_profiles.network_request("single_pc")
    wanted = {f"{v['NETWORK_ITEM_TYPE_HOST_IP']}:{v['NETWORK_ITEM_TYPE_HOST_PORT']}"
              for v in ({x["type"]: x["value"] for x in item["items"]}
                        for item in request["protocol_settings"]["items"])}
    output = subprocess.run(["ss", "-Hlunp"], capture_output=True, text=True, check=True).stdout
    owned = {line.split()[3] for line in output.splitlines() if "Simulator.x86" in line}
    return sorted(wanted - owned)


def canvas_point(info, dx, dy):
    """Map centered 1920x1080 Unity canvas coordinates into the client area."""
    def field(name):
        match = re.search(r"^\s*" + re.escape(name) + r":\s*(-?\d+)\s*$", info, re.MULTILINE)
        if not match:
            raise RuntimeError("MORAI window geometry missing: " + name)
        return int(match.group(1))

    x, y = field("Absolute upper-left X"), field("Absolute upper-left Y")
    width, height = field("Width"), field("Height")
    if width <= 0 or height <= 0:
        raise RuntimeError("MORAI window has no client area")
    scale = min(width / 1920, height / 1080)
    return round(x + width / 2 + dx * scale), round(y + height / 2 + dy * scale)


def load_network_gui():
    # MORAI's gRPC SetVehicleNetwork rejects this preset after a ROS scenario
    # load. Use its preset dialog; socket ownership remains the success check.
    env = dict(os.environ, DISPLAY=os.environ.get("DISPLAY", ":0"))

    def run(args):
        # Let a short held-input sequence finish its key/button release even
        # when the dashboard interrupts the launch process group.
        held_input = "keydown" in args or "mousedown" in args
        return subprocess.run(args, capture_output=True, text=True, env=env, check=True,
                              start_new_session=held_input).stdout

    windows = run(["xdotool", "search", "--name", "^Simulator$"]).split()
    if len(windows) != 1:
        raise RuntimeError("MORAI 창을 하나로 식별할 수 없습니다.")
    window = windows[0]

    def click(dx, dy, pause=1.5):
        run(["xdotool", "windowactivate", "--sync", window])
        x, y = canvas_point(run(["xwininfo", "-id", window]), dx, dy)
        # Unity polls button state per frame; a 12 ms synthetic click can vanish
        # between frames under simulator load.
        run(["xdotool", "mousemove", str(x), str(y), "mousedown", "1",
             "sleep", "0.15", "mouseup", "1"])
        time.sleep(pause)

    run(["xdotool", "windowactivate", "--sync", window])
    time.sleep(.5)
    run(["xdotool", "keydown", "F4", "sleep", "0.2", "keyup", "F4"])
    time.sleep(1.5)
    click(-208, 273)                       # Network Settings: Load
    click(-440, -230, .3)                  # File manager search
    run(["xdotool", "keydown", "ctrl", "keydown", "a", "sleep", "0.15", "keyup", "a", "keyup", "ctrl"])
    run(["xdotool", "type", "--clearmodifiers", "--delay", "80",
         "2026_molit_comp_network_single_pc"])
    time.sleep(1)
    click(-440, -147)                      # First filtered preset
    click(-340, 273)                       # File manager: Load
    click(0, 130)                          # Loaded notice: OK
    click(-188, -284)                      # Close file manager; network recenters
    if missing_udp_ports():
        click(285, 273, 3.)                # Connect only while disconnected
    click(383, -284)                       # Close Network Settings


def ensure_network():
    missing = missing_udp_ports()
    if not missing:
        return
    print(json.dumps({"msg": "MORAI UDP network missing - loading single-PC preset",
                      "missing": missing}), flush=True)
    load_network_gui()
    time.sleep(2)
    missing = missing_udp_ports()
    if missing:
        raise RuntimeError("MORAI 네트워크 로드 실패: " + " ".join(missing))


if __name__ == "__main__":
    ensure_network()
