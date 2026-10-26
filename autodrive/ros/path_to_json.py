#!/usr/bin/env python3
"""Convert an 'x y' per-line path text file into the {"waypoints":[{"x","y"}]} JSON moraimpc loads."""
import json
import sys


def main(src, dst):
    wps = []
    with open(src) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) >= 2:
                wps.append({"x": float(parts[0]), "y": float(parts[1])})
    if len(wps) < 2:
        sys.exit(f"{src}: fewer than 2 waypoints")
    with open(dst, "w") as f:
        json.dump({"waypoints": wps, "source": src}, f)
    print(f"{dst}: {len(wps)} waypoints")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
