#!/usr/bin/env python3
"""Install the single-PC and two-PC competition presets, archiving old files."""
import argparse
import os
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "autodrive"))
import competition_profiles as profiles


def archive_unselected(directory, keep, archive):
    """Move obsolete presets outside MORAI's Sensor/Network selection trees."""
    moved = 0
    for path in sorted(directory.iterdir()) if directory.exists() else ():
        if path.name not in keep and (path.is_file() or path.is_symlink()):
            archive.mkdir(parents=True, exist_ok=True)
            shutil.move(str(path), str(archive / path.name))
            moved += 1
    return moved


def install(savefile, *, prune=False):
    if not savefile.is_dir():
        raise ValueError("MORAI SaveFile 디렉터리 없음: " + str(savefile))
    profiles.materialize()
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-" + str(time.time_ns())
    archive = savefile / "PresetArchive" / stamp
    keep = {"Sensor": set(), "Network": set()}
    for name in ("single_pc", "two_pc"):
        p = profiles.profile(name)
        for key, kind in (("sensor_file", "Sensor"), ("network_file", "Network")):
            source = Path(p[key])
            destination = savefile / kind / "25.S4.MolitComp03" / source.name
            keep[kind].add(source.name)
            destination.parent.mkdir(parents=True, exist_ok=True)
            if not destination.exists() or destination.read_bytes() != source.read_bytes():
                if destination.exists():
                    old = archive / kind / destination.name
                    old.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(destination, old)
                temporary = destination.with_name("." + destination.name + ".tmp")
                shutil.copy2(source, temporary)
                os.replace(temporary, destination)
            print(destination)
    if prune:
        moved = sum(archive_unselected(savefile / kind / "25.S4.MolitComp03", names,
                                      archive / kind) for kind, names in keep.items())
        source_archive = ROOT / "morai/preset_archive" / stamp
        selected = {filename for files in profiles.PROFILE_FILES.values() for filename in files.values()}
        moved += archive_unselected(profiles.PRESETS, selected | {"final_sensor.zip"}, source_archive)
        print("Archived obsolete presets:", moved)
        if archive.exists():
            print("MORAI preset archive:", archive)
        if source_archive.exists():
            print("Source preset archive:", source_archive)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--savefile", type=Path,
                        default=Path(os.environ.get("MORAI_SAVEFILE_DIR", str(Path.home() / "MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile"))))
    parser.add_argument("--prune", action="store_true", help="archive other presets; leave two files per kind")
    args = parser.parse_args()
    install(args.savefile, prune=args.prune)
