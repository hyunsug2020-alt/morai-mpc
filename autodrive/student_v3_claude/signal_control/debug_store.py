"""Bounded, atomic HSV snapshots shared by the worker and standard-library web.

Each archive contains one frame's images and metadata. Readers verify the run,
frame identity and camera receipt time before exposing it as live input.
"""
import json
import math
import os
import time
import zipfile
from pathlib import Path
from .camera import CAMERA_ID, CAMERA_VIEW

VIEWS = ("original", "semantic", "crop", "red", "yellow", "green")
SLOTS = 8
MAX_JSON_BYTES = 65536
MAX_IMAGE_BYTES = 2 * 1024 * 1024


def debug_directory():
    return Path(os.environ.get("SEMANTIC_FEED_DIR", "/dev/shm/alpamayo_semantic")) / "signal_debug"


class SignalDebugStore:
    def __init__(self, directory=None):
        self.directory = Path(directory) if directory is not None else debug_directory()
        self.sequence = 0

    def publish(self, report, config, images):
        if set(images) != set(VIEWS) or any(not isinstance(data, bytes) or
                not 0 < len(data) <= MAX_IMAGE_BYTES for data in images.values()):
            raise ValueError("signal debug snapshot requires six encoded images")
        self.sequence += 1
        slot = self.sequence % SLOTS
        metadata = dict(report, config=config, slot=slot, image_views=list(VIEWS),
            frame_id="%s:%s:%s" % (report["run_id"], report["camera_stamp_ns"], self.sequence))
        encoded = json.dumps(metadata, ensure_ascii=False).encode()
        if len(encoded) > MAX_JSON_BYTES:
            raise ValueError("signal debug metadata is too large")
        self.directory.mkdir(parents=True, exist_ok=True)
        target = self.directory / ("frame-%d.zip" % slot)
        temporary = target.with_suffix(".zip.tmp")
        with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_STORED) as archive:
            archive.writestr("metadata.json", encoded)
            for view, data in images.items():
                archive.writestr(view + ".jpg", data)
        os.replace(temporary, target)
        manifest = self.directory / "latest.json"
        temporary = manifest.with_suffix(".json.tmp")
        temporary.write_bytes(encoded)
        os.replace(temporary, manifest)
        return metadata

    @staticmethod
    def fresh(metadata, run_id, now):
        try:
            age = float((metadata.get("config") or {}).get("stale_seconds", 1.))
            return bool(isinstance(metadata, dict) and metadata.get("run_id") == run_id and run_id and
                metadata.get("fresh") is True and metadata.get("source") == "live" and
                metadata.get("camera_id") == CAMERA_ID and metadata.get("camera_view") == CAMERA_VIEW and
                math.isfinite(age) and .2 <= age <= 5. and
                0. <= now-float(metadata["frame_wall"]) <= age and
                0. <= now-float(metadata["time"]) <= age)
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
            return False

    def latest(self, run_id, now=None):
        now = time.time() if now is None else now
        try:
            with (self.directory / "latest.json").open("rb") as handle:
                encoded = handle.read(MAX_JSON_BYTES+1)
            if len(encoded) > MAX_JSON_BYTES:
                return None
            metadata = json.loads(encoded)
            if (self.fresh(metadata, run_id, now) and type(metadata.get("slot")) is int
                    and 0 <= metadata["slot"] < SLOTS and isinstance(metadata.get("frame_id"), str)
                    and metadata.get("image_views") == list(VIEWS)):
                return metadata
        except (OSError, ValueError, TypeError):
            pass
        return None

    def image(self, run_id, frame_id, slot, view, now=None, line_id=None):
        if type(slot) is not int or not 0 <= slot < SLOTS or view not in VIEWS:
            return None
        now = time.time() if now is None else now
        try:
            with zipfile.ZipFile(self.directory / ("frame-%d.zip" % slot)) as archive:
                if archive.getinfo("metadata.json").file_size > MAX_JSON_BYTES:
                    return None
                metadata = json.loads(archive.read("metadata.json"))
                if (not self.fresh(metadata, run_id, now) or metadata.get("frame_id") != frame_id
                        or metadata.get("slot") != slot
                        or (line_id is not None and metadata.get("line_id") != line_id)):
                    return None
                info = archive.getinfo(view + ".jpg")
                if not 0 < info.file_size <= MAX_IMAGE_BYTES:
                    return None
                return archive.read(info)
        except (OSError, ValueError, KeyError, zipfile.BadZipFile, EOFError, RuntimeError):
            return None
