#!/usr/bin/env python3
"""Subscriber-only driving diagnostics recorder (ROS Noetic, Python 3) - hardened lifecycle.

Every reception on the diagnostic topics becomes
  * the original serialized message, written to size/duration-split bags under the
    publisher's actual connection header (callerid, type, md5sum, message_definition,
    latching; a changed header opens a new bag connection), followed by a bag-local
    metadata record (never advertised on ROS), and
  * one CSV row (no fusion or interpolation).

Admission (ROS receive threads): under one admission lock the recorder stamps the
wall / monotonic / ROS clocks, assigns event_id (= admission order), the per-topic
revision and the planner-path association (latest *admitted* /avoid_waypoints event, so
never a later reception), counts, and put_nowait()s into a bounded queue. Nothing in
admission blocks on I/O. Closing admission takes the same lock, so every callback either
completed admission before the close or is counted as rejected_after_close.

Single writer: one thread owns every bag/CSV write, rotation and the final
close/fsync/hash/snapshot. The main thread only monitors the writer, persists
capture.json and drives the lifecycle. If the writer does not finish within
--drain-timeout the run is abandoned: main never touches bag/CSV, reports unresolved
events and the open shard, and exits 5.

Accounting per topic (capture.json "topics"):
  received = admitted + rejected_after_close
  admitted = persisted_full + persisted_csv_only + persisted_bag_only + failed_none
             + dropped_queue + dropped_capacity + unresolved
decode_failed is orthogonal (raw payload identity is persisted regardless).

Startup (every failure before "ready" leaves the previous latest CSV byte-identical):
  unique capture dir -> master check (RPC timeout) -> flock <latest>.lock ->
  staging CSV beside latest -> first shard -> writer thread -> init node + subscribe ->
  registration verified by timed master RPCs -> hardlink backup of previous latest ->
  atomic rename staging -> latest -> capture.json(recording) -> COMMIT: rename of the
  "ready" marker (run id, pid, pid starttime, boot id, master URI, committed=true). Any
  failure before the commit renames the backup back (previous latest byte-identical) and
  removes nothing an observer could have trusted: "ready" never existed. After the commit
  nothing rolls back; backup cleanup / directory fsync problems become capture.json warnings.
  The internal watchdog bounds most of startup but not blocking kernel I/O; callers must
  impose their own outer deadline on the process and treat the process exit as authoritative
  (after exit 6 the persisted capture.json may still say "recording").

Clocks: every bag record uses the reception wall clock as its bag time (one policy for
the whole run); original wall, monotonic and ROS (sim) clocks are kept in CSV and meta.
Bag order is therefore wall-clock order, not proof of causal order; use event_id.

Semantics: /ctrl_cmd rows are the exact ROS publication received here, not the command
applied by the simulator (applied_command_ack = unavailable); ROS field presence says
nothing about simulator wire JSON presence (unknown); active_authority_verified = false;
manual override is never inferred; services are not recorded.

Exit codes: 0 complete, 4 incomplete, 5 drain timeout / abandoned (unresolved events),
6 terminal capture.json could not be saved, 2 startup failure, 3 lock held / run id reused.
"""
import argparse
import csv
import errno
import fcntl
import hashlib
import http.client
import itertools  # noqa: F401  (kept for API compatibility with the first candidate)
import json
import os
import queue
import re
import shutil
import signal
import socket
import sys
import threading
import time
import xmlrpc.client

DEFAULT_TOPICS = [
    "/ctrl_cmd", "/Ego_topic", "/Competition_topic", "/Object_topic", "/CollisionData",
    "/avoid_waypoints", "/avoid_path", "/avoid_target_vel", "/lane_change_status",
    "/lane_change_mode", "/avoidance/diagnostics", "/planning/global_path",
    "/traffic_stop_status", "/mpc_status", "/mpc_performance", "/InsnControl",
    "/competition/gear_cmd", "/rosout_agg",
]
META_TOPIC = "/_drive_diag_bag_local/event_meta"   # bag-local only, never advertised
PLANNER_TEXT_TOPICS = {"/lane_change_status", "/avoidance/diagnostics", "/lane_change_mode",
                       "/traffic_stop_status", "/mpc_status"}
ASSOC_TOPICS = PLANNER_TEXT_TOPICS | {"/avoid_target_vel"}
PATH_JSON_TOPICS = {"/avoid_waypoints"}
CMD_FIELDS = ["longlCmdType", "steering", "velocity", "accel", "brake", "acceleration"]
EGO_SCALARS = ["heading", "accel", "brake", "wheel_angle", "front_steer", "rear_steer", "gear", "ctrl_mode"]
EGO_VECTORS = ["position", "velocity", "angular_velocity", "acceleration"]
BAG_TIME_POLICY = "recv_wall_ns for every record"
TERMINAL = ["persisted_full", "persisted_csv_only", "persisted_bag_only", "failed_none", "dropped_queue", "dropped_capacity"]

CSV_COLUMNS = (
    ["run_id", "event_id", "topic", "msg_type", "md5sum", "callerid", "latching",
     "recv_wall_ns", "recv_monotonic_ns", "recv_ros_ns", "payload_bytes", "payload_sha256",
     "topic_revision", "decode_status", "has_header", "header_seq", "header_stamp_ns", "header_frame_id"]
    + ["cmd_" + f for f in CMD_FIELDS]
    + ["cmd_semantics", "applied_command_ack", "wire_field_presence"]
    + ["ego_%s_%s" % (v, a) for v in EGO_VECTORS for a in "xyz"]
    + ["ego_" + f for f in EGO_SCALARS]
    + ["ego_ros_fields_absent",
       "text_data", "text_in_bag_only", "text_json_ok", "refresh_fields_json", "path_meta_json",
       "path_n_points", "float_data", "int_data", "array_data",
       "objects_npc", "objects_pedestrian", "objects_obstacle",
       "assoc_latest_avoid_waypoints_event_id", "assoc_latest_avoid_waypoints_sha256",
       "small_msg_json", "decode_error"]
)


def now_ns():
    return time.time_ns(), time.monotonic_ns()


def free_bytes(path):
    """Free bytes available on the filesystem holding path (patchable in tests)."""
    st = os.statvfs(path)
    return st.f_bavail * st.f_frsize


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def atomic_write(path, text):
    """Write text via tmp + fsync + rename + dir fsync. Raises on any failure."""
    tmp = "%s.tmp.%d.%d" % (path, os.getpid(), threading.get_ident())
    try:
        with open(tmp, "w") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            try:
                os.unlink(tmp)
            except OSError:
                pass
    fsync_dir(os.path.dirname(path))


def proc_starttime(pid):
    """Start time in clock ticks since boot (field 22 of /proc/<pid>/stat)."""
    with open("/proc/%d/stat" % pid) as f:
        return int(f.read().rsplit(")", 1)[1].split()[19])


def boot_id():
    try:
        return open("/proc/sys/kernel/random/boot_id").read().strip()
    except OSError:
        return None


_POST_PUBLISH = {}


def publish_ready(path, obj):
    """tmp + fsync, then rename = publication/commit. Errors after the rename are returned as warnings."""
    tmp = "%s.tmp.%d" % (path, os.getpid())
    with open(tmp, "w") as f:
        f.write(json.dumps(obj, sort_keys=True))
        f.flush()
        os.fsync(f.fileno())
    os.rename(tmp, path)
    try:
        fsync_dir(os.path.dirname(path))
    except OSError as e:
        _POST_PUBLISH["warning"] = repr(e)


def fsync_dir(d):
    fd = os.open(d or ".", os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def fnum(v):
    """Round-trip-safe text for numeric ROS fields."""
    if isinstance(v, bool):
        return str(int(v))
    if isinstance(v, float):
        return repr(v)
    return str(v)


def msg_to_obj(m, depth=0):
    if depth > 8:
        return "<depth>"
    if hasattr(m, "__slots__") and hasattr(m, "_slot_types"):
        return {s: msg_to_obj(getattr(m, s), depth + 1) for s in m.__slots__}
    if isinstance(m, (list, tuple)):
        return [msg_to_obj(x, depth + 1) for x in m]
    if isinstance(m, bytes):
        return {"bytes_hex": m.hex()}
    if hasattr(m, "secs") and hasattr(m, "nsecs"):
        return int(m.secs) * 1_000_000_000 + int(m.nsecs)
    if isinstance(m, float):
        return repr(m)
    return m


class _TimeoutTransport(xmlrpc.client.Transport):
    def __init__(self, timeout):
        super().__init__()
        self._timeout = timeout

    def make_connection(self, host):
        conn = super().make_connection(host)
        if isinstance(conn, http.client.HTTPConnection):
            conn.timeout = self._timeout
        return conn


def master_rpc(uri, timeout):
    """XML-RPC proxy with a per-connection timeout (no global socket default is changed)."""
    return xmlrpc.client.ServerProxy(uri, transport=_TimeoutTransport(timeout), allow_none=True)


class _RawType(object):
    """pytype for rosbag raw writes, built from the publisher's own connection header."""
    def __init__(self, md5sum, full_text):
        self._md5sum = md5sum
        self._full_text = full_text


class Reception(object):
    """Immutable reception identity, fixed at admission and independent of decoding."""
    __slots__ = ("event_id", "topic", "msg_type", "md5sum", "callerid", "latching", "header", "buff",
                 "payload_bytes", "payload_sha256", "recv_wall_ns", "recv_monotonic_ns", "recv_ros_ns",
                 "topic_revision", "assoc_path_event_id", "assoc_path_sha256", "bag_done", "csv_done")

    def ident(self):
        return {"event_id": self.event_id, "topic": self.topic, "msg_type": self.msg_type, "md5sum": self.md5sum,
                "callerid": self.callerid, "latching": self.latching, "payload_bytes": self.payload_bytes,
                "payload_sha256": self.payload_sha256, "recv_wall_ns": self.recv_wall_ns,
                "recv_monotonic_ns": self.recv_monotonic_ns, "recv_ros_ns": self.recv_ros_ns,
                "topic_revision": self.topic_revision}


class ShardError(Exception):
    pass


class ShardWriter(object):
    """Split bag writer; one bag connection per distinct (topic, full connection header)."""

    def __init__(self, directory, prefix, split_bytes, split_seconds):
        import rosbag
        self._rosbag = rosbag
        self.directory, self.prefix = directory, prefix
        self.split_bytes, self.split_seconds = split_bytes, split_seconds
        self.index = -1
        self.bag = None
        self.opened_mono = None
        self.conns = {}
        self.shards = []   # {"file","index","state","messages","bytes","sha256","error"}
        self.lock = threading.Lock()

    def snapshot(self):
        with self.lock:
            return [dict(x) for x in self.shards]

    def _set(self, **kw):
        with self.lock:
            self.shards[-1].update(kw)

    def open_next(self):
        self.index += 1
        name = "%s_%04d.bag" % (self.prefix, self.index)
        with self.lock:
            self.shards.append({"file": name, "index": self.index, "state": "opening", "messages": 0})
        self.bag = self._rosbag.Bag(os.path.join(self.directory, name), "w")
        self.opened_mono = time.monotonic()
        self.conns = {}
        self._set(state="open")

    def close(self, state="complete"):
        """Close + fsync + hash the current shard. Raises ShardError on any failure (file kept)."""
        if self.bag is None:
            return
        name = self.shards[-1]["file"]
        bag, self.bag = self.bag, None
        path = os.path.join(self.directory, name)
        try:
            bag.close()
        except Exception as e:
            self._set(state="close_failed", error=repr(e))
            raise ShardError("close %s: %r" % (name, e))
        try:
            fd = os.open(path, os.O_RDONLY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
            size, digest = os.path.getsize(path), sha256_file(path)
        except Exception as e:
            self._set(state="fsync_or_hash_failed", error=repr(e))
            raise ShardError("fsync/hash %s: %r" % (name, e))
        self._set(state=state, bytes=size, sha256=digest)

    def due_rotation(self):
        if self.bag is None:
            return True
        size = self.bag._file.tell() if self.bag._file else 0
        return bool((self.split_bytes and size >= self.split_bytes) or
                    (self.split_seconds and time.monotonic() - self.opened_mono >= self.split_seconds))

    def write_event(self, ev, meta_json):
        """Write the original raw message under its own connection header, then the meta record."""
        import genpy
        from std_msgs.msg import String
        t = genpy.Time(ev.recv_wall_ns // 1_000_000_000, ev.recv_wall_ns % 1_000_000_000)
        hdr = dict(ev.header)
        hdr["topic"] = ev.topic
        key = (ev.topic, tuple(sorted(hdr.items())))
        tc = self.bag._topic_connections
        if key in self.conns:
            tc[ev.topic] = self.conns[key]
        else:
            tc.pop(ev.topic, None)   # force a new connection record for a new/changed publisher header
        pytype = _RawType(hdr.get("md5sum", ""), hdr.get("message_definition", ""))
        self.bag.write(ev.topic, (hdr.get("type", ""), ev.buff, hdr.get("md5sum", ""), None, pytype), t,
                       raw=True, connection_header=hdr)
        self.conns[key] = tc[ev.topic]
        self.bag.write(META_TOPIC, String(data=meta_json), t)
        with self.lock:
            self.shards[-1]["messages"] += 1


class Recorder(object):
    def __init__(self, a):
        self.a = a
        self.lock = threading.Lock()        # counters / reasons / errors
        self.adm = threading.Lock()         # admission order
        self.admitting = False
        self.next_event_id = 1
        self.rev = {}
        self.latest_path_adm = (None, None)
        self.q = queue.Queue(maxsize=a.queue_size)
        self.counts = {t: dict({k: 0 for k in ["received", "rejected_after_close", "admitted"] + TERMINAL + ["decode_failed"]},
                               callerids={}, types={}) for t in a.topics}
        self.reasons, self.errors, self.error_overflow = [], [], 0
        self.state = "initializing"
        self.times = {}
        self.source_meta = {"files": [], "hashes": []}
        self.types = {}
        self.type_status = {}
        self.warnings = []
        self.shards = None
        self.csv_file = None
        self.csv_writer = None
        self.csv_rows = 0
        self.csv_path = None          # path currently naming the owned CSV (staging, then latest)
        self.bag_failed = False
        self.csv_failed = False
        self.capacity_exhausted = False
        self._last_cap = 0.0
        # writer lifecycle
        self.finalize_req = threading.Event()
        self.writer_exit = threading.Event()
        self.abandoned = False
        self.inflight = None
        self.writer_fatal = None
        self.final = {"snapshot": None}
        self.startup_failed = False
        self.status_ok = True
        self.last_status = 0.0
        self.unresolved = {}

    # ---------- bookkeeping (never persists anything, so no recursion under ENOSPC) ----------
    def reason(self, r):
        with self.lock:
            if r not in self.reasons:
                self.reasons.append(r)

    def error(self, where, exc):
        with self.lock:
            if len(self.errors) < 50:
                self.errors.append({"where": where, "error": repr(exc), "wall_ns": time.time_ns()})
            else:
                self.error_overflow += 1

    def count(self, topic, key, n=1):
        with self.lock:
            self.counts[topic][key] += n

    # ---------- admission (ROS receive threads) ----------
    def make_cb(self, topic):
        import rospy

        def cb(msg):
            buff = msg._buff
            hdr = {str(k): str(v) for k, v in (getattr(msg, "_connection_header", None) or {}).items()}
            sha = hashlib.sha256(buff).hexdigest()
            with self.adm:
                wall, mono = now_ns()
                try:
                    ros = rospy.get_rostime().to_nsec()
                except Exception:
                    ros = -1
                c = self.counts[topic]
                with self.lock:
                    c["received"] += 1
                    cid, ty = hdr.get("callerid", ""), hdr.get("type", "")
                    c["callerids"][cid] = c["callerids"].get(cid, 0) + 1
                    c["types"][ty] = c["types"].get(ty, 0) + 1
                    if not self.admitting:
                        c["rejected_after_close"] += 1
                        return
                    c["admitted"] += 1
                ev = Reception()
                ev.event_id = self.next_event_id
                self.next_event_id += 1
                self.rev[topic] = self.rev.get(topic, 0) + 1
                ev.topic, ev.header, ev.buff = topic, hdr, buff
                ev.msg_type, ev.md5sum = hdr.get("type", ""), hdr.get("md5sum", "")
                ev.callerid, ev.latching = hdr.get("callerid", ""), hdr.get("latching", "")
                ev.payload_bytes, ev.payload_sha256 = len(buff), sha
                ev.recv_wall_ns, ev.recv_monotonic_ns, ev.recv_ros_ns = wall, mono, ros
                ev.topic_revision = self.rev[topic]
                ev.assoc_path_event_id, ev.assoc_path_sha256 = (self.latest_path_adm if topic in ASSOC_TOPICS else (None, None))
                if topic in PATH_JSON_TOPICS:
                    self.latest_path_adm = (ev.event_id, sha)
                ev.bag_done = ev.csv_done = False
                try:
                    self.q.put_nowait(ev)
                except queue.Full:
                    self.count(topic, "dropped_queue")
                    self.reason("queue_overflow")
        return cb

    def close_admission(self):
        with self.adm:
            with self.lock:
                self.admitting = False
        self.times["admission_closed_wall_ns"] = time.time_ns()

    # ---------- decoding ----------
    def msg_class(self, hdr):
        """Class whose MD5 equals the declared MD5, or None with a reason in self.type_status.

        Cache key includes the definition text hash, so a wrong definition for a type/MD5 never
        poisons a later connection that carries the correct definition."""
        definition = hdr.get("message_definition", "")
        key = (hdr.get("type"), hdr.get("md5sum"), hashlib.sha256(definition.encode()).hexdigest())
        if key in self.types:
            return self.types[key]
        cls, status = None, "no_class"
        try:
            import genpy.message
            c = genpy.message.get_message_class(key[0])
            if c is not None and c._md5sum == key[1]:
                cls, status = c, "installed"
        except Exception:
            pass
        if cls is None and definition:
            try:
                import genpy.dynamic
                c = genpy.dynamic.generate_dynamic(key[0], definition)[key[0]]
                if c._md5sum == key[1]:
                    cls, status = c, "dynamic"
                else:
                    status = "definition_md5_mismatch"
            except Exception:
                status = "dynamic_generation_failed"
        self.types[key] = cls
        self.type_status[key] = status
        return cls

    def row(self, ev):
        """CSV row: identity columns always from the Reception; decoded columns best-effort."""
        a = self.a
        r = dict.fromkeys(CSV_COLUMNS, "")
        r.update(ev.ident())
        r["run_id"] = a.run_id
        if ev.topic in ASSOC_TOPICS:
            r["assoc_latest_avoid_waypoints_event_id"] = "" if ev.assoc_path_event_id is None else ev.assoc_path_event_id
            r["assoc_latest_avoid_waypoints_sha256"] = ev.assoc_path_sha256 or ""
        try:
            self._decode_into(ev, r)
        except Exception as e:
            r["decode_status"] = "decode_exception"
            r["decode_error"] = repr(e)[:300]
        if r["decode_status"] != "ok":
            self.count(ev.topic, "decode_failed")
        return r

    def _decode_into(self, ev, r):
        a = self.a
        cls = self.msg_class(ev.header)
        if cls is None:
            d = ev.header.get("message_definition", "")
            r["decode_status"] = self.type_status.get((ev.header.get("type"), ev.header.get("md5sum"),
                                                        hashlib.sha256(d.encode()).hexdigest()), "no_class")
            return
        try:
            m = cls().deserialize(ev.buff)
        except Exception as e:
            r["decode_status"] = "deserialize_failed"
            r["decode_error"] = repr(e)[:300]
            return
        r["decode_status"] = "ok"
        h = getattr(m, "header", None)
        if h is not None and hasattr(h, "stamp") and hasattr(h, "seq"):
            r.update({"has_header": 1, "header_seq": h.seq, "header_stamp_ns": h.stamp.to_nsec(), "header_frame_id": h.frame_id})
        else:
            r["has_header"] = 0
        ty, topic, special = ev.msg_type, ev.topic, False
        if ty == "morai_msgs/CtrlCmd":
            special = True
            for f in CMD_FIELDS:
                if hasattr(m, f):
                    r["cmd_" + f] = fnum(getattr(m, f))
            r.update({"cmd_semantics": "ros_publication_received", "applied_command_ack": "unavailable",
                      "wire_field_presence": "unknown"})
        elif ty == "morai_msgs/EgoVehicleStatus":
            special = True
            absent = []
            for v in EGO_VECTORS:
                vec = getattr(m, v, None)
                if vec is None:
                    absent.append(v)
                    continue
                for ax in "xyz":
                    r["ego_%s_%s" % (v, ax)] = fnum(getattr(vec, ax))
            for f in EGO_SCALARS:
                if hasattr(m, f):
                    r["ego_" + f] = fnum(getattr(m, f))
                else:
                    absent.append(f)
            r["ego_ros_fields_absent"] = ",".join(absent)
            r["wire_field_presence"] = "unknown"
        elif ty == "std_msgs/String":
            special = True
            txt = m.data
            if topic in PATH_JSON_TOPICS or len(txt.encode()) > a.text_limit:
                r["text_in_bag_only"] = 1
            else:
                r["text_data"], r["text_in_bag_only"] = txt, 0
            try:
                j = json.loads(txt)
                r["text_json_ok"] = 1
            except Exception:
                j, r["text_json_ok"] = None, 0
            if isinstance(j, dict):
                ref = {k: v for k, v in j.items() if "refresh" in k.lower()}
                if ref:
                    r["refresh_fields_json"] = json.dumps(ref, sort_keys=True)
                if topic in PATH_JSON_TOPICS:
                    r["path_meta_json"] = json.dumps({k: v for k, v in j.items() if not isinstance(v, (list, dict))}, sort_keys=True)
                    wps = j.get("waypoints")
                    r["path_n_points"] = len(wps) if isinstance(wps, list) else ""
        elif ty in ("std_msgs/Float32", "std_msgs/Float64"):
            special = True
            r["float_data"] = fnum(m.data)
        elif ty in ("std_msgs/Int32", "std_msgs/Int64", "std_msgs/Int8", "std_msgs/UInt8", "std_msgs/Bool"):
            special = True
            r["int_data"] = fnum(m.data)
        elif ty in ("std_msgs/Float32MultiArray", "std_msgs/Float64MultiArray"):
            special = True
            r["array_data"] = json.dumps([fnum(x) for x in m.data])
        elif ty == "nav_msgs/Path":
            special = True
            r["path_n_points"] = len(m.poses)
        elif ty.startswith("morai_msgs/ObjectStatusList"):
            special = True
            for f, col in (("npc_list", "objects_npc"), ("pedestrian_list", "objects_pedestrian"), ("obstacle_list", "objects_obstacle")):
                if hasattr(m, f):
                    r[col] = len(getattr(m, f))
        if not special and len(ev.buff) <= a.small_json_limit:
            r["small_msg_json"] = json.dumps(msg_to_obj(m), sort_keys=True)

    # ---------- writer thread: sole owner of bag/CSV I/O and finalization ----------
    def handle(self, ev):
        topic = ev.topic
        if not self.capacity_exhausted and time.monotonic() - self._last_cap >= self.a.capacity_check_period:
            self._last_cap = time.monotonic()
            self.check_capacity()
        if self.capacity_exhausted:
            self.count(topic, "dropped_capacity")
            return
        r = self.row(ev)
        if not self.bag_failed:
            try:
                if self.shards.due_rotation():
                    if self.shards.bag is not None:
                        try:
                            self.shards.close()
                        except ShardError as e:
                            self.error("shard_rotate_close", e)
                            self.reason("shard_close_failed")
                    self.shards.open_next()
                meta = dict(ev.ident(), run_id=self.a.run_id, bag_time_policy=BAG_TIME_POLICY,
                            assoc_latest_avoid_waypoints_event_id=ev.assoc_path_event_id,
                            assoc_latest_avoid_waypoints_sha256=ev.assoc_path_sha256)
                self.shards.write_event(ev, json.dumps(meta, sort_keys=True))
                ev.bag_done = True
            except Exception as e:
                self.bag_failed = True
                self.error("bag_write", e)
                self.reason("bag_write_failed")
                try:
                    self.shards.close(state="closed_after_write_error")
                except ShardError as e2:
                    self.error("shard_close_after_write_error", e2)
                    self.reason("shard_close_failed")
        if not self.csv_failed:
            try:
                self.csv_writer.writerow([r[k] for k in CSV_COLUMNS])
                ev.csv_done = True
                self.csv_rows += 1
            except Exception as e:
                self.csv_failed = True
                self.error("csv_write", e)
                self.reason("csv_write_failed")
        self.account(ev)

    def account(self, ev):
        k = {(True, True): "persisted_full", (False, True): "persisted_csv_only",
             (True, False): "persisted_bag_only", (False, False): "failed_none"}[(ev.bag_done, ev.csv_done)]
        self.count(ev.topic, k)

    def check_capacity(self):
        try:
            fb = free_bytes(self.a.capture_dir)
        except Exception as e:
            self.error("capacity_check", e)
            return
        if fb < self.a.min_free_bytes:
            self.capacity_exhausted = True
            self.reason("capacity_exhausted")
            self.error("capacity", RuntimeError("free %d < min %d" % (fb, self.a.min_free_bytes)))
            try:
                self.shards.close(state="closed_capacity")
            except ShardError as e:
                self.error("shard_close_capacity", e)
                self.reason("shard_close_failed")

    def writer_main(self):
        try:
            while True:
                try:
                    ev = self.q.get(timeout=0.05)
                except queue.Empty:
                    if self.abandoned:
                        return
                    if self.finalize_req.is_set() and not self.admitting:
                        break
                    self.flush_csv()
                    continue
                if self.abandoned:
                    return
                self.inflight = ev
                try:
                    self.handle(ev)
                except BaseException as e:   # unexpected: account the event, report, keep going
                    self.error("writer_event_%d" % ev.event_id, e)
                    self.reason("writer_event_exception")
                    self.account(ev)
                finally:
                    self.inflight = None
            self.finalize()
        except BaseException as e:
            self.writer_fatal = repr(e)
            self.error("writer_fatal", e)
            self.reason("writer_dead")
        finally:
            self.writer_exit.set()

    def flush_csv(self):
        if self.csv_file is not None and not self.csv_failed:
            try:
                self.csv_file.flush()
            except Exception as e:
                self.csv_failed = True
                self.error("csv_flush", e)
                self.reason("csv_write_failed")

    def finalize(self):
        """Owned finalization: close/fsync/hash shard, fsync/close CSV, snapshot. Each failure is a reason."""
        if self.abandoned:
            return
        try:
            self.shards.close()
        except ShardError as e:
            self.error("shard_final_close", e)
            self.reason("shard_close_failed")
        if self.abandoned:
            return
        try:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
        except Exception as e:
            self.error("csv_fsync", e)
            self.reason("csv_write_failed")
        try:
            self.csv_file.close()
        except Exception as e:
            self.error("csv_close", e)
            self.reason("csv_write_failed")
        if self.abandoned or self.startup_failed or self.capacity_exhausted:
            return
        try:
            dst = os.path.join(self.a.capture_dir, "events.csv")
            shutil.copyfile(self.csv_path, dst)
            src_sha, dst_sha = sha256_file(self.csv_path), sha256_file(dst)
            if src_sha != dst_sha:
                raise IOError("snapshot hash mismatch")
            self.final["snapshot"] = {"file": "events.csv", "sha256": dst_sha}
        except Exception as e:
            self.error("csv_snapshot", e)
            self.reason("csv_snapshot_failed")

    # ---------- status (main thread only) ----------
    def unresolved_counts(self):
        """Admitted events not yet terminal: queued items + the writer's in-flight event."""
        per = {}
        with self.q.mutex:
            items = list(self.q.queue)
        cur = self.inflight
        if cur is not None:
            items.append(cur)
        for ev in items:
            per[ev.topic] = per.get(ev.topic, 0) + 1
        return per

    def capture_obj(self, unresolved=None):
        a = self.a
        with self.lock:
            counts = json.loads(json.dumps(self.counts))
            reasons, errors = list(self.reasons), list(self.errors)
        un = unresolved if unresolved is not None else self.unresolved_counts()
        inv_ok = True
        for t, c in counts.items():
            c["unresolved"] = un.get(t, 0)
            c["invariant_received"] = c["received"] == c["admitted"] + c["rejected_after_close"]
            c["invariant_admitted"] = c["admitted"] == sum(c[k] for k in TERMINAL) + c["unresolved"]
            inv_ok = inv_ok and c["invariant_received"] and c["invariant_admitted"]
        missing = sorted(t for t, c in counts.items() if c["received"] == 0)
        conflicts = {t: c["types"] for t, c in counts.items() if len(c["types"]) > 1}
        return {
            "schema": "drive_diagnostics_capture/2",
            "state": self.state,
            "incomplete_reasons": reasons,
            "errors": errors, "errors_not_listed": self.error_overflow, "warnings": list(self.warnings),
            "session_id": a.session_id, "run_id": a.run_id,
            "ros_master_uri": os.environ.get("ROS_MASTER_URI"),
            "node_name": a.node_name, "pid": os.getpid(), "host": socket.gethostname(),
            "times": self.times,
            "capture_dir": a.capture_dir, "latest_csv": a.latest_csv,
            "files": {"bag_shards": self.shards.snapshot() if self.shards else [],
                      "csv_rows_written": self.csv_rows, "csv_snapshot": self.final.get("snapshot"),
                      "ready_marker": "ready" if os.path.exists(os.path.join(a.capture_dir, "ready")) else None},
            "writer": {"alive": getattr(self, "writer", None) is not None and self.writer.is_alive(),
                       "fatal": self.writer_fatal, "abandoned": self.abandoned},
            "config": {"topics": a.topics, "queue_size": a.queue_size, "split_bytes": a.split_bytes,
                       "split_seconds": a.split_seconds, "min_free_bytes": a.min_free_bytes,
                       "drain_timeout": a.drain_timeout, "startup_timeout": a.startup_timeout,
                       "rpc_timeout": a.rpc_timeout, "text_limit": a.text_limit, "small_json_limit": a.small_json_limit},
            "topics": counts, "accounting_invariant_ok": inv_ok,
            "accounting_definition": "received = admitted + rejected_after_close; admitted = " + " + ".join(TERMINAL) + " + unresolved",
            "missing_topics": missing, "type_conflicts": conflicts,
            "ordering": {"event_id": "admission order under one lock (clocks stamped inside the lock)",
                         "assoc": "latest ADMITTED /avoid_waypoints at admission of the row (never a later reception; may be a dropped event)",
                         "bag_time_policy": BAG_TIME_POLICY},
            "bag_local_meta_topic": META_TOPIC,
            "semantics": {"ctrl_cmd": "exact ROS publication received by this node",
                          "applied_command_ack": "unavailable", "wire_field_presence": "unknown",
                          "active_authority_verified": False, "manual_override": "not inferred",
                          "service_responses": "not recorded", "row_model": "one row per reception; no fusion or interpolation"},
            "source_metadata": self.source_meta,
        }

    def save_status(self, unresolved=None):
        try:
            atomic_write(os.path.join(self.a.capture_dir, "capture.json"),
                         json.dumps(self.capture_obj(unresolved), indent=1, sort_keys=True))
            return True
        except Exception as e:
            self.status_ok = False
            self.reason("status_persist_failed")
            self.error("capture.json", e)
            return False

    def periodic_status(self):
        t = time.monotonic()
        if t - self.last_status >= self.a.status_period:
            self.last_status = t
            self.save_status()


def positive(kind):
    def f(s):
        v = kind(s)
        if not v > 0:
            raise argparse.ArgumentTypeError("must be > 0")
        return v
    return f


def non_negative(kind):
    def f(s):
        v = kind(s)
        if v < 0:
            raise argparse.ArgumentTypeError("must be >= 0")
        return v
    return f


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--session-id", required=True)
    p.add_argument("--run-id", required=True)
    p.add_argument("--capture-root", required=True, help="drive capture root (not data/raw)")
    p.add_argument("--capture-dir", default=None, help="exact capture directory (must not exist); default <root>/<session>_<UTC>_<run>")
    p.add_argument("--latest-csv", required=True)
    p.add_argument("--node-name", default=None)
    p.add_argument("--topics", default=",".join(DEFAULT_TOPICS))
    p.add_argument("--queue-size", type=positive(int), default=20000)
    p.add_argument("--split-bytes", type=non_negative(int), default=512 * 1024 * 1024, help="0 disables size split")
    p.add_argument("--split-seconds", type=non_negative(float), default=300.0, help="0 disables time split")
    p.add_argument("--min-free-bytes", type=non_negative(int), default=2 * 1024 ** 3)
    p.add_argument("--capacity-check-period", type=non_negative(float), default=1.0)
    p.add_argument("--status-period", type=positive(float), default=1.0)
    p.add_argument("--text-limit", type=positive(int), default=65536)
    p.add_argument("--small-json-limit", type=non_negative(int), default=4096)
    p.add_argument("--drain-timeout", type=positive(float), default=10.0)
    p.add_argument("--rpc-timeout", type=positive(float), default=2.0)
    p.add_argument("--startup-timeout", "--ready-timeout", dest="startup_timeout", type=positive(float), default=20.0,
                   help="bound for the whole startup incl. registration (--ready-timeout kept as alias)")
    p.add_argument("--master-timeout", type=positive(float), default=None, help="master check RPC timeout (default --rpc-timeout)")
    p.add_argument("--unregister-timeout", type=positive(float), default=2.0)
    p.add_argument("--source-meta", action="append", default=[], help="JSON file with source params/hashes to embed")
    p.add_argument("--hash-file", action="append", default=[], help="file whose sha256 is recorded")
    a = p.parse_args(argv)
    a.topics = [t for t in a.topics.split(",") if t]
    if not a.topics:
        p.error("no topics")
    if not re.match(r"^[A-Za-z0-9_.-]+$", a.run_id) or not re.match(r"^[A-Za-z0-9_.-]+$", a.session_id):
        p.error("session/run id must match [A-Za-z0-9_.-]+")
    if a.node_name is None:
        a.node_name = "drive_diag_rec_" + re.sub(r"[^A-Za-z0-9_]", "_", a.run_id)
    a.capture_root = os.path.abspath(a.capture_root)
    a.latest_csv = os.path.abspath(a.latest_csv)
    if "/data/raw/" in a.capture_root + "/" or "/data/raw/" in a.latest_csv:
        p.error("capture root / latest csv must not be inside data/raw")
    utc = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    if a.capture_dir:
        a.capture_dir = os.path.abspath(a.capture_dir)
        if os.path.dirname(a.capture_dir) != a.capture_root or not os.path.basename(a.capture_dir).endswith("_" + a.run_id):
            p.error("--capture-dir must be <capture-root>/<name>_<run-id>")
    else:
        a.capture_dir = os.path.join(a.capture_root, "%s_%s_%s" % (a.session_id, utc, a.run_id))
    return a


def run(argv=None):
    import rospy
    argv = rospy.myargv(sys.argv if argv is None else argv)[1:]
    a = parse_args(argv)
    rec = Recorder(a)
    sigs = []
    stop_req, abandon_req = threading.Event(), threading.Event()

    def on_signal(signum, _frame):
        sigs.append(signum)
        if stop_req.is_set():
            abandon_req.set()      # second signal: bounded immediate abandonment
        stop_req.set()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # capture directory: unique per run id, never reused
    try:
        os.makedirs(a.capture_root, exist_ok=True)
        if any(d.endswith("_" + a.run_id) for d in os.listdir(a.capture_root)):
            print("run id %s already has a capture directory; refusing" % a.run_id, file=sys.stderr)
            return 3
        os.makedirs(a.capture_dir)
    except OSError as e:
        print("cannot create capture dir: %r" % e, file=sys.stderr)
        return 2
    rec.times["start_wall_ns"], rec.times["start_monotonic_ns"] = now_ns()
    for f in a.source_meta:
        try:
            rec.source_meta["files"].append({"path": f, "sha256": sha256_file(f), "content": json.load(open(f))})
        except Exception as e:
            rec.source_meta["files"].append({"path": f, "error": repr(e)})
    for f in a.hash_file:
        try:
            rec.source_meta["hashes"].append({"path": f, "sha256": sha256_file(f)})
        except Exception as e:
            rec.source_meta["hashes"].append({"path": f, "error": repr(e)})
    if not rec.save_status():
        print("cannot persist initial capture.json", file=sys.stderr)
        return 2

    phase = threading.Lock()   # serializes the startup watchdog with the latest switch
    st = {"lock_fd": None, "staging": None, "backup": None, "switched": False, "ready": False, "dead": False}
    latest_dir = os.path.dirname(a.latest_csv)
    staging_path = os.path.join(latest_dir, ".%s.staging.%s" % (os.path.basename(a.latest_csv), a.run_id))
    backup_path = os.path.join(latest_dir, ".%s.previous.%s" % (os.path.basename(a.latest_csv), a.run_id))

    def owned_startup_cleanup():
        """Undo only owned startup artefacts; previous latest restored/untouched."""
        if st["switched"]:
            try:
                if st["backup"]:
                    os.replace(backup_path, a.latest_csv)       # atomic restore of the previous latest
                else:
                    os.unlink(a.latest_csv)
                fsync_dir(latest_dir)
                rec.times["latest_restored"] = True
            except OSError as e:
                rec.error("latest_restore", e)
                rec.reason("latest_restore_failed")
            st["switched"] = False
        elif st["backup"] and os.path.exists(backup_path):
            try:
                os.unlink(backup_path)
            except OSError as e:
                rec.error("backup_cleanup", e)
        if rec.csv_path and os.path.exists(rec.csv_path) and rec.csv_path != a.latest_csv:
            try:
                os.replace(rec.csv_path, os.path.join(a.capture_dir, "startup_staging_events.csv"))
            except OSError:
                try:
                    shutil.copyfile(rec.csv_path, os.path.join(a.capture_dir, "startup_staging_events.csv"))
                    os.unlink(rec.csv_path)
                except OSError as e:
                    rec.error("staging_cleanup", e)
        if os.path.exists(os.path.join(a.capture_dir, "ready")):
            try:
                os.unlink(os.path.join(a.capture_dir, "ready"))
            except OSError as e:
                rec.error("ready_cleanup", e)

    def startup_fail(reason, exc=None, code=2):
        rec.startup_failed = True
        rec.state = "failed_startup"
        rec.reason(reason)
        if exc is not None:
            rec.error("startup", exc)
        rec.close_admission()
        w = getattr(rec, "writer", None)
        if w is not None and w.is_alive():
            rec.finalize_req.set()
            if not rec.writer_exit.wait(a.drain_timeout):
                rec.abandoned = True
                rec.reason("drain_timeout")
        elif rec.shards is not None and rec.shards.bag is not None:
            try:
                rec.shards.close(state="closed_startup_failure")
            except ShardError as e:
                rec.error("shard_close_startup", e)
            try:
                rec.csv_file.close()
            except Exception:
                pass
        owned_startup_cleanup()
        rec.times["end_wall_ns"], rec.times["end_monotonic_ns"] = now_ns()
        rec.save_status()
        print("startup failed: %s %s" % (reason, "" if exc is None else repr(exc)), file=sys.stderr)
        return code

    def watchdog():
        if not stop_req.wait(0) and ready_evt.wait(a.startup_timeout):
            return
        with phase:
            if st["ready"] or st["dead"]:
                return
            st["dead"] = True
            reason = "startup_deadline_exceeded" if not stop_req.is_set() else "stopped_during_startup"
            code = startup_fail(reason)
        os._exit(code)
    ready_evt = threading.Event()

    # 1. master reachable (timed RPC; no global socket timeout)
    try:
        master_rpc(os.environ.get("ROS_MASTER_URI", ""), a.master_timeout or a.rpc_timeout).getPid("/" + a.node_name)
    except Exception as e:
        return startup_fail("master_unreachable", e)

    # 2. single-writer ownership
    try:
        os.makedirs(latest_dir, exist_ok=True)
        st["lock_fd"] = os.open(a.latest_csv + ".lock", os.O_RDWR | os.O_CREAT, 0o644)
        fcntl.flock(st["lock_fd"], fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as e:
        try:
            rec.times["lock_holder"] = open(a.latest_csv + ".lock").read()
        except Exception:
            pass
        if st["lock_fd"] is not None:
            os.close(st["lock_fd"])
        return startup_fail("latest_csv_lock_held" if e.errno in (errno.EAGAIN, errno.EACCES) else "lock_error", e, code=3)
    os.ftruncate(st["lock_fd"], 0)
    os.write(st["lock_fd"], json.dumps({"pid": os.getpid(), "run_id": a.run_id, "session_id": a.session_id,
                                        "capture_dir": a.capture_dir, "wall_ns": time.time_ns()}).encode())
    wd = threading.Thread(target=watchdog, name="startup_watchdog", daemon=True)
    wd.start()

    # 3. staged outputs (previous latest untouched)
    try:
        if free_bytes(latest_dir) < a.min_free_bytes:
            raise OSError(errno.ENOSPC, "free space below --min-free-bytes")
        rec.csv_path = staging_path
        rec.csv_file = open(staging_path, "w", newline="")
        rec.csv_writer = csv.writer(rec.csv_file, lineterminator="\n")
        rec.csv_writer.writerow(CSV_COLUMNS)
        rec.csv_file.flush()
        rec.shards = ShardWriter(a.capture_dir, "diag", a.split_bytes, a.split_seconds)
        rec.shards.open_next()
    except Exception as e:
        with phase:
            st["dead"] = True
            return startup_fail("output_init_failed", e)

    # 4. writer thread, admission, node, subscriptions
    rec.writer = threading.Thread(target=rec.writer_main, name="diag_writer", daemon=True)
    rec.writer.start()
    subs = []
    try:
        with rec.adm:
            rec.admitting = True
        rospy.init_node(a.node_name, anonymous=False, disable_signals=True, disable_rosout=True)
        for t in a.topics:
            subs.append(rospy.Subscriber(t, rospy.AnyMsg, rec.make_cb(t), queue_size=None))
        me = "/" + a.node_name
        deadline = time.monotonic() + a.startup_timeout
        missing = set(a.topics)
        while not stop_req.is_set():
            remain = deadline - time.monotonic()
            if remain <= 0:
                break
            try:
                code, msg, (_, subs_state, _) = master_rpc(os.environ["ROS_MASTER_URI"], min(a.rpc_timeout, remain)).getSystemState(me)
                missing = set(a.topics) - {t for t, nodes in subs_state if me in nodes}
            except Exception as e:
                rec.error("registration_rpc", e)
            if not missing:
                break
            time.sleep(0.05)
        if stop_req.is_set():
            raise RuntimeError("stop requested during startup")
        if missing:
            raise RuntimeError("subscriptions not registered before deadline: %s" % sorted(missing))
    except Exception as e:
        with phase:
            if st["dead"]:
                time.sleep(3600)   # the watchdog owns the exit
            st["dead"] = True
            return startup_fail("subscription_failed" if not stop_req.is_set() else "stopped_during_startup", e)

    # 5. atomic switch of the latest CSV, ready marker, recording manifest (rollback on failure)
    with phase:
        if st["dead"]:
            time.sleep(3600)
        try:
            if os.path.exists(a.latest_csv):
                if os.path.exists(backup_path):
                    os.unlink(backup_path)
                os.link(a.latest_csv, backup_path)
                st["backup"] = True
            fd = os.open(staging_path, os.O_RDONLY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
            os.replace(staging_path, a.latest_csv)
            rec.csv_path = a.latest_csv
            st["switched"] = True
            fsync_dir(latest_dir)
            rec.times["latest_switched_wall_ns"] = time.time_ns()
            rec.state = "recording"
            rec.times["ready_wall_ns"], rec.times["ready_monotonic_ns"] = now_ns()
            if not rec.save_status():
                raise IOError("capture.json not persisted before ready")
            # Commit point: the rename that makes "ready" visible. Nothing after it rolls back.
            publish_ready(os.path.join(a.capture_dir, "ready"), {
                "run_id": a.run_id, "session_id": a.session_id, "committed": True,
                "pid": os.getpid(), "pid_starttime": proc_starttime(os.getpid()), "boot_id": boot_id(),
                "ros_master_uri": os.environ.get("ROS_MASTER_URI"), "capture_dir": a.capture_dir, "latest_csv": a.latest_csv,
                "wall_ns": rec.times["ready_wall_ns"],
                "meaning": "committed: latest CSV switched, capture.json(recording) persisted, all subscribers registered; "
                           "publishers may not exist yet; no rollback after this marker exists"})
            st["ready"] = True
            ready_evt.set()
        except Exception as e:
            st["dead"] = True
            rec.csv_path = staging_path if not st["switched"] else rec.csv_path
            if st["switched"]:
                # keep our rows as evidence under the capture dir before the latest name is restored
                try:
                    shutil.copyfile(a.latest_csv, os.path.join(a.capture_dir, "startup_staging_events.csv"))
                except OSError as e2:
                    rec.error("staging_evidence", e2)
            code = startup_fail("ready_publication_failed", e)
            return code

    # post-commit housekeeping: failures are warnings only (never a rollback after readiness)
    if st["backup"]:
        try:
            os.unlink(backup_path)
            st["backup"] = False
        except OSError as e:
            rec.warnings.append({"where": "previous_latest_backup_cleanup", "error": repr(e), "path": backup_path})
    if _POST_PUBLISH.get("warning"):
        rec.warnings.append({"where": "ready_dir_fsync", "error": _POST_PUBLISH["warning"]})
    rec.save_status()

    # 6. recording: main monitors writer health and persists status
    while not stop_req.is_set():
        if not rec.writer.is_alive():
            rec.reason("writer_dead")
            break
        if rospy.is_shutdown():
            rec.reason("ros_shutdown")
            break
        rec.periodic_status()
        stop_req.wait(0.2)

    # 7. shutdown: stop request -> close admission -> unregister (bounded) -> writer drain+finalize (bounded)
    rec.state = "stopping"
    rec.times["stop_request_wall_ns"] = time.time_ns()
    rec.times["stop_signals"] = list(sigs)
    rec.close_admission()

    def unregister():
        for s in subs:
            try:
                s.unregister()
            except Exception as e:
                rec.error("unregister", e)
    ut = threading.Thread(target=unregister, daemon=True)
    ut.start()
    ut.join(a.unregister_timeout)
    if ut.is_alive():
        rec.error("unregister", RuntimeError("unregister exceeded %.1f s" % a.unregister_timeout))
    rec.finalize_req.set()
    t_end = time.monotonic() + a.drain_timeout
    while rec.writer.is_alive() and time.monotonic() < t_end and not abandon_req.is_set():
        rec.writer_exit.wait(0.1)
    if not rec.writer.is_alive() and rec.writer_fatal:
        # writer thread is dead, so no concurrent I/O: ownership of finalization reverts to main
        rec.times["main_finalized_after_writer_death"] = True
        try:
            rec.finalize()
        except BaseException as e:
            rec.error("main_finalize", e)
            rec.reason("finalize_failed")
        unresolved = rec.unresolved_counts()
        if sum(unresolved.values()):
            rec.reason("unresolved_events")
        rec.state = "incomplete"
        rec.times["end_wall_ns"], rec.times["end_monotonic_ns"] = now_ns()
        ok = rec.save_status(unresolved)
        os.close(st["lock_fd"])
        return 4 if ok else 6
    if rec.writer.is_alive():
        rec.abandoned = True       # writer must stop touching files; main never does
        rec.reason("drain_timeout" if not abandon_req.is_set() else "abandoned_by_second_signal")
        with rec.shards.lock:
            for s in rec.shards.shards:
                if s["state"] == "open":
                    s["state"] = "unresolved_open"
        unresolved = rec.unresolved_counts()
        if sum(unresolved.values()):
            rec.reason("unresolved_events")
        rec.state = "incomplete"
        rec.times["end_wall_ns"], rec.times["end_monotonic_ns"] = now_ns()
        ok = rec.save_status(unresolved)
        sys.stderr.write("recorder abandoned: writer %s, unresolved %s\n" % ("alive" if rec.writer.is_alive() else "dead", unresolved))
        sys.stderr.flush()
        os._exit(5 if ok else 6)
    rec.times["end_wall_ns"], rec.times["end_monotonic_ns"] = now_ns()
    unresolved = rec.unresolved_counts()
    if sum(unresolved.values()):
        rec.reason("unresolved_events")
    with rec.lock:
        reasons = list(rec.reasons)
    rec.state = "complete" if not reasons else "incomplete"
    obj_ok = rec.save_status(unresolved)
    if not obj_ok and rec.state == "complete":
        rec.state = "incomplete"
    try:
        rospy.signal_shutdown("recorder stop")
    except Exception:
        pass
    os.close(st["lock_fd"])
    if not obj_ok:
        sys.stderr.write("terminal capture.json could not be saved; not reporting success\n")
        return 6
    return 0 if rec.state == "complete" else 4


if __name__ == "__main__":
    sys.exit(run())
