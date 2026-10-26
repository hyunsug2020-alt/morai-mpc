"""Highway lateral forecasts supported by distinct measured observations."""
from collections import deque
import math

import numpy as np

from highway_planner import world_points
from .object_policy import MOTION_TRACK_KEEP_S
from .policy import native_motion


# A long box swings its ends with yaw jitter: a 9x3.2 m truck box varied
# +-4.4 deg (+-0.35 m at its ends) while following its lane and cancelled a
# lane change passing it (run follow-20261009T204445Z t=223.72). While the
# centre track shows no lateral motion, the lane is the steadier heading.
LANE_HEADING_MIN_LENGTH_M = 6.
LANE_HEADING_WINDOW_S = .5      # recent box headings that must all agree with the lane
LANE_HEADING_TOLERANCE_DEG = 6.
LANE_FOLLOW_LATERAL_MPS = .3    # centre-track lateral speed beyond its own noise band
LANE_HEADING_GAP_S = .45        # more than two missed 7.7 Hz scans restarts the history
LANE_HEADING_JUMP_M = 2.        # centre off its own motion: another body


class HighwayMotionEvidence:
    """Keep measured bodies; suppress a lateral slope inside its own noise.

    Forward speed remains measured. Only highway lane-normal motion receives
    this confidence check; unknown, stopped and unmapped objects remain intact.
    ``lane_headings`` (uid -> deg) gives long lane followers their lane's
    heading for footprint checks; positions, sizes and the box stay measured.
    """
    def __init__(self):
        self.tracks = {}
        self.stamp = None
        self.lane_headings = {}

    @staticmethod
    def lane_heading(state, obj, stamp, path, tangent, centre_lateral, evidence):
        """Lane direction (deg) for a long lane follower, else None (measured yaw)."""
        samples = state.setdefault('headings', deque())
        position = np.asarray(obj[:2], dtype=float)
        if samples:
            last = samples[-1]
            gap = stamp-last[0]
            if (gap > LANE_HEADING_GAP_S or np.linalg.norm(
                    position-np.asarray(last[1:3])-np.asarray(last[4:6])*gap) > LANE_HEADING_JUMP_M):
                samples.clear()
        if not samples or stamp > samples[-1][0]:
            samples.append((stamp, *position, float(obj[8]), float(obj[9]), float(obj[10])))
        while stamp-samples[0][0] > LANE_HEADING_WINDOW_S+1e-9:
            samples.popleft()
        if (float(obj[6]) < LANE_HEADING_MIN_LENGTH_M or evidence == 'observed turn' or
                centre_lateral is None or centre_lateral >= LANE_FOLLOW_LATERAL_MPS or
                len(samples) < 3 or samples[-1][0]-samples[0][0] < .25):
            return None
        recent = np.asarray([s[:4] for s in samples])
        at = np.argmin(np.sum((recent[:, None, 1:3]-path[None])**2, axis=2), axis=1)
        lane = path[np.minimum(at+1, len(path)-1)]-path[np.maximum(at-1, 0)]
        deviation = (recent[:, 3]-np.degrees(np.arctan2(lane[:, 1], lane[:, 0]))+90.) % 180.-90.
        if not np.all(np.abs(deviation) <= LANE_HEADING_TOLERANCE_DEG):
            return None
        return math.degrees(math.atan2(tangent[1], tangent[0]))

    def update(self, objects, stamp, lanes, ego):
        objects = list(objects)
        self.lane_headings = {}
        if stamp is None or not math.isfinite(stamp) or stamp <= 0.:
            return objects, {'vehicles': [], 'source_stamp': stamp}
        if self.stamp is not None and stamp < self.stamp:
            self.tracks.clear()
        self.stamp = stamp
        self.tracks = {uid: state for uid, state in self.tracks.items()
                       if 0. <= stamp-state['observed_stamp'] <= MOTION_TRACK_KEEP_S}
        geometry = [(lane, world_points(lane.centre, ego)) for lane in lanes]
        result, diagnostics = [], []
        for obj in objects:
            if (obj[3] != 'lidar_car' or len(obj) < 15 or not obj[14] or
                    float(obj[5]) < 2. or not np.isfinite(obj[9:11]).all()):
                result.append(obj)
                continue
            age = float(obj[15]) if len(obj)>15 else 0.
            uid = obj[11]
            state = self.tracks.get(uid)
            if 0. < age <= MOTION_TRACK_KEEP_S and state is not None:
                # Detector predictions use their latest observation, which the
                # slower planner may never have consumed. Age is relative to
                # that observation, not necessarily our last validated sample.
                # Epoch timestamps need an absolute tolerance, not np.isclose's
                # default relative tolerance.
                if abs((stamp-age)-state['observed_stamp']) > 1e-6:
                    result.append(obj)
                    diagnostics.append(dict(uid=uid,
                        evidence='unseen observation prediction', age_s=age))
                    continue
                row = list(obj)
                row[0:2] = (state['position']+state['velocity']*age).tolist()
                row[9:11] = state['velocity'].tolist()
                row[5] = float(np.linalg.norm(state['velocity']))
                result.append(tuple(row))
                diagnostics.append(dict(uid=uid, evidence='retained motion', age_s=age))
                # One or two missed scans keep the last observation's lane heading.
                if state.get('lane_heading') is not None and age <= LANE_HEADING_GAP_S:
                    self.lane_headings[uid] = state['lane_heading']
                continue
            motion = native_motion(obj, geometry) if age == 0. else None
            if motion is None:
                if state is not None:
                    state.pop('headings', None)
                    state['lane_heading'] = None
                result.append(obj)
                continue
            if state is None:
                state = {'history': deque(maxlen=32)}
                self.tracks[uid] = state
            history = state['history']
            if not history or stamp > history[-1][0]:
                history.append((stamp, *obj[:2], float(obj[8])))
            path, stations, start, forward, _, raw = motion
            at = int(np.clip(np.searchsorted(stations, start), 1, len(path)-1))
            tangent = path[at]-path[at-1]
            tangent /= max(1e-6, float(np.linalg.norm(tangent)))
            normal = np.array([-tangent[1], tangent[0]])
            recent = np.asarray([r for r in history if stamp-r[0] <= 1.2])
            used, evidence, centre_lateral = raw, 'warmup', None
            if len(recent)>=3 and recent[-1, 0]-recent[0, 0]>=.3-1e-9:
                indices = np.argmin(np.sum((recent[:, None, 1:3]-path[None])**2, axis=2), axis=1)
                tangents = path[np.minimum(indices+1, len(path)-1)]-path[np.maximum(indices-1, 0)]
                tangents /= np.maximum(1e-6, np.linalg.norm(tangents, axis=1, keepdims=True))
                normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
                offsets = np.sum((recent[:, 1:3]-path[indices])*normals, axis=1)
                dt = recent[None, :, 0]-recent[:, None, 0]
                pairs = dt >= .3-1e-9
                slopes = (offsets[None]-offsets[:, None])[pairs]/dt[pairs]
                median = float(np.median(slopes))
                uncertainty = 2.5*1.4826*float(np.median(np.abs(slopes-median)))
                centre_lateral = max(0., abs(median)-uncertainty)
                used = (math.copysign(min(abs(raw), centre_lateral), raw)
                        if raw*median > 0. else 0.)
                evidence = 'measured trend' if abs(used)>.01 else 'uncertain lateral motion'
                # Independent current orientation preserves a clear, real turn
                # even while its position fit still carries acceleration lag.
                yaw = math.degrees(math.atan2(tangent[1], tangent[0]))
                angles = (recent[-2:, 3]-yaw+90.) % 180.-90.
                if (np.isfinite(angles).all() and np.all(np.abs(angles)>5.) and
                        np.all(np.abs(angles)<30.) and np.all(angles*raw>0.)):
                    used, evidence = raw, 'observed turn'
            velocity = tangent*forward+normal*used
            row = list(obj)
            row[9:11], row[5] = velocity.tolist(), float(np.linalg.norm(velocity))
            state.update(observed_stamp=history[-1][0], position=np.asarray(obj[:2]), velocity=velocity)
            state['lane_heading'] = self.lane_heading(state, obj, stamp, path, tangent,
                                                      centre_lateral, evidence)
            result.append(tuple(row))
            diagnostics.append(dict(uid=uid, evidence=evidence,
                raw_lateral_mps=round(raw, 3), used_lateral_mps=round(used, 3),
                samples=len(recent)))
            if state['lane_heading'] is not None:
                self.lane_headings[uid] = state['lane_heading']
                diagnostics[-1]['lane_heading_deg'] = round(state['lane_heading'], 2)
        return result, dict(source_stamp=stamp, vehicles=diagnostics)
