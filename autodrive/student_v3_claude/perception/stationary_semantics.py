"""Road-only evidence for a stationary semantic person that is a low object.

Fusion publishes LiDAR height in ObjectStatus.size.z, but no class probability.
Tuple slot 17 holds shape evidence (slots 14-16 belong to vehicle motion).
No actor IDs, truth positions or simulator queries enter this policy.
"""
from __future__ import annotations

import math
import numpy as np

SHAPE_SLOT = 17
STATIC_EVIDENCE_SIM_S = 2.
STATIC_DISPLACEMENT_M = .15
MAX_SOURCE_GAP_S = .5


def metadata(obj):
    return obj[SHAPE_SLOT] if len(obj) > SHAPE_SLOT and isinstance(obj[SHAPE_SLOT], dict) else {}


def with_metadata(obj, **values):
    row = list(obj)
    while len(row) <= SHAPE_SLOT:
        row.append(None if len(row) != 15 else 0.)
    row[SHAPE_SLOT] = dict(metadata(obj), **values)
    return tuple(row)


def low_box(obj):
    """A broad, low measured cluster; missing height is not evidence."""
    shape = metadata(obj)
    size = shape.get('raw_size_m', obj[6:8])
    try:
        length, width, height = float(size[0]), float(size[1]), float(shape['height_m'])
    except (KeyError, TypeError, ValueError, IndexError):
        return False
    return (str(obj[3]).startswith('fusion_') and
            np.isfinite([length, width, height]).all() and
            .9 < max(length, width) <= 1.95 and .15 <= min(length, width) <= 1.2 and
            .15 <= height <= 1.1)


def _corners(obj):
    h = math.radians(float(obj[8]))
    axes = np.array([[math.cos(h), math.sin(h)], [-math.sin(h), math.cos(h)]])
    return (np.array([[-1., -1.], [-1., 1.], [1., -1.], [1., 1.]]) *
            np.asarray(obj[6:8], float)/2.) @ axes + obj[:2]


def annotate_static_candidates(objects):
    """Capture overlapping low boxes before pedestrian-fragment deduplication."""
    boxes = [o for o in objects if o[4] == 'obstacle' and abs(float(o[5])) < .2 and low_box(o)]
    out = []
    for obj in objects:
        if obj[4] != 'pedestrian' or obj[3] != 'fusion_pedestrian':
            out.append(obj)
            continue
        # Re-evaluate this observation; old evidence tags cannot survive new
        # shape data. An explicitly tall human overrides a nearby low box.
        if 'static_shape_reason' in metadata(obj):
            row = list(obj)
            row[SHAPE_SLOT] = {k:v for k,v in metadata(obj).items()
                               if k not in ('static_shape_reason', 'static_support_box')}
            obj = tuple(row)
        height = metadata(obj).get('height_m')
        if isinstance(height, (int, float)) and math.isfinite(height) and height > 1.1:
            out.append(obj)
            continue
        support = None
        for box in boxes:
            if np.linalg.norm(np.asarray(obj[:2])-box[:2]) > .6:
                continue
            h = math.radians(float(box[8]))
            axes = np.array([[math.cos(h), math.sin(h)], [-math.sin(h), math.cos(h)]])
            rel = (_corners(obj)-box[:2]) @ axes.T
            half = np.asarray(box[6:8], float)/2.
            if np.all(rel.max(axis=0) >= -half) and np.all(rel.min(axis=0) <= half):
                support = box
                break
        if support is not None or low_box(obj):
            obj = with_metadata(obj, static_shape_reason='overlapping_low_static_box' if support else 'low_broad_lidar_cluster',
                                static_support_box=support)
        out.append(obj)
    return out


def as_obstacle(obj, reason):
    """Keep the complete observed footprint, including an overlapping box."""
    row = list(obj)
    support = metadata(obj).get('static_support_box')
    if support is not None:
        h = math.radians(float(obj[8]))
        axes = np.array([[math.cos(h), math.sin(h)], [-math.sin(h), math.cos(h)]])
        points = np.vstack((_corners(obj), _corners(support))) @ axes.T
        lo, hi = points.min(axis=0), points.max(axis=0)
        centre = (lo+hi)/2. @ axes
        row[0], row[1] = float(centre[0]), float(centre[1])
        row[6], row[7] = map(float, hi-lo)
        row[2] = max(row[6:8])
    row[3], row[4] = 'fusion_static_candidate', 'obstacle'
    row[5], row[9], row[10] = 0., 0., 0.
    return with_metadata(row, pedestrian_static_reason=reason, semantic_person=tuple(obj))


class StaticPedestrianEvidence:
    """Fresh continuous simulator-time immobility plus independent shape evidence.

    Low overlapping static-box centres avoid mistaking alternating visible human
    fragments for translation. Any fresh person translation or measured motion
    revokes permission; stale/repeated frames cannot extend the evidence.
    """
    def __init__(self):
        self.tracks = {}

    def update(self, objects, stamp, now, rtf):
        from sim_time import clip_rtf
        current = {}
        for obj in objects:
            if obj[4] != 'pedestrian' or obj[3] != 'fusion_pedestrian' or len(obj) <= 11:
                continue
            shape = metadata(obj)
            if not shape.get('static_shape_reason') or abs(float(obj[5])) >= .2:
                continue
            xy = np.asarray(obj[:2], float)
            box = shape.get('static_support_box')
            centre = np.asarray(box[:2] if box is not None else obj[:2], float)
            if not np.isfinite(np.r_[xy, centre, stamp, now]).all():
                continue
            uid = int(obj[11])
            old = self.tracks.get(uid)
            fresh = old is not None and 0. <= now-old['seen'] <= MAX_SOURCE_GAP_S
            stable = (fresh and 0. <= stamp-old['stamp'] <= MAX_SOURCE_GAP_S and
                      np.linalg.norm(centre-old['anchor']) < STATIC_DISPLACEMENT_M and
                      np.linalg.norm(xy-old['person_anchor']) < STATIC_DISPLACEMENT_M and
                      np.linalg.norm(xy-old['xy']) < STATIC_DISPLACEMENT_M)
            if stable and stamp == old['stamp']:
                current[uid] = old
                continue
            elapsed = old['elapsed']+(stamp-old['stamp'])*clip_rtf(rtf) if stable else 0.
            current[uid] = dict(stamp=stamp, seen=now, xy=xy, anchor=old['anchor'] if stable else centre,
                                person_anchor=old['person_anchor'] if stable else xy,
                                elapsed=elapsed, reason=shape['static_shape_reason'])
        self.tracks = current

    def reason(self, obj, now):
        entry = self.tracks.get(int(obj[11])) if len(obj) > 11 and obj[11] is not None else None
        if (obj[4] == 'pedestrian' and obj[3] == 'fusion_pedestrian' and entry is not None and
                metadata(obj).get('static_shape_reason') and
                0. <= now-entry['seen'] <= MAX_SOURCE_GAP_S and abs(float(obj[5])) < .2 and
                entry['elapsed']+1e-6 >= STATIC_EVIDENCE_SIM_S and
                np.linalg.norm(np.asarray(obj[:2])-entry['xy']) < STATIC_DISPLACEMENT_M):
            return entry['reason']
        return None
