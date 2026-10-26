"""Associate observed returns with surveyed roadside structures.

Mapped infrastructure leaves semantic clustering and model inference at the
point stage. Independent raw LiDAR collision checks keep the original cloud. Association uses
the sampled footprint, rather than a convex hull that fills empty road.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .roundabout_shapes import RoundaboutShapeRegion, ROUTE_SHA256


COPY_ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class RoadsideEvidence:
    structure_id: str
    position: tuple
    size: tuple
    yaw: float


class RoadsideStructures:
    """Fail closed when the saved map, route, source pose or points differ."""

    def __init__(self, route, *, map_path=None, survey_path=None):
        self.certified = False
        self.region = RoundaboutShapeRegion(route)
        try:
            payload = json.loads(Path(survey_path or COPY_ROOT/'roadside_structures.json').read_text())
            map_digest = hashlib.sha256(Path(map_path or COPY_ROOT/'link_set.json').read_bytes()).hexdigest()
            if (not self.region.certified or payload['schema'] != 1
                    or payload['route_sha256'] != ROUTE_SHA256
                    or payload['map_sha256'] != map_digest):
                return
            self.start, self.end = map(float, payload['station_window_m'])
            self.tolerance = float(payload['association_tolerance_m'])
            if (not all(math.isfinite(v) for v in (self.start, self.end, self.tolerance))
                    or not 0. <= self.start < self.end
                    or not 0. < self.tolerance <= .25):
                return
            self.structures = []
            for structure in payload['structures']:
                points = np.asarray(structure['observed_footprint_xy'], dtype=float)
                name = structure['id']
                if (not isinstance(name, str) or not name or points.ndim != 2
                        or points.shape[1] != 2 or len(points) < 4
                        or not np.isfinite(points).all()):
                    return
                self.structures.append((name, cKDTree(points)))
            self.certified = bool(self.structures)
        except (OSError, KeyError, TypeError, ValueError, OverflowError):
            pass

    def active(self, source_pose):
        if not self.certified:
            return False
        try:
            pose = np.asarray(source_pose, dtype=float)
            if pose.shape not in ((4,), (6,)) or not np.isfinite(pose).all() or pose[0] <= 0.:
                return False
            relative = pose[1:3]-self.region.starts
            along = np.clip(np.sum(relative*self.region.tangents, axis=1), 0., self.region.lengths)
            projected = self.region.starts+along[:, None]*self.region.tangents
            distances = np.linalg.norm(projected-pose[1:3], axis=1)
            index = int(np.argmin(distances))
            station = self.region.stations[index]+along[index]
            return bool(distances[index] <= 3. and self.start <= station <= self.end)
        except (TypeError, ValueError, OverflowError):
            return False

    def association(self, world_xy, source_pose, *, min_points=4):
        """Every source return must match one registered footprint."""
        if not self.active(source_pose):
            return None
        try:
            points = np.asarray(world_xy, dtype=float)
            if (points.ndim != 2 or points.shape[1] != 2 or len(points) < min_points
                    or not np.isfinite(points).all()):
                return None
            for name, tree in self.structures:
                distances, _ = tree.query(points, distance_upper_bound=self.tolerance)
                if np.isfinite(distances).all():
                    return name
        except (TypeError, ValueError, OverflowError):
            pass
        return None

    def point_mask(self, world_xy, source_pose):
        """Classify each known infrastructure return without swallowing its neighbours."""
        points = np.asarray(world_xy, dtype=float)
        result = np.zeros(len(points), dtype=bool)
        if (points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all()
                or not self.active(source_pose)):
            return result
        for _, tree in self.structures:
            distances, _ = tree.query(points, distance_upper_bound=self.tolerance)
            result |= np.isfinite(distances)
        return result

    def local_point_mask(self, points, source_pose, *, mount=None):
        """Use the aligned source pose; preserve every unregistered return."""
        values = np.asarray(points, dtype=float)
        if values.ndim != 2 or values.shape[1] < 2 or not self.active(source_pose):
            return np.zeros(len(values), dtype=bool)
        xy = values[:, :2]
        if mount is not None:
            xy = xy+np.asarray(mount, dtype=float)[:2]
        c, s = math.cos(source_pose[3]), math.sin(source_pose[3])
        world = xy @ np.array([[c, s], [-s, c]])+source_pose[1:3]
        return self.point_mask(world, source_pose)

    def semantic_evidence(self, detection, world_xy, source_pose):
        name = self.association(world_xy, source_pose)
        if name is None:
            return None
        return RoadsideEvidence(name, tuple(detection['pos']), tuple(detection['size']),
                                float(detection['yaw']))

    def vehicle_structure(self, points, box, mount, source_pose):
        """Validate the model prediction against its actual 3D source support.

        The model receives the full cloud. A prediction containing any return
        outside the saved footprint remains a vehicle, including parked cars.
        """
        if not self.active(source_pose):
            return None
        try:
            points = np.asarray(points, dtype=float)
            box = np.asarray(box, dtype=float)
            mount = np.asarray(mount, dtype=float)
            if (points.ndim != 2 or points.shape[1] < 3 or box.shape != (7,)
                    or mount.shape != (3,) or not np.isfinite(points[:, :3]).all()
                    or not np.isfinite(np.r_[box, mount]).all() or np.min(box[3:6]) <= 0.):
                return None
            relative = points[:, :3]-box[:3]
            c, s = math.cos(box[6]), math.sin(box[6])
            local = relative[:, :2] @ np.array([[c, -s], [s, c]])
            support = (np.abs(local) <= box[3:5]/2.).all(axis=1) & (np.abs(relative[:, 2]) <= box[5]/2.)
            body = points[support, :2]+mount[:2]
            c, s = math.cos(source_pose[3]), math.sin(source_pose[3])
            world = body @ np.array([[c, s], [-s, c]])+source_pose[1:3]
            name = self.association(world, source_pose, min_points=15)
            return (name, int(support.sum())) if name is not None else None
        except (TypeError, ValueError, IndexError, OverflowError):
            return None


def roadside_structure(detection):
    """Map identity owns infrastructure; tracker centroid jitter is not motion."""
    evidence = detection.get('roadside_evidence')
    if (not isinstance(evidence, RoadsideEvidence) or detection.get('kind') != 'obstacle'
            or detection.get('merged_part_ids')):
        return None
    try:
        velocity = np.asarray(detection['vel'], dtype=float)
        if (velocity.shape != (2,) or not np.isfinite(velocity).all()
                or not np.array_equal(detection['pos'], evidence.position)
                or not np.array_equal(detection['size'], evidence.size)
                or float(detection['yaw']) != evidence.yaw):
            return None
        return evidence.structure_id
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
