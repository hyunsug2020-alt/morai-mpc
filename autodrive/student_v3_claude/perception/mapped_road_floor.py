"""Pinned first-signal road elevation with source-frame ground calibration.

The 0.20 m map-to-mesh allowance is an explicit bounded-error assumption for
this surveyed MORAI patch. It is additional to calibration residuals and distance
leverage; statistical fit quality alone does not establish road elevation.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .imu_attitude import ATTITUDE_RATE_BOUND_RAD_S, MAX_IMU_DELTA_S

MAP_SHA256 = '5ce0fd57da07b83221c0a4d117b0606b2d9c094c8de476a7d1176c78a4dc5b0f'
ROUTE_SHA256 = '543ad736783c9bc6cb75ad4d545c07ec4399231f2e2ee163fdaac6d66b5e7655'
MAP_MESH_LOWER_M = .20
CALIBRATION_LOWER_M = .03
SOURCE_STATION_M = (145., 195.)
TARGET_STATION_M = (195., 230.)


class MappedRoadFloor:
    """An immutable, connected part of the known route, never a nearest-road mix."""

    def __init__(self, map_path, route_path):
        map_bytes = Path(map_path).read_bytes()
        route_bytes = Path(route_path).read_bytes()
        if (hashlib.sha256(map_bytes).hexdigest() != MAP_SHA256 or
                hashlib.sha256(route_bytes).hexdigest() != ROUTE_SHA256):
            raise ValueError('mapped road floor source hash mismatch')
        links = json.loads(map_bytes)
        route = np.array([[p['x'], p['y']] for p in json.loads(route_bytes)['waypoints']], float)
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
        selected = np.flatnonzero((arc >= SOURCE_STATION_M[0]-2.) &
                                  (arc <= TARGET_STATION_M[1]+2.))
        xyz, owners = [], []
        by_id = {str(link['idx']): link for link in links}
        for link in links:
            for point in link['points']:
                xyz.append(point)
                owners.append(str(link['idx']))
        xyz = np.asarray(xyz, float)
        tree = cKDTree(xyz[:, :2])
        altitude, labels = [], []
        for point in route[selected]:
            exact = tree.query_ball_point(point, 1e-4)
            if not exact or np.ptp(xyz[exact, 2]) >= .01:
                raise ValueError('selected route has missing or ambiguous map elevation')
            altitude.append(float(np.min(xyz[exact, 2])))
            labels.append(tuple(sorted({owners[i] for i in exact})))
        # Interior vertices identify the actual directed route links. Shared
        # intersection nodes may name several links, but cannot change the chain.
        chain = []
        for choices in labels:
            if len(choices) == 1 and (not chain or choices[0] != chain[-1]):
                chain.append(choices[0])
        if not chain or any(by_id[a]['to_node_idx'] != by_id[b]['from_node_idx']
                            for a, b in zip(chain, chain[1:])):
            raise ValueError('selected map elevation links are not a connected directed route')
        self._route = route[selected].copy()
        self._arc = arc[selected].copy()
        self._altitude = np.asarray(altitude)
        for array in (self._route, self._arc, self._altitude):
            array.setflags(write=False)
        self._tree = cKDTree(self._route)
        self._chain = tuple(chain)
        self._certified = True

    @property
    def certified(self):
        return self._certified

    def _project(self, world):
        _, nearest = self._tree.query(world)
        choices = np.column_stack((np.maximum(nearest-1, 0),
                                   np.minimum(nearest, len(self._route)-2)))
        start, end = self._route[choices], self._route[choices+1]
        direction = end-start
        delta = world[:, None, :]-start
        fraction = np.clip(np.sum(delta*direction, axis=2) /
                           np.maximum(np.sum(direction*direction, axis=2), 1e-12), 0., 1.)
        projected = start+fraction[:, :, None]*direction
        distances = np.linalg.norm(projected-world[:, None, :], axis=2)
        which = np.argmin(distances, axis=1)
        row = np.arange(len(world))
        index, u = choices[row, which], fraction[row, which]
        distance = distances[row, which]
        station = self._arc[index]+u*(self._arc[index+1]-self._arc[index])
        altitude = self._altitude[index]+u*(self._altitude[index+1]-self._altitude[index])
        tangent = self._route[index+1]-self._route[index]
        heading = np.arctan2(tangent[:, 1], tangent[:, 0])
        return altitude, station, distance, heading

    def heights(self, points, near_plane, pose):
        """Return per-return ground Z; any unsupported context keeps the near plane."""
        p = np.asarray(points, dtype=float)
        plane = np.asarray(near_plane, dtype=float)
        ground = p[:, :2] @ plane[:2]+plane[2]
        diagnostic = dict(applied=False, reason='unavailable source pose')
        if not isinstance(pose, dict):
            return ground, diagnostic
        try:
            values = [float(pose[key]) for key in ('x', 'y', 'heading_deg', 'stamp', 'age_s')]
        except (KeyError, TypeError, ValueError, OverflowError):
            return ground, diagnostic
        if (not self.certified or not np.isfinite(values).all() or
                not np.isfinite(p).all() or not np.isfinite(plane).all() or
                pose.get('measured') is not True or pose.get('src') == 'file' or
                values[3] <= 0. or not 0. <= values[4] <= .2):
            return ground, diagnostic
        diagnostic['reason'] = 'unavailable measured source attitude'
        try:
            quaternion = np.asarray(pose['imu_quaternion'], dtype=float)
            covariance = np.asarray(pose['imu_orientation_covariance'], dtype=float)
            imu_stamp, imu_age = float(pose['imu_stamp']), float(pose['imu_age_s'])
            source_stamps = np.asarray(pose['imu_source_stamps'], dtype=float)
            angular_velocity = np.asarray(pose['imu_angular_velocity'], dtype=float)
            adjacent_rate = float(pose['imu_adjacent_rate_rad_s'])
        except (KeyError, TypeError, ValueError, OverflowError):
            return ground, diagnostic
        if (pose.get('imu_measured') is not True or quaternion.shape != (4,) or
                covariance.shape != (9,) or source_stamps.ndim != 1 or
                len(source_stamps) != 1 or angular_velocity.shape != (3,) or
                not np.isfinite(quaternion).all() or not np.isfinite(covariance).all() or
                not np.isfinite(source_stamps).all() or not np.isfinite(angular_velocity).all() or
                np.linalg.norm(angular_velocity[:2]) > .1 or
                not math.isfinite(adjacent_rate) or not 0. <= adjacent_rate <= ATTITUDE_RATE_BOUND_RAD_S or
                np.any(covariance[[0, 4, 8]] < 0.) or
                abs(np.linalg.norm(quaternion)-1.) > 1e-3 or
                not math.isfinite(imu_stamp) or abs(imu_stamp-values[3]) > MAX_IMU_DELTA_S or
                not math.isfinite(imu_age) or not 0. <= imu_age <= .2 or
                abs((imu_stamp-values[3])+(imu_age-values[4])) > 1e-6 or
                np.any(source_stamps <= 0.) or
                source_stamps.max() > values[3]+values[4]+1e-6 or
                abs(source_stamps[0]-imu_stamp) > 1e-6):
            return ground, diagnostic
        quaternion = quaternion/np.linalg.norm(quaternion)
        qx, qy, qz, qw = quaternion
        z_row = np.array([2.*(qx*qz-qy*qw), 2.*(qy*qz+qx*qw),
                          1.-2.*(qx*qx+qy*qy)])
        imu_yaw = math.atan2(2.*(qw*qz+qx*qy), 1.-2.*(qy*qy+qz*qz))
        if (z_row[2] < .99 or
                abs((imu_yaw-math.radians(values[2])+math.pi) % (2*math.pi)-math.pi)
                > math.radians(5.)):
            return ground, diagnostic
        diagnostic['reason'] = 'outside validated source route'
        source_altitude, source_station, source_distance, source_heading = self._project(np.array([values[:2]]))
        yaw = math.radians(values[2])
        heading_error = abs((yaw-source_heading[0]+math.pi) % (2*math.pi)-math.pi)
        if (not SOURCE_STATION_M[0] <= source_station[0] <= SOURCE_STATION_M[1] or
                source_distance[0] > .6 or heading_error > math.radians(10.)):
            return ground, diagnostic
        c, s = math.cos(yaw), math.sin(yaw)
        world = p[:, :2] @ np.array([[c, s], [-s, c]])+values[:2]
        mapped, station, distance, _ = self._project(world)
        mapped_valid = ((distance <= 1.7) & (station >= self._arc[0]) &
                        (station <= self._arc[-1]))
        relative = p[:, 2]-ground
        known = (mapped_valid & (p[:, 0] > 2.) & (p[:, 0] < 15.) &
                 (np.abs(p[:, 1]) < 1.5) & (np.abs(relative) < .12))
        diagnostic['reason'] = 'insufficient measured near ground'
        if np.count_nonzero(known) < 80:
            return ground, diagnostic
        ids = np.flatnonzero(known)
        base = (mapped-p[:, :2] @ z_row[:2])/z_row[2]
        delta = p[ids, 2]-base[ids]
        cells = np.floor(p[ids, :2]).astype(np.int32)
        order = np.argsort(delta, kind='stable')
        _, first = np.unique(cells[order], axis=0, return_index=True)
        ids = ids[order[first]]
        span = np.ptp(p[ids, :2], axis=0)
        if len(ids) < 12 or span[0] < 4. or span[1] < 2.:
            return ground, diagnostic
        singular = np.linalg.svd(p[ids, :2]-p[ids, :2].mean(axis=0), compute_uv=False)
        if singular[1] < .1*singular[0]:
            return ground, diagnostic
        # Roll/pitch come exclusively from the measured IMU. Fitting either
        # slope to a nearby surface would amplify a small ramp into a distant
        # false ground prediction. Only the vertical offset is calibrated here.
        delta = p[ids, 2]-base[ids]
        offset = float(np.mean(delta))
        residual = delta-offset
        rms = float(np.sqrt(np.mean(residual**2)))
        diagnostic['reason'] = 'map calibration inconsistent with measured ground'
        if rms > .015 or np.max(np.abs(residual)) > .04:
            return ground, diagnostic
        # This pinned Ioniq5 mount observes the road below its rear-axle frame.
        # A broad raised object in front must not masquerade as the entire road
        # and shift every subsequent map prediction upward.
        origin_ground = float(source_altitude[0]/z_row[2]+offset)
        if not -.65 <= origin_ground <= -.25:
            diagnostic['reason'] = 'near surface inconsistent with source body height'
            return ground, diagnostic
        candidate = (mapped_valid & (p[:, 0] > 20.) & (p[:, 0] < 45.) &
                     (station >= TARGET_STATION_M[0]) &
                     (station <= TARGET_STATION_M[1]))
        selected = np.flatnonzero(candidate)
        if not len(selected):
            diagnostic['reason'] = 'no measured return in validated target patch'
            return ground, diagnostic
        offset_uncertainty = 3.*rms/math.sqrt(len(ids))
        # An actual nearby sample is used, never an interpolated attitude. The
        # bounded operating rate covers both the target rotation and the scalar
        # offset calibrated from nearby points at the same small time offset.
        attitude_dt = abs(imu_stamp-values[3])
        near_radius = float(np.max(np.linalg.norm(p[ids, :2], axis=1)))
        timing_lower = (ATTITUDE_RATE_BOUND_RAD_S*attitude_dt*
                        (np.linalg.norm(p[selected, :2], axis=1)+near_radius))
        lower = (base[selected]+offset+float(residual.min())-
                 CALIBRATION_LOWER_M-MAP_MESH_LOWER_M-offset_uncertainty-timing_lower)
        corrected = lower > ground[selected]
        ground[selected] = np.maximum(ground[selected], lower)
        diagnostic.update(applied=bool(np.any(corrected)), reason='pinned road elevation lower bound',
            source_stamp=values[3], source_station_m=float(source_station[0]),
            map_sha256=MAP_SHA256, route_sha256=ROUTE_SHA256,
            route_links=list(self._chain), map_mesh_lower_m=MAP_MESH_LOWER_M,
            calibration_cells=len(ids), calibration_rms_m=rms,
            calibration_residual_min_m=float(residual.min()),
            calibration_span_m=span.tolist(), calibration_offset_m=offset,
            calibration_offset_uncertainty_m=offset_uncertainty,
            measured_imu_z_row=z_row.tolist(), imu_source_stamps=source_stamps.tolist(),
            imu_sample_delta_s=imu_stamp-values[3],
            attitude_rate_bound_rad_s=ATTITUDE_RATE_BOUND_RAD_S,
            attitude_timing_lower_max_m=float(np.max(timing_lower)),
            source_origin_ground_z_m=origin_ground,
            corrected_points=int(np.count_nonzero(corrected)))
        return ground, diagnostic
