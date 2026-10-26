"""Bounded measured static returns for the tunnel portal approach only."""
import math
import threading

import numpy as np


class TunnelApproachMemory:
    """Keep actual world points across brief gaps in the portal's low returns.

    The caller owns approach activation and resets this memory on departure.
    Update poses belong to the scan source time; snapshot poses belong to the
    query source time. Only fresh, confirmed moving bodies may be supplied.
    Object tuples use the existing world-frame position/box/velocity format.
    """
    def __init__(self, max_age_s=1.5, voxel_m=.25, max_points=4096):
        self.max_age_s = float(max_age_s)
        self.voxel_m = float(voxel_m)
        self.max_points = int(max_points)
        if (not math.isfinite(self.max_age_s) or self.max_age_s <= 0. or
                not math.isfinite(self.voxel_m) or self.voxel_m <= 0. or self.max_points <= 0):
            raise ValueError('Approach memory bounds must be finite and positive')
        self.lock = threading.RLock()
        self.reset()

    def reset(self):
        with self.lock:
            self.cells = {}
            self.last_scan = self.last_query = self.last_pose = None
            self.last_pose_stamp = None
            self.source_poses = {}

    @staticmethod
    def _pose(ego, stamp):
        values = np.asarray([ego['x'], ego['y'], ego['heading_deg'], stamp], float)
        if not np.isfinite(values).all() or stamp <= 0.:
            raise ValueError('Approach memory needs a finite pose and positive source time')
        pose_stamp = float(ego.get('stamp', stamp))
        if not math.isfinite(pose_stamp) or abs(pose_stamp-stamp) > .12:
            raise ValueError('Approach memory pose must match the observation source time')
        if 'age_s' in ego and not 0. <= float(ego['age_s']) <= .25:
            raise ValueError('Approach memory cannot use a stale pose')
        angle = math.radians(values[2])
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        return values[:2], rotation

    def _observe_pose(self, ego, stamp, position):
        if self.last_pose is not None:
            dt = max(0., stamp-self.last_pose_stamp)
            speed = max(abs(float(ego.get('speed_kmh', 0.))), self.last_pose[2])/3.6
            turn = abs((float(ego['heading_deg'])-self.last_pose[1]+180.) % 360.-180.)
            # Ordinary steering and lane changes preserve world evidence.
            # A localization discontinuity invalidates its world-frame anchor.
            if (np.linalg.norm(position-self.last_pose[0]) > max(5., 2.*speed*dt+1.) or
                    turn > max(45., 160.*dt)):
                self.cells.clear()
        self.last_pose = (position.copy(), float(ego['heading_deg']), abs(float(ego.get('speed_kmh', 0.))))
        self.last_pose_stamp = stamp

    def _expire(self, stamp):
        self.cells = {key: row for key, row in self.cells.items()
                      if stamp-row[0] <= self.max_age_s}

    @staticmethod
    def _static_mask(world, times, stamp, moving_objects):
        keep = np.ones(len(world), dtype=bool)
        for obj in moving_objects:
            if len(obj) < 11 or obj[4] not in ('npc', 'pedestrian'):
                continue
            if obj[4] == 'npc' and (len(obj) <= 14 or not obj[14]):
                continue
            numeric = np.asarray([*obj[:2], *obj[5:11]], float)
            threshold = .5 if obj[4] == 'npc' else .3
            if (not np.isfinite(numeric).all() or obj[5] <= threshold or
                    np.linalg.norm(numeric[-2:]) <= threshold or min(obj[6:8]) <= 0.):
                continue
            metadata = obj[12] if len(obj) > 12 and isinstance(obj[12], dict) else {}
            source = metadata.get('source_stamp', metadata.get('stamp', stamp))
            age = metadata.get('age_s', obj[15] if len(obj) > 15 else 0.)
            if not 0. <= stamp-float(source) <= .3 or not 0. <= float(age) <= .3:
                continue
            angle = math.radians(float(obj[8]))
            forward = np.array([math.cos(angle), math.sin(angle)])
            left = np.array([-forward[1], forward[0]])
            centre = np.asarray(obj[:2], float)
            velocity = np.asarray(obj[9:11], float)
            # Remove the corresponding measured footprint at its original
            # source time as well as the body's current measured footprint.
            previous = centre-(stamp-times)[:, None]*velocity
            for delta in (world-centre, world-previous):
                occupied = ((abs(delta@forward) <= float(obj[6])/2+.35) &
                            (abs(delta@left) <= float(obj[7])/2+.25))
                keep &= ~occupied
        return keep

    def _bound(self, ego, position, rotation):
        if len(self.cells) <= self.max_points:
            return
        keys = list(self.cells)
        world = np.array([self.cells[key][1] for key in keys])
        local = (world-position)@rotation
        # Preserve spatial coverage before filling finer cells. Dense wall
        # returns must not evict the handful of far, low barrier observations.
        coarse = np.floor(local).astype(np.int64)
        _, representatives = np.unique(coarse, axis=0, return_index=True)
        distance = np.linalg.norm(local, axis=1)
        representatives = representatives[np.argsort(distance[representatives], kind='stable')]
        order = np.argsort(distance, kind='stable')
        represented = np.zeros(len(keys), dtype=bool)
        represented[representatives] = True
        remaining = order[~represented[order]]
        selected = np.r_[representatives, remaining][:self.max_points]
        self.cells = {keys[index]: self.cells[keys[index]] for index in selected}

    def update(self, points, ego, source_stamp, moving_objects=()):
        """Add one scan without confirmation delay or refreshing duplicate scans."""
        stamp = float(source_stamp)
        position, rotation = self._pose(ego, stamp)
        points = np.asarray(points, float).reshape(-1, 2)
        points = points[np.isfinite(points).all(axis=1)]
        points = points[(points[:, 0] >= -3.) & (points[:, 0] <= 70.) & (abs(points[:, 1]) <= 18.)]
        with self.lock:
            if self.last_scan is not None and stamp < self.last_scan:
                self.reset()
            self._observe_pose(ego, stamp, position)
            self._expire(stamp)
            if stamp != self.last_scan:
                world = points@rotation.T+position
                times = np.full(len(world), stamp)
                keep = self._static_mask(world, times, stamp, moving_objects)
                world = world[keep]
                keys, indices = np.unique(np.floor(world/self.voxel_m).astype(np.int64),
                                          axis=0, return_index=True)
                for key, index in zip(keys, indices):
                    self.cells[tuple(key)] = (stamp, world[index].copy())
                self.last_scan = stamp
            if self.cells and moving_objects:
                keys = list(self.cells)
                times = np.array([self.cells[key][0] for key in keys])
                world = np.array([self.cells[key][1] for key in keys])
                keep = self._static_mask(world, times, stamp, moving_objects)
                self.cells = {key: self.cells[key] for key, retain in zip(keys, keep) if retain}
            self._bound(ego, position, rotation)
            # Query clocks may be slightly newer than the next arriving scan.
            return self._snapshot(ego, stamp, position, rotation)

    def _snapshot(self, ego, stamp, position, rotation, exclude_stamp=None):
        self._expire(stamp)
        if not self.cells:
            return np.empty((0, 2))
        world = np.array([row[1] for row in self.cells.values()
                          if stamp-row[0] >= -.12 and row[0] != exclude_stamp])
        if not len(world):
            return np.empty((0, 2))
        local = (world-position)@rotation
        keep = (local[:, 0] >= -3.) & (np.linalg.norm(local, axis=1) <= 80.)
        return local[keep]

    def snapshot(self, ego, source_stamp, exclude_stamp=None):
        """Express unexpired measured evidence in the current vehicle frame."""
        stamp = float(source_stamp)
        position, rotation = self._pose(ego, stamp)
        with self.lock:
            previous = max(self.last_query or 0., self.last_scan or 0.)
            if stamp < previous-.12:
                self.reset()
            self.last_query = stamp
            self._observe_pose(ego, stamp, position)
            return self._snapshot(ego, stamp, position, rotation, exclude_stamp)


def update_approach_points(runtime, scan, points, ego, moving_objects=()):
    """Adapt measured scan points to the current portal approach control frame.

    Existing callers without a scan/pose receiver interface retain their own
    validation. Real sensor interfaces must satisfy both source and receipt
    freshness; rejected measurements never become an empty, apparently safe scan.
    """
    active = bool(getattr(runtime, '_transition_entry_obstacles', False))
    memory = getattr(runtime, '_portal_approach_memory', None)
    status = dict(active=active, valid=None, reason='inactive', quality='inactive',
                  source_stamp=None, query_stamp=None, source_count=0,
                  retained_count=0, max_age_s=1.5)
    runtime._portal_memory_status = status
    if not active:
        if memory is not None:
            memory.reset()
        return points
    receiver = getattr(runtime, 'ego_receiver', None)
    if (not isinstance(scan, dict) or 'stamp' not in scan or 'age_s' not in scan or
            not callable(getattr(receiver, 'at_stamp', None))):
        status.update(reason='legacy interface', quality='legacy unchecked')
        return points
    if memory is None:
        memory = runtime._portal_approach_memory = TunnelApproachMemory()
    status['max_age_s'] = memory.max_age_s

    def reject(reason):
        status.update(valid=False, reason=reason, quality='rejected',
                      retained_count=len(memory.cells))
        return None

    try:
        source = float(scan['stamp'])
        query = float(ego['stamp'])
        scan_age = float(scan['age_s'])
        query_age = float(ego.get('age_s', 999.))
        status.update(source_stamp=source, query_stamp=query, scan_age_s=scan_age,
                      query_age_s=query_age, source_count=len(points))
        if not all(map(math.isfinite, (source, query, scan_age, query_age))):
            return reject('nonfinite sensor time')
        if source <= 0. or query <= 0.:
            return reject('invalid sensor source time')
        if scan.get('xyz') is None or scan.get('valid', True) is False:
            return reject('invalid scan')
        if not 0. <= scan_age <= .6:
            return reject('stale scan receipt')
        if not 0. <= query_age <= .25:
            return reject('stale query pose')
        if not -.12 <= query-source <= .6:
            return reject('scan query source time mismatch')
        raw = np.asarray(points, float).reshape(-1, 2)
        raw = raw[np.isfinite(raw).all(axis=1)]
        with memory.lock:
            latest = max(memory.last_scan or 0., memory.last_query or 0.)
            # A bounded out-of-order callback must not rewind shared memory.
            # A source/query clock reset beyond the valid scan window still
            # invalidates every previous world-frame measurement and pose.
            if source < latest-.6 and query < latest-.6:
                memory.reset()
            late_scan = memory.last_scan is not None and source < memory.last_scan
            scan_ego = memory.source_poses.get(source)
            if scan_ego is None:
                scan_ego = receiver.at_stamp(source)
            if scan_ego is None:
                return reject('missing scan source pose')
            scan_position, scan_rotation = memory._pose(scan_ego, source)
            query_position, query_rotation = memory._pose(ego, query)
            if not late_scan and memory.last_scan != source:
                memory.update(raw, scan_ego, source, moving_objects)
            memory.source_poses[source] = dict(scan_ego)
            if len(memory.source_poses) > 16:
                del memory.source_poses[next(iter(memory.source_poses))]
            if memory.last_query is not None and query < memory.last_query:
                remembered = memory._snapshot(ego, query, query_position, query_rotation, source)
            else:
                remembered = memory.snapshot(ego, query, exclude_stamp=source)
            # Full current measurements are mandatory for the body check:
            # only retained evidence is voxelized, cropped, and bounded.
            current_raw = (raw@scan_rotation.T+scan_position-query_position)@query_rotation
            current = np.vstack((current_raw, remembered))
    except (ValueError, TypeError, KeyError, OverflowError) as error:
        return reject(str(error))
    status.update(valid=True, reason='fresh measured evidence', quality='measured',
                  retained_count=len(memory.cells), remembered_count=len(remembered),
                  output_count=len(current), out_of_order=late_scan)
    return current
