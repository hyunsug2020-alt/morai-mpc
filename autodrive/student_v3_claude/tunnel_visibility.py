"""Bound tunnel planning by the angular reach of actual VLP16 returns."""
import math

import numpy as np

from ego_body import EGO_CENTRE_M, EGO_HALF_LENGTH_M, EGO_HALF_WIDTH_M, body_centres, self_return_mask


class ObservedLidarCoverage:
    """Measured horizontal ray support, with no extrapolation across missing sectors.

    XYZ, ground plane, laser origin and query paths must share one vehicle frame.
    Ground and vehicle-height returns provide support; overhead-only returns do
    not authorize travel. Collision checks remain separate and use every obstacle.
    """
    def __init__(self, xyz, ground_plane, origin_xy=(1.05, 0.), bin_deg=2.,
                 range_margin_m=.5, min_bin_samples=3):
        self.origin = np.asarray(origin_xy, dtype=float)
        self.bin_count = int(math.ceil(360. / float(bin_deg)))
        self.bin_width = 2. * math.pi / self.bin_count
        self.range_margin = float(range_margin_m)
        self.reach = np.zeros(self.bin_count, dtype=float)
        angles = -math.pi + (np.arange(self.bin_count) + .5) * self.bin_width
        self.rays = np.column_stack((np.cos(angles), np.sin(angles)))
        if xyz is None or ground_plane is None or self.origin.shape != (2,):
            return
        points, plane = np.asarray(xyz, dtype=float), np.asarray(ground_plane, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or plane.shape != (3,):
            return
        if not np.isfinite(plane).all() or not np.isfinite(self.origin).all():
            return
        points = points[np.isfinite(points).all(axis=1)]
        height = points[:, 2] - (points[:, 0]*plane[0] + points[:, 1]*plane[1] + plane[2])
        delta = points[:, :2] - self.origin
        radius = np.linalg.norm(delta, axis=1)
        # A radial cutoff also erases real nearby walls (e.g. a side wall
        # 2.3 m from the laser). Mask only the known vehicle footprint; measured
        # returns outside it must retain their actual, possibly short, reach.
        keep = (height >= -.25) & (height <= 2.5) & self_return_mask(points)
        delta, radius = delta[keep], radius[keep]
        if not len(radius):
            return
        angle = np.mod(np.arctan2(delta[:, 1], delta[:, 0]) + math.pi, 2.*math.pi)
        bins = np.floor(angle / self.bin_width).astype(int) % self.bin_count
        order = np.lexsort((radius, bins))
        bins, radius, angle = bins[order], radius[order], angle[order]
        starts = np.r_[0, np.flatnonzero(np.diff(bins)) + 1]
        ends = np.r_[starts[1:], len(bins)]
        counts = ends-starts
        # Multiple vertical channels at one azimuth do not cover a sector.
        spread = np.maximum.reduceat(angle, starts)-np.minimum.reduceat(angle, starts)
        supported = (counts >= min_bin_samples) & (spread >= .4*self.bin_width)
        # A lone distant return must not authorize an entire angular sector.
        rank = np.floor(.9*(counts-1)).astype(int)
        reach = radius[starts+rank]
        self.reach[bins[starts[supported]]] = np.maximum(0., reach[supported]-self.range_margin)

    def clear_distance(self, path, horizon, extra_margin=.25, *, exclude_initial_footprint=False,
                       lateral_room=None):
        """First path station whose whole vehicle footprint lacks ray support.

        Path samples are rear-axle positions; each footprint is the body centred
        EGO_CENTRE_M ahead along the path heading there (ego_body).

        lateral_room(centres, normals) -> (left, right): distance from each station centre
        to a physical boundary on each side (tunnel kerb faces). The footprint and its
        margins are not required to be seen beyond it: a kerb return ends every ray
        there, so a line 0.3 m from the kerb read 1 m of coverage with the walls and
        returns clear for 60 m (follow-20261010T173230Z 02:41:45-02:42:11)."""
        path = np.asarray(path, dtype=float)
        if path.ndim != 2 or path.shape[1] < 2 or len(path) < 3 or not np.isfinite(path).all():
            return 0.
        if not math.isfinite(horizon) or horizon <= 0.:
            return 0.
        path = path[:, :2]
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        unique, owners = np.unique(arc, return_index=True)
        if len(unique) < 3:
            return 0.
        path, arc = path[owners], unique
        limit = min(float(horizon), float(arc[-1]))
        sample = np.r_[np.arange(0., limit, .25), limit]
        if len(sample) < 3:
            return 0.
        centres = np.column_stack([np.interp(sample, arc, path[:, i]) for i in (0, 1)])
        tangent = np.gradient(centres, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        centres = body_centres(centres, tangent)
        origin_delta = self.origin - centres
        origin_local = np.column_stack((np.sum(origin_delta*tangent, axis=1),
                                        np.sum(origin_delta*normal, axis=1)))
        # Inflate by half a sector's transverse uncertainty, then intersect
        # every ray with the complete oriented rectangle, not only its corners.
        uncertainty = (np.linalg.norm(origin_delta, axis=1) + 2.6) * math.sin(self.bin_width / 2.)
        half = np.column_stack((np.full(len(sample), EGO_HALF_LENGTH_M + extra_margin) + uncertainty,
                                np.full(len(sample), EGO_HALF_WIDTH_M + extra_margin) + uncertainty))
        upper = half.copy()
        lower = -half
        if lateral_room is not None:
            left, right = lateral_room(centres, normal)
            # Rays end range_margin short of the boundary return (reach): stop the footprint
            # there too, never inside the body itself (a body that close stays unconfirmed).
            upper[:, 1] = np.minimum(half[:, 1], np.maximum(np.asarray(left, dtype=float)-self.range_margin,
                                                            EGO_HALF_WIDTH_M))
            lower[:, 1] = -np.minimum(half[:, 1], np.maximum(np.asarray(right, dtype=float)-self.range_margin,
                                                             EGO_HALF_WIDTH_M))
        # A circumscribed circle discards rays that cannot hit the footprint.
        # Most distant path stations use only a few forward sectors.
        diagonal = np.linalg.norm(half, axis=1)
        forward = -origin_delta[:, 0, None]*self.rays[:, 0] - origin_delta[:, 1, None]*self.rays[:, 1]
        sideways = -origin_delta[:, 0, None]*self.rays[:, 1] + origin_delta[:, 1, None]*self.rays[:, 0]
        candidates = ((forward >= 0.) & (np.abs(sideways) <= diagonal[:, None]))
        candidates |= np.linalg.norm(origin_delta, axis=1)[:, None] <= diagonal[:, None]
        rows, ray_ids = np.nonzero(candidates)
        chosen = self.rays[ray_ids]
        tangent_rays = tangent[rows, 0]*chosen[:, 0] + tangent[rows, 1]*chosen[:, 1]
        normal_rays = normal[rows, 0]*chosen[:, 0] + normal[rows, 1]*chosen[:, 1]
        local_rays = np.column_stack((tangent_rays, normal_rays))
        parallel = np.abs(local_rays) < 1e-9
        safe_rays = np.where(parallel, 1., local_rays)
        selected_lower, selected_upper, selected_origin = lower[rows], upper[rows], origin_local[rows]
        a = (selected_lower - selected_origin) / safe_rays
        b = (selected_upper - selected_origin) / safe_rays
        near, far = np.minimum(a, b), np.maximum(a, b)
        inside_parallel = (selected_origin >= selected_lower) & (selected_origin <= selected_upper)
        near = np.where(parallel, np.where(inside_parallel, -np.inf, np.inf), near)
        far = np.where(parallel, np.where(inside_parallel, np.inf, -np.inf), far)
        entering = np.maximum(np.max(near, axis=1), 0.)
        exiting = np.min(far, axis=1)
        required = exiting >= entering
        if exclude_initial_footprint and np.any(self.reach > 0.):
            # The current footprint is occupied by our vehicle, not unseen
            # road that the vehicle is about to enter. Require measured rays
            # for the newly swept portion; wall/obstacle checks still test the
            # full footprint, including its margins, at every station.
            # Scan coordinates already encode the actual vehicle pose: its
            # heading is zero here, regardless of the planned first tangent.
            # The body at the pose is centred EGO_CENTRE_M ahead of the rear axle.
            initial_origin = self.origin - [EGO_CENTRE_M, 0.]
            initial_uncertainty = (np.linalg.norm(initial_origin) + 2.6) * math.sin(self.bin_width / 2.)
            # Inflated at least as much as the station footprint it is compared with:
            # that centre lies ahead of the sensor too, and a station box a few mm
            # wider from its own sector inflation read its whole side, where the rear
            # sectors end, as newly swept (no coverage from the first station).
            inflation = np.maximum(initial_uncertainty, uncertainty[rows])[:, None]
            initial_half = np.array([EGO_HALF_LENGTH_M + extra_margin, EGO_HALF_WIDTH_M + extra_margin]) + inflation
            initial_rays = self.rays[ray_ids]
            parallel_initial = np.abs(initial_rays) < 1e-9
            initial_rays = np.where(parallel_initial, 1., initial_rays)
            initial_far = np.maximum((-initial_half-initial_origin)/initial_rays,
                                     (initial_half-initial_origin)/initial_rays)
            initial_far = np.where(parallel_initial,
                np.where(np.abs(initial_origin) <= initial_half, np.inf, -np.inf), initial_far)
            initial_exit = np.maximum(0., np.min(initial_far, axis=1))
            required &= exiting > initial_exit + 1e-6
        unavailable = required & ((self.reach[ray_ids] <= 0.) |
                                  (exiting > self.reach[ray_ids]))
        bad = np.flatnonzero(unavailable)
        if len(bad):
            station = rows[bad[0]]
            if exclude_initial_footprint:
                station = max(0, station-1)
            return float(sample[station])
        return limit

    def status(self):
        supported = self.reach > 0.
        return dict(supported_bins=int(supported.sum()), total_bins=self.bin_count,
                    bin_degrees=math.degrees(self.bin_width),
                    maximum_observed_reach_m=float(self.reach.max()))
