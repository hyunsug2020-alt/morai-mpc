"""LiDAR cloud decoding, ego footprint exclusion and supported road planes."""
import numpy as np

_DT = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16, 5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def outside_ego_footprint(points):
    """Exclude vehicle/housing returns without masking the front collision zone.

    Road/highway LiDAR (LidarCorridor). It still masks 0.84-2.37 m behind the rear
    axle, which lies behind the real rear bumper (the pose is the rear axle,
    ego_body): the tunnel uses ego_body.self_return_mask instead."""
    p = np.asarray(points)
    return (np.abs(p[:, 0]) > 2.3175 + .05) | (np.abs(p[:, 1]) > .946 + .05)


def fit_road_plane(points, fallback_z):
    """Fit supported road slope from low returns, retaining the flat fallback.

    Equal weighting of ground cells prevents a dense vertical object face from
    becoming the road. Fits must explain the low returns as well as span both
    axes; sparse scans and unsupported surfaces retain the previous filter.
    """
    p = np.asarray(points, dtype=float)
    road = p[(p[:, 0] > 2.) & (p[:, 0] < 15.) & (np.abs(p[:, 1]) < 4.)]
    fallback = np.array([0., 0., float(fallback_z)])
    if len(road) < 50:
        return fallback
    cells = np.floor(road[:, :2]).astype(np.int32)
    _, owners = np.unique(cells, axis=0, return_inverse=True)
    anchors = []
    for owner in np.unique(owners):
        member = road[owners == owner]
        if len(member) >= 3:
            anchors.append(member[int(np.argmin(member[:, 2]))])
    if len(anchors) < 12:
        return fallback
    anchors = np.asarray(anchors)
    design = np.column_stack((anchors[:, :2], np.ones(len(anchors))))
    low = anchors[:, 2] <= np.percentile(anchors[:, 2], 25) + .05
    rng = np.random.RandomState(7)
    best, best_count = None, 0
    for _ in range(64):
        chosen = rng.choice(len(anchors), 3, replace=False)
        try:
            plane = np.linalg.solve(design[chosen], anchors[chosen, 2])
        except np.linalg.LinAlgError:
            continue
        if not np.isfinite(plane).all() or np.max(np.abs(plane[:2])) > .2:
            continue
        support = np.abs(design @ plane - anchors[:, 2]) <= .08
        if support[low].mean() < .65 or support.sum() < max(12, len(anchors) * .35):
            continue
        if np.ptp(anchors[support, 0]) < 4. or np.ptp(anchors[support, 1]) < 2.:
            continue
        if support.sum() > best_count:
            best, best_count = support, int(support.sum())
    if best is None:
        return fallback
    plane, _, rank, _ = np.linalg.lstsq(design[best], anchors[best, 2], rcond=None)
    if rank < 3 or np.max(np.abs(plane[:2])) > .2:
        return fallback
    return plane


def cloud_xyz(msg):
    """PointCloud2 -> (N,3) float32 without the slow python generator (velodyne: x,y,z,intensity float32, ring uint16, time)."""
    names, formats, offsets = [], [], []
    for f in msg.fields:
        names.append(f.name); formats.append(_DT[f.datatype]); offsets.append(f.offset)
    dt = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step})
    arr = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    xyz = np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]
