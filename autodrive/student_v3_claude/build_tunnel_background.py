#!/usr/bin/env python3
"""Build tunnel_background.npz (M7 tunnel obstacle extraction) from an empty tunnel pass.

The empty recorded traversal is wall-aligned scan by scan with the same TunnelMap
alignment the runtime uses, every non-ground return is accumulated in the tunnel
frame (along, lateral, height above the G1 road plane) and three surveys are saved:

* a 0.1 m voxel occupancy grid (dilated by one voxel, bit-packed),
* the per-height wall-section profile of each side (how far wall + kerb reach
  inward, p99.5 per 5 cm height bin, 3-bin running max),
* the kerb face at bumper height (0.3-0.6 m, p99) of each side.

usage: python build_tunnel_background.py [--capture tunnel_scans.npz ...] [--out tunnel_background.npz]
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy import ndimage

from ego_body import self_return_mask
from lidar_corridor import fit_road_plane
from tunnel_map import BACKGROUND_FORMAT, TunnelMap, _dot, _rotation, road_plane
from tunnel_runtime import pose_at_stamp

HERE = Path(__file__).resolve().parent
DEFAULT_CAPTURES = [HERE.parents[1]/'data/autodrive/follow_runs/follow-20261002T152952Z-1309248/'
                    'tunnel_analysis/tunnel_scans.npz']
MOUNT = np.array([1.05, 0., 1.35])
ALONG_MIN, ALONG_MAX, LATERAL_MIN, LATERAL_MAX, HEIGHT_MAX = -40., 160., -6., 11., 4.5
VOXEL_M, PROFILE_BIN_M = .1, .05


def frames(path):
    """(points in vehicle frame, ego at scan stamp, LidarCorridor-style ground plane)."""
    with np.load(path) as data:
        ego, xyz, offsets, stamps = (data[key] for key in ('ego', 'xyz', 'offsets', 'stamps'))
    poses = sorted([dict(stamp=float(e[1]), x=float(e[2]), y=float(e[3]), heading_deg=float(e[4]),
                         speed_kmh=0., age_s=.01) for e in ego], key=lambda q: q['stamp'])
    times = np.array([q['stamp'] for q in poses])
    ground_z = 0.
    for i in range(len(stamps)):
        stamp = float(stamps[i, 1])
        k = int(np.searchsorted(times, stamp))
        pose = pose_at_stamp(poses[max(0, k-3):k+3], stamp)
        a, b = offsets[i:i+2]
        p = np.asarray(xyz[a:b], dtype=np.float32)
        p = (p[np.isfinite(p).all(1)]+MOUNT.astype(np.float32)).astype(float)
        near = p[(p[:, 0] > 2.) & (p[:, 0] < 15.) & (np.abs(p[:, 1]) < 3.)]
        if len(near) > 50:          # running value, as LidarCorridor.ingest
            ground_z = float(np.percentile(near[:, 2], 5))
        if pose is not None:
            yield p, pose, np.asarray(fit_road_plane(p, ground_z), dtype=float)


def accumulate(model, captures):
    out, used = [], 0
    for path in captures:
        for p, ego, plane0 in frames(path):
            delta = np.array([ego['x'], ego['y']])-model.origin
            lateral = delta @ model.normal+_dot(p[:, :2], _rotation(ego['heading_deg']).T @ model.normal)
            plane = road_plane(p, lateral, model.left, model.right, plane0)
            match = model.process(p, ego, plane)
            if not match['valid']:
                continue
            aligned = match['aligned_ego']
            rotation = _rotation(aligned['heading_deg'])
            delta = np.array([aligned['x'], aligned['y']])-model.origin
            along = delta @ model.axis+_dot(p[:, :2], rotation.T @ model.axis)
            lateral = delta @ model.normal+_dot(p[:, :2], rotation.T @ model.normal)
            height = p[:, 2]-(_dot(p[:, :2], plane[:2])+plane[2])
            body_free = self_return_mask(p)
            keep = body_free & (height > .05) & (height < HEIGHT_MAX)
            out.append(np.column_stack((along[keep], lateral[keep], height[keep])).astype(np.float32))
            used += 1
    if used < 20:
        raise ValueError(f'too few wall-aligned scans: {used}')
    return np.vstack(out), used


def survey(acc, model):
    a = acc[(acc[:, 2] > .1) & (acc[:, 2] < HEIGHT_MAX)]
    a = a[(a[:, 0] > ALONG_MIN) & (a[:, 0] < ALONG_MAX) & (a[:, 1] > LATERAL_MIN) & (a[:, 1] < LATERAL_MAX)]
    shape = (int((ALONG_MAX-ALONG_MIN)/VOXEL_M)+1, int((LATERAL_MAX-LATERAL_MIN)/VOXEL_M)+1,
             int(HEIGHT_MAX/VOXEL_M)+1)
    grid = np.zeros(shape, dtype=bool)
    grid[((a[:, 0]-ALONG_MIN)/VOXEL_M).astype(np.int32), ((a[:, 1]-LATERAL_MIN)/VOXEL_M).astype(np.int32),
         (a[:, 2]/VOXEL_M).astype(np.int32)] = True
    occupied = int(grid.sum())
    # +-1 voxel for pose and beam jitter.
    grid = ndimage.binary_dilation(grid, structure=np.ones((3, 3, 3), bool))
    bins = int(HEIGHT_MAX/PROFILE_BIN_M)+1
    mid = a[(a[:, 0] > model.start+3.) & (a[:, 0] < model.end-3.)]
    profiles, kerb = {}, {}
    for side, offset in ((1, model.left), (-1, model.right)):
        inward = (offset-mid[:, 1]) if side == 1 else (mid[:, 1]-offset)
        sel = (inward > -.3) & (inward < 1.)
        hb = (mid[sel, 2]/PROFILE_BIN_M).astype(int)
        values = inward[sel]
        profile = np.full(bins, np.nan)
        for b in range(bins):
            v = values[hb == b]
            if len(v) >= 30:
                profile[b] = np.percentile(v, 99.5)
        good = np.isfinite(profile)
        profile = np.interp(np.arange(bins), np.flatnonzero(good), profile[good])
        profile = ndimage.maximum_filter1d(profile, 3)
        # Below 0.3 m the band is polluted by road returns at the wall: kerb toe = 0.3 m value + 0.1.
        low = int(.3/PROFILE_BIN_M)
        profile[:low] = min(profile[:low].max(), profile[low]+.1)
        profiles[side] = profile
        face = sel & (mid[:, 2] > .3) & (mid[:, 2] < .6)
        kerb[side] = float(np.percentile(inward[face], 99))
    return grid, occupied, profiles, kerb


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--capture', action='append', type=Path, help='empty tunnel_scans.npz (repeatable)')
    parser.add_argument('--map', type=Path, default=HERE/'tunnel_wall_map.json')
    parser.add_argument('--out', type=Path, default=HERE/'tunnel_background.npz')
    args = parser.parse_args(argv)
    captures = args.capture or DEFAULT_CAPTURES
    payload = json.loads(args.map.read_text())
    model = TunnelMap.load(args.map, background=None)
    acc, used = accumulate(model, captures)
    grid, occupied, profiles, kerb = survey(acc, model)
    frame = list(payload['origin_xy'])+[payload[key] for key in (
        'axis_heading_deg', 'left_offset_m', 'right_offset_m', 'longitudinal_min_m', 'longitudinal_max_m')]
    np.savez_compressed(
        args.out, format=np.array(BACKGROUND_FORMAT), map_frame=np.asarray(frame, dtype=float),
        grid_bits=np.packbits(grid.ravel()), grid_shape=np.asarray(grid.shape, dtype=np.int64),
        voxel_m=VOXEL_M, along_min_m=ALONG_MIN, lateral_min_m=LATERAL_MIN, profile_bin_m=PROFILE_BIN_M,
        profile_left_m=profiles[1], profile_right_m=profiles[-1],
        kerb_face_left_m=kerb[1], kerb_face_right_m=kerb[-1],
        sources=np.array([str(c) for c in captures]), scans_used=used, points=len(acc))
    print(f'{args.out}: {used} scans, {len(acc)} returns, {occupied} voxels '
          f'({int(grid.sum())} dilated), kerb face L {kerb[1]:.3f} R {kerb[-1]:.3f} m, '
          f'{args.out.stat().st_size/1024:.0f} KiB')
    for side in (1, -1):
        print('  profile', 'L' if side == 1 else 'R', ' '.join(
            f'{b*PROFILE_BIN_M:.1f}:{v:.2f}' for b, v in enumerate(profiles[side]) if b % 4 == 0 and b*PROFILE_BIN_M < 3.6))


if __name__ == '__main__':
    main()
