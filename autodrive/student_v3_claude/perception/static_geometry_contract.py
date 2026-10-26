"""Atomic provenance for an unchanged, measured fusion obstacle publication.

The versioned ROS name attests to the producer's cluster/padding checks. The
receiver restores the ordinary name and binds that evidence to this exact raw
row and physical source timestamp. Stored estimates cannot normalize a new row;
their bounded history only supports a fresh check of the current margin deficit.
"""
import math

import numpy as np

from perception.obstacle_geometry import (FUSION_OBSTACLE_MIN_DIMENSION_M,
    FUSION_OBSTACLE_DIMENSION_PADDING_M)
from perception.stationary_semantics import metadata


CONTRACT_NAME = 'fusion_obstacle_padded_v1'
CANONICAL_NAME = 'fusion_obstacle'
PROOF_KEY = 'producer_static_geometry'
STORED_PROOF_KEY = 'stored_static_geometry'
MIN_SOURCE_POINTS = 4
MAX_SOURCE_AGE_S = .3
MAX_STORED_SOURCE_AGE_S = 3.
SOURCE_FUTURE_TOLERANCE_S = .05
_TOL = 1e-6


def canonical_static_name(name):
    return CANONICAL_NAME if name == CONTRACT_NAME else name


def _vector(value, length):
    result = np.asarray(value, float)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError('invalid geometry vector')
    return result


def _uid(value):
    result = int(value)
    if isinstance(value, bool) or result != value or not 0 <= result < 2**31:
        raise ValueError('invalid source identity')
    return result


def _signature(uid, kind, position_xy, size_xy, heading_deg, velocity_xy,
               speed_mps, height_m):
    uid = _uid(uid)
    position, size, velocity = (_vector(value, 2) for value in
                               (position_xy, size_xy, velocity_xy))
    heading, speed, height = map(float, (heading_deg, speed_mps, height_m))
    if (kind != 'obstacle' or not np.isfinite([heading, speed, height]).all()
            or speed != 0. or np.any(velocity != 0.) or height <= 0.
            or np.any(size < FUSION_OBSTACLE_MIN_DIMENSION_M+
                       FUSION_OBSTACLE_DIMENSION_PADDING_M-_TOL)):
        raise ValueError('not a stationary padded generic obstacle')
    return (uid, kind, tuple(position), tuple(size), heading, tuple(velocity), speed, height)


def _same_signature(a, b):
    return (a[:2] == b[:2] and np.allclose(
        np.r_[a[2], a[3], a[4], a[5], a[6:]],
        np.r_[b[2], b[3], b[4], b[5], b[6:]], rtol=0., atol=_TOL))


def producer_geometry_name(detection, *, name=CANONICAL_NAME, uid, kind,
        position_xy, size_xy, heading_deg, velocity_xy, speed_mps, height_m,
        alignment, timing_ready):
    """Tag only the unmodified current zero-motion raw-cluster publication."""
    try:
        if (name != CANONICAL_NAME or timing_ready is not True
                or detection.get('kind') != 'obstacle' or detection.get('moving') is not False
                or detection.get('merged_part_ids')
                or detection.get('publish_geometry') is not None
                or detection.get('publication_geometry') is not None
                or alignment.get('source_pose_valid') is not True
                or alignment.get('legacy_fallback') is not False
                or alignment.get('mode') != 'measured_pose'):
            return name
        evidence = detection['_static_geometry_evidence']
        count = evidence['points_n']
        if (isinstance(count, bool) or int(count) != count or count < MIN_SOURCE_POINTS
                or detection['n'] != count):
            return name
        stamp = float(evidence['source_stamp'])
        if (not math.isfinite(stamp) or stamp <= 0.
                or float(alignment['source_stamp']) != stamp):
            return name
        physical = _vector(evidence['physical_size_m'], 2)
        source_size = _vector(evidence['size_xy'], 2)
        if (np.any(physical < FUSION_OBSTACLE_MIN_DIMENSION_M-_TOL)
                or not np.allclose(source_size, physical+
                    FUSION_OBSTACLE_DIMENSION_PADDING_M, rtol=0., atol=_TOL)):
            return name
        final = _signature(uid, kind, position_xy, size_xy, heading_deg,
                           velocity_xy, speed_mps, height_m)
        measured = _signature(detection['uid'], detection['kind'],
            evidence['position_xy'], source_size, math.degrees(float(evidence['yaw_rad'])),
            detection['vel'], float(np.linalg.norm(detection['vel'])), evidence['height_m'])
        current = _signature(detection['uid'], detection['kind'], detection['pos'],
            detection['size'], math.degrees(float(detection['yaw'])), detection['vel'],
            float(np.linalg.norm(detection['vel'])), detection['height'])
        if _same_signature(measured, current) and _same_signature(measured, final):
            return CONTRACT_NAME
    except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
        pass
    return name


def decode_static_geometry(name, *, uid, kind, position_xy, size_xy, heading_deg,
        velocity_xy, speed_mps, height_m, source_stamp, measured_stamp):
    """Decode a fresh versioned ROS row; file fallbacks never call this helper."""
    try:
        if name != CONTRACT_NAME:
            return None
        stamp, measured = float(source_stamp), float(measured_stamp)
        if (not np.isfinite([stamp, measured]).all() or min(stamp, measured) <= 0.
                or not -SOURCE_FUTURE_TOLERANCE_S <= measured-stamp <= MAX_SOURCE_AGE_S):
            return None
        signature = _signature(uid, kind, position_xy, size_xy, heading_deg,
                               velocity_xy, speed_mps, height_m)
        physical = np.asarray(signature[3])-FUSION_OBSTACLE_DIMENSION_PADDING_M
        return dict(version=1, source='semantic_fusion', contract=CONTRACT_NAME,
            source_stamp=stamp, source_points_min=MIN_SOURCE_POINTS,
            source_padding_m=FUSION_OBSTACLE_DIMENSION_PADDING_M,
            physical_size_m=physical.tolist(), signature=signature)
    except (TypeError, ValueError, IndexError, OverflowError):
        return None


def validate_static_geometry(proof, row, measured_stamp):
    """Revalidate the original padded row, before any normalization or motion edit."""
    try:
        if (not isinstance(proof, dict) or proof.get('version') != 1
                or isinstance(proof.get('version'), bool)
                or proof.get('source') != 'semantic_fusion' or proof.get('contract') != CONTRACT_NAME
                or proof.get('source_points_min') != MIN_SOURCE_POINTS
                or isinstance(proof.get('source_points_min'), bool)
                or proof.get('source_padding_m') != FUSION_OBSTACLE_DIMENSION_PADDING_M
                or len(row) < 12 or row[3] not in (CANONICAL_NAME, CONTRACT_NAME)):
            return None
        bound = proof['signature']
        original = _signature(*bound)
        current = _signature(row[11], row[4], row[:2], row[6:8], row[8], row[9:11],
                             row[5], metadata(row)['height_m'])
        if (not _same_signature(original, current)
                or not math.isclose(float(row[2]), max(current[3]), rel_tol=0., abs_tol=_TOL)):
            return None
        valid = decode_static_geometry(CONTRACT_NAME, uid=original[0], kind=original[1],
            position_xy=original[2], size_xy=original[3], heading_deg=original[4],
            velocity_xy=original[5], speed_mps=original[6], height_m=original[7],
            source_stamp=proof['source_stamp'], measured_stamp=measured_stamp)
        physical = _vector(proof['physical_size_m'], 2)
        if valid is None or not np.allclose(physical, valid['physical_size_m'], rtol=0., atol=_TOL):
            return None
        return valid
    except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
        return None


def _normalized_static_geometry(row):
    """Reconstruct the bound source without granting it current freshness."""
    geometry = metadata(row)['road_static_geometry']
    if (row[3] != CANONICAL_NAME or geometry.get('version') != 1
            or isinstance(geometry.get('version'), bool)
            or geometry.get('source') != 'semantic_fusion'
            or geometry.get('producer_geometry') is not True
            or geometry.get('observed_points') != 0
            or isinstance(geometry.get('observed_points'), bool)
            or geometry.get('source_padding_m') != FUSION_OBSTACLE_DIMENSION_PADDING_M):
        raise ValueError('not a producer-only normalized physical estimate')
    physical, source, size = (_vector(value, 2) for value in
        (geometry['physical_size_m'], geometry['source_size_m'], row[6:8]))
    padding = float(geometry['applied_padding_m'])
    if (not math.isfinite(padding) or not 0. <= padding < FUSION_OBSTACLE_DIMENSION_PADDING_M
            or not np.allclose(size, physical+padding, rtol=0., atol=_TOL)
            or not math.isclose(float(row[2]), max(size), rel_tol=0., abs_tol=_TOL)):
        raise ValueError('changed normalized physical bounds')
    raw_size = metadata(row).get('raw_size_m')
    if raw_size is not None and not np.allclose(_vector(raw_size, 2), source, rtol=0., atol=_TOL):
        raise ValueError('changed raw source bounds')
    raw = list(row)
    raw[6:8], raw[2] = source, max(source)
    proof = metadata(row)[PROOF_KEY]
    # Validate the original binding at its original clock. The caller must
    # separately enforce either fresh issuance or bounded history age.
    original = validate_static_geometry(proof, raw, proof['source_stamp'])
    if original is None or not np.allclose(physical, original['physical_size_m'], rtol=0., atol=_TOL):
        raise ValueError('changed producer physical evidence')
    return raw, original, tuple(physical), tuple(size), padding


def mark_stored_static_geometry(row, measured_stamp):
    """Seal fresh producer-only physical bounds when stationary memory stores them.

    The immutable tuple retains the original source clock and dimensions. It
    supplies no new observation, source preference or normalization evidence.
    """
    try:
        raw, original, physical, size, padding = _normalized_static_geometry(row)
        if validate_static_geometry(metadata(row)[PROOF_KEY], raw, measured_stamp) is None:
            return None
        return (1, CONTRACT_NAME, original['source_stamp'], original['signature'],
                physical, size, padding)
    except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
        return None


def validate_stored_static_geometry(marker, row, measured_stamp):
    """Validate an already stored estimate for current-pose margin recovery only."""
    try:
        if (not isinstance(marker, tuple) or len(marker) != 7 or marker[0] != 1
                or isinstance(marker[0], bool) or marker[1] != CONTRACT_NAME
                or not isinstance(marker[3], tuple) or len(marker[3]) != 8
                or not all(isinstance(marker[3][i], tuple) for i in (2, 3, 5))
                or not isinstance(marker[4], tuple)
                or not isinstance(marker[5], tuple)):
            return None
        stamp, measured = float(marker[2]), float(measured_stamp)
        if (not np.isfinite([stamp, measured]).all() or min(stamp, measured) <= 0.
                or not 0. <= measured-stamp <= MAX_STORED_SOURCE_AGE_S):
            return None
        _, original, physical, size, padding = _normalized_static_geometry(row)
        if (stamp != original['source_stamp']
                or not _same_signature(_signature(*marker[3]), original['signature'])
                or not np.allclose(_vector(marker[4], 2), physical, rtol=0., atol=_TOL)
                or not np.allclose(_vector(marker[5], 2), size, rtol=0., atol=_TOL)
                or float(marker[6]) != padding):
            return None
        return marker
    except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
        return None
