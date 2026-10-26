"""Measured, time-bounded MORAI IMU attitude for a LiDAR source frame."""
import math

import numpy as np

MAX_IMU_DELTA_S = .006
MAX_MEASURED_TILT_RATE_RAD_S = .1
ATTITUDE_RATE_BOUND_RAD_S = .2
MAX_NEIGHBOUR_SPAN_S = .2


def _record_attitude(record, *, selected):
    q = np.asarray(record['quaternion'], dtype=float)
    covariance = np.asarray(record['covariance'], dtype=float)
    gyro = np.asarray(record['angular_velocity'], dtype=float)
    if (record.get('measured') is not True or record.get('src') == 'file' or
            record.get('frame_id') != 'imu_link' or q.shape != (4,) or
            covariance.shape != (9,) or gyro.shape != (3,) or
            not np.isfinite(q).all() or not np.isfinite(covariance).all() or
            not np.isfinite(gyro).all() or np.any(covariance[[0, 4, 8]] < 0.) or
            not .99 <= np.linalg.norm(q) <= 1.01 or
            (selected and np.linalg.norm(gyro[:2]) > MAX_MEASURED_TILT_RATE_RAD_S)):
        return None
    return q/np.linalg.norm(q), covariance, gyro


def _same_stamp_attitude(records, stamp, *, selected):
    reference = None
    for record in records:
        if record['stamp'] != stamp:
            continue
        value = _record_attitude(record, selected=selected)
        if value is None:
            return None
        if reference is not None and (
                abs(float(np.dot(reference[0], value[0]))) < 1.-1e-12 or
                not np.array_equal(reference[1], value[1]) or
                not np.array_equal(reference[2], value[2])):
            return None
        reference = value
    return reference


def measured_imu_at(samples, stamp):
    """Keep the nearest actual header; never interpolate attitude.

    The fixed bridge covariance is not measured accuracy. A narrow source-time
    window and bounded attitude-rate operating domain define the additional
    height uncertainty applied by the mapped-road classifier. Unsupported
    timing, duplicate revisions or motion retain the original raw filter.
    """
    try:
        stamp = float(stamp)
        records = sorted(samples, key=lambda r: r['stamp'])
        times = np.asarray([r['stamp'] for r in records], dtype=float)
        if (not math.isfinite(stamp) or stamp <= 0. or not len(times) or
                not np.isfinite(times).all() or np.any(times <= 0.)):
            return None
        nearest = int(np.argmin(np.abs(times-stamp)))
        source_stamp = float(times[nearest])
        if abs(source_stamp-stamp) > MAX_IMU_DELTA_S+1e-9:
            return None
        reference = _same_stamp_attitude(records, source_stamp, selected=True)
        if reference is None:
            return None
        before = [r for r in records if r['stamp'] < source_stamp]
        after = [r for r in records if r['stamp'] > source_stamp]
        neighbours = ([before[-1]] if before else [])+([after[0]] if after else [])
        supported_neighbours = []
        rates = []
        for record in neighbours:
            span = abs(record['stamp']-source_stamp)
            if not 0. < span <= MAX_NEIGHBOUR_SPAN_S:
                continue
            # The selected sample owns the gyro operating limit. Neighbours
            # establish a measured quaternion rate, not a second source attitude
            # at the scan time. Every revision must still agree and be valid.
            value = _same_stamp_attitude(records, record['stamp'], selected=False)
            if value is None:
                return None
            angle = 2.*math.acos(np.clip(abs(float(np.dot(reference[0], value[0]))), 0., 1.))
            rate = angle/span
            if rate > ATTITUDE_RATE_BOUND_RAD_S:
                return None
            supported_neighbours.append(float(record['stamp']))
            rates.append(rate)
        if not supported_neighbours:
            return None
        return dict(imu_quaternion=reference[0].tolist(), imu_stamp=source_stamp,
                    imu_measured=True, imu_source_stamps=[source_stamp],
                    imu_neighbour_stamps=supported_neighbours,
                    imu_orientation_covariance=reference[1].tolist(),
                    imu_angular_velocity=reference[2].tolist(),
                    imu_adjacent_rate_rad_s=max(rates, default=0.),
                    imu_rate_bound_rad_s=ATTITUDE_RATE_BOUND_RAD_S)
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
