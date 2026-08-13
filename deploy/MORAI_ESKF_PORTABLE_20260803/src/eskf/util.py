import math

import numpy as np


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def latlon_to_utm(latitude, longitude, zone):
    """Convert WGS84 latitude/longitude to UTM without an external dependency."""
    a = 6378137.0
    ecc_sq = 0.00669438
    k0 = 0.9996
    ecc_prime_sq = ecc_sq / (1.0 - ecc_sq)

    lat = math.radians(latitude)
    lon = math.radians(longitude)
    lon_origin = math.radians((zone - 1) * 6 - 180 + 3)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    tan_lat = math.tan(lat)
    n = a / math.sqrt(1.0 - ecc_sq * sin_lat * sin_lat)
    t = tan_lat * tan_lat
    c = ecc_prime_sq * cos_lat * cos_lat
    aa = cos_lat * (lon - lon_origin)

    m = a * (
        (1.0 - ecc_sq / 4.0 - 3.0 * ecc_sq**2 / 64.0
         - 5.0 * ecc_sq**3 / 256.0) * lat
        - (3.0 * ecc_sq / 8.0 + 3.0 * ecc_sq**2 / 32.0
           + 45.0 * ecc_sq**3 / 1024.0) * math.sin(2.0 * lat)
        + (15.0 * ecc_sq**2 / 256.0
           + 45.0 * ecc_sq**3 / 1024.0) * math.sin(4.0 * lat)
        - (35.0 * ecc_sq**3 / 3072.0) * math.sin(6.0 * lat)
    )

    easting = k0 * n * (
        aa + (1.0 - t + c) * aa**3 / 6.0
        + (5.0 - 18.0 * t + t**2 + 72.0 * c
           - 58.0 * ecc_prime_sq) * aa**5 / 120.0
    ) + 500000.0

    northing = k0 * (
        m + n * tan_lat * (
            aa**2 / 2.0
            + (5.0 - t + 9.0 * c + 4.0 * c**2) * aa**4 / 24.0
            + (61.0 - 58.0 * t + t**2 + 600.0 * c
               - 330.0 * ecc_prime_sq) * aa**6 / 720.0
        )
    )
    if latitude < 0.0:
        northing += 10000000.0
    return easting, northing


def robust_gps_velocity(samples, min_span=0.8, min_samples=5):
    """Estimate map-frame velocity from a robust fit over recent GPS fixes."""
    if len(samples) < min_samples:
        return None
    ordered = sorted(samples, key=lambda sample: sample[0])
    times = np.asarray([sample[0] for sample in ordered], dtype=float)
    positions = np.asarray([sample[1] for sample in ordered], dtype=float)
    if times[-1] - times[0] < min_span:
        return None

    centered_times = times - np.mean(times)
    denominator = float(centered_times @ centered_times)
    if denominator < 1e-6:
        return None
    position_mean = np.mean(positions, axis=0)
    velocity = centered_times @ (positions - position_mean) / denominator
    predicted = position_mean + np.outer(centered_times, velocity)
    residuals = np.linalg.norm(positions - predicted, axis=1)
    median = float(np.median(residuals))
    mad = float(np.median(np.abs(residuals - median)))
    threshold = max(1.0, median + 3.0 * 1.4826 * mad)
    inliers = residuals <= threshold
    if np.count_nonzero(inliers) >= min_samples:
        times = times[inliers]
        positions = positions[inliers]
        centered_times = times - np.mean(times)
        denominator = float(centered_times @ centered_times)
        position_mean = np.mean(positions, axis=0)
        velocity = (
            centered_times @ (positions - position_mean) / denominator)
        predicted = position_mean + np.outer(centered_times, velocity)
        residuals = np.linalg.norm(positions - predicted, axis=1)

    robust_residual_std = (
        float(np.median(residuals)) / math.sqrt(2.0 * math.log(2.0))
        if len(residuals) else 0.0)
    return velocity, denominator, robust_residual_std**2


