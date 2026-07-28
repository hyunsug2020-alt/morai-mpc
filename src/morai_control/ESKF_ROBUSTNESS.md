# MORAI ESKF Robustness Review

## Decision

The downloaded `eskf_localization_share_20260721.tar.gz` was reviewed but was
not installed as the production localizer. Its 15-state position, velocity,
attitude, accelerometer-bias, and gyro-bias model is a better starting point
than the former five-state filter, but the package has production blockers:

- Initialization depends on MORAI ground truth from `/Ego_topic`.
- GPS and IMU lever arms from `morai_sensor.json` are not compensated.
- GPS course yaw is derived directly from successive noisy positions.
- GPS quality, non-finite data, persistent faults, and IMU spikes are not
  handled.
- The GPS Mahalanobis gate is loose and there is no soft robust weighting.
- Error-state attitude injection has no reset Jacobian.
- It requires `pyproj`, which is not installed in the current runtime.

The production node therefore keeps the existing MORAI planar interface and
uses an eight-state error-state model:

`[imu_x, imu_y, imu_vx, imu_vy, yaw, accel_bias_x, accel_bias_y, gyro_bias_z]`

This is intentional for the mostly planar K-City vehicle task. A full 15-state
model should only replace it after roll, pitch, gravity, altitude, frame
conventions, and observability are validated using recorded sensor data.

## Added Robustness

- Accelerometer and gyro biases are estimated with random-walk process noise.
- GPS position, finite-difference velocity, and IMU yaw use normalized
  innovation squared (NIS) tests.
- Startup requires spatial consensus from multiple GPS samples, preventing one
  bad first fix from defining the map origin.
- Measurements between the soft and hard gates receive reduced weight;
  measurements beyond the hard gate are rejected.
- Accepted innovations adapt bounded measurement covariance scales.
- GPS velocity covariance accounts for GPS position variance and sample time.
- A Hampel-style window suppresses isolated accelerometer and gyro spikes.
- The vehicle non-holonomic constraint limits lateral dead-reckoning drift.
- Zero-velocity updates constrain drift while the vehicle is stationary.
- GPS and IMU lever arms are loaded from the MORAI sensor JSON.
- `/eskf/diagnostics` reports mode, covariance, NIS, estimated biases, adaptive
  scales, and accepted/rejected measurement counters.
- Delayed GPS and out-of-order IMU messages are rejected instead of corrupting
  the current state.
- LIO-SAM is aligned to the MORAI map while GPS is healthy and supplies
  conservative pose corrections only after GPS has been unavailable for one
  second.

Parameters are in `config/eskf_robust.yaml`. The noise test is:

```bash
rosrun morai_control validate_eskf_noise.py --trials 10
```

The deterministic 10-trial test result for the combined case was:

| Metric | Former filter | Robust ESKF |
|---|---:|---:|
| Position RMSE | 8.768 m | 0.994 m |
| Position p95 | 22.888 m | 2.265 m |
| Yaw RMSE | 0.734 deg | 0.346 deg |

The combined case includes 1.5 m GPS noise, 18-35 m GPS jumps, 15 s and 10 s
GPS outages, constant IMU bias, Gaussian IMU noise, and random IMU spikes.
These are simulation results, not a substitute for a rosbag test using the
competition sensor settings.

The GPS-denied SLAM integration test is:

```bash
rosrun morai_control validate_eskf_slam.py --trials 10
```

It includes two 45-second GPS outages, GPS outliers, IMU bias and spikes, two
five-second SLAM outages, two five-second LiDAR-degenerate intervals, local
SLAM drift, arbitrary local-frame rotation/translation, and SLAM pose outliers.

| GPS-denied metric | ESKF only | LIO-aided ESKF |
|---|---:|---:|
| Full trajectory RMSE | 10.070 m | 0.799 m |
| GPS-outage p95 | 29.948 m | 1.598 m |
| Maximum error | - | 2.597 m |

## LIO-SAM Integration

ESKF and LIO-SAM are complementary, but directly treating `/eskf/odom` as an
independent LIO-SAM GPS factor is not statistically correct. Both estimators
use the same IMU, so their errors are correlated and naive fusion double-counts
IMU information.

The implemented architecture is one-way and conditional:

1. LIO-SAM continues to consume raw IMU and LiDAR for deskewing and relative
   motion.
2. ESKF continues to own GPS fault rejection and the global MORAI map frame.
3. While GPS is healthy, `/lio_sam/mapping/odometry` is aligned to the ESKF
   global pose with an SE(2) transform after converting the LIO-SAM LiDAR
   origin to `base_link` using the `morai_sensor.json` LiDAR lever arm.
4. While GPS is healthy, SLAM is not fed back into ESKF. This avoids repeatedly
   counting the common IMU information.
5. After GPS loss, non-degenerate SLAM pose updates ESKF with covariance floors,
   distance-based drift growth, correlation inflation, and NIS gates.
6. SLAM dropouts, degeneracy flags, stale timestamps, and isolated frame jumps
   fall back to IMU/NHC dead reckoning.
7. A persistent LIO frame change such as loop closure must agree for three
   consecutive samples before the alignment is reset.

This combination can bound ESKF drift during GPS outages and can give LIO-SAM
global position in feature-poor areas. It cannot repair incorrect LiDAR timing,
extrinsics, sparse geometry, or moving-object contamination. A future fully
tightly coupled implementation should move raw GNSS, IMU preintegration, and
LiDAR factors into one graph instead of fusing completed estimates.

## Primary References

- J. Sola, "Quaternion kinematics for the error-state Kalman filter":
  https://arxiv.org/abs/1711.02508
- A. H. Mohamed and K. P. Schwarz, "Innovation-based Adaptive Kalman Filter
  for INS/GPS":
  https://www.ion.org/publications/abstract.cfm?articleID=1263
- H. Fang et al., "Robust Extended Kalman Filtering for Systems with
  Measurement Outliers":
  https://arxiv.org/abs/1904.00335
- "A Robust Adaptive Extended Kalman Filter ... in GNSS/INS Vehicle
  Navigation":
  https://www.mdpi.com/2072-4292/15/17/4125
- "Required Lever Arm Accuracy of Non-Holonomic Constraint for Land Vehicle
  Navigation":
  https://doi.org/10.1109/TVT.2020.2995076
- T. Shan et al., "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via
  Smoothing and Mapping":
  https://arxiv.org/abs/2007.00258
- "FGO-GIL: Factor Graph Optimization-Based GNSS RTK/INS/LiDAR Tightly
  Coupled Integration":
  https://doi.org/10.1109/JSEN.2023.3278723
- "Split Covariance Intersection with Correlated Components for Distributed
  Estimation":
  https://arxiv.org/abs/2403.03543
