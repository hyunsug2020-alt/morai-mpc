# LIO-SAM odometry experiment

MORAI sensor connections and LIO-SAM run persistently in `morai.launch`.
The experiment launch starts only the validation GUI/CSV logger, so closing or
restarting the GUI does not disconnect MORAI sensors.

## Run

Terminal 1: start the persistent MORAI connection and LIO-SAM.

```bash
source /opt/ros/noetic/setup.bash
source /home/coss/catkin_ws/devel/setup.bash
roslaunch morai_launch morai.launch start_localization:=false
```

Leave terminal 1 running. Terminal 2: start or restart only the measurement.

```bash
source /opt/ros/noetic/setup.bash
source /home/coss/catkin_ws/devel/setup.bash
roslaunch morai_control lio_odometry_experiment.launch
```

For CSV-only operation:

```bash
roslaunch morai_control lio_odometry_experiment.launch gui:=false
```

The default result location is:

```text
/home/coss/catkin_ws/src/morai_control/logs/
```

Each run creates a timestamped CSV and a matching `_summary.json`.

## Interpretation

- `position_error_m`: total 2D position error after initial SE(2) alignment.
- `along_track_error_m`: error in the vehicle's forward direction.
- `cross_track_error_m`: error perpendicular to the vehicle heading.
- `yaw_error_deg`: aligned LIO-SAM yaw minus MORAI ground-truth yaw.
- `lio_degenerate`: LIO-SAM's incremental odometry degeneracy flag.
- `gps_shadow`: no valid GPS message within the configured timeout.
- `status`: combined diagnostic flags for the sample.

The LIO pose is published at the LiDAR origin. The logger first converts it to
`base_link` using `lidar_x` and `lidar_y`, then estimates a fixed SE(2)
alignment from the first `alignment_samples` synchronized measurements.
