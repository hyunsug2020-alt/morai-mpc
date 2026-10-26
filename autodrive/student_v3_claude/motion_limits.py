"""Longitudinal limits shared by motion forecasts and ROS target publishing."""

# Published target ramp. 1.2 m/s^2 kept the target near zero after brief stop
# pulses; forecasts must use the same permitted acceleration as the publisher.
LAUNCH_ACCEL_MPS2 = 2.5
