"""Compatibility import for LiDAR geometry, motion history and scan reception.

Alias the receiver module so patches such as ``lidar_corridor.fit_road_plane``
continue to affect the globals used by ``LidarCorridor.ingest``.
"""
import sys

from perception import lidar_receiver as _implementation

sys.modules[__name__] = _implementation
