# MORAI pedestrian PointPillars training

This experiment is independent from the existing Car checkpoint and output.
It detects `Pedestrian` and `Obstacle`; the Car model remains unchanged.

- Verified source frames: 30,000
- Train: 000000 through 023999 (24,000 frames)
- Validation: 024000 through 029999 (6,000 frames)
- GPU power limit: 250 W
- CPU turbo: disabled
- Batch size: 2
- Data-loader workers: 0
- Epochs: 40
- Watchdog stops at GPU 80 C, CPU 85 C, RAM below 3 GiB, disk below 5 GiB,
  or a kernel hardware error.

The epoch 39 Car checkpoint is read only to initialize compatible PointPillars
backbone weights. The class head is newly initialized for the two new classes,
and every new checkpoint is written below the separate
`pointpillar_pedestrian/morai_pedestrian_30k` output directory.
