#!/bin/bash
set -e
echo "================================================"
echo "  MORAI MPC (ROS2 Humble) - Ioniq 5 SUV"
echo "  전진: LTV-MPC  /  후진주차: RTI-NMPC"
echo "================================================"
source /opt/ros/humble/setup.bash
[ -f /home/moraimpc/install/setup.bash ] && \
    source /home/moraimpc/install/setup.bash && \
    echo "[OK] 워크스페이스 로드" || \
    echo "[WARN] colcon build 필요"
[ -n "$DISPLAY" ] && xhost +local:root 2>/dev/null || true
echo "ros2 launch moraimpc morai_full.launch.py"
exec "$@"
