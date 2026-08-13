#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ESKF_NODE="${WORKSPACE}/src/eskf/scripts/eskf_node.py"
VENV_DIR="${ESKF_VENV_DIR:-${WORKSPACE}/.venv-eskf}"
RESULTS_DIR="${ESKF_RESULTS_DIR:-/tmp/eskf_validation}"
STRESS_ITERATIONS="${1:-${ESKF_STRESS_ITERATIONS:-100}}"
STRESS_SEED="${2:-${ESKF_STRESS_SEED:-20260728}}"
NOISE_TRIALS="${ESKF_NOISE_TRIALS:-10}"
SLAM_TRIALS="${ESKF_SLAM_TRIALS:-10}"
ODOMETRY_TRIALS="${ESKF_ODOMETRY_TRIALS:-10}"

set +u
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi
if [[ -f "${WORKSPACE}/devel/setup.bash" ]]; then
  source "${WORKSPACE}/devel/setup.bash"
fi
set -u

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  /usr/bin/python3 -m venv \
    --without-pip \
    --system-site-packages \
    "${VENV_DIR}"
fi

PYTHON="${VENV_DIR}/bin/python"
mkdir -p "${RESULTS_DIR}"

"${PYTHON}" -c "import numpy, rospy"
"${PYTHON}" -m py_compile \
  "${ESKF_NODE}" \
  "${SCRIPT_DIR}/validate_eskf_noise.py" \
  "${SCRIPT_DIR}/validate_eskf_odometry.py" \
  "${SCRIPT_DIR}/validate_eskf_rosbag_faults.py" \
  "${SCRIPT_DIR}/validate_eskf_slam.py" \
  "${SCRIPT_DIR}/stress_eskf_slam.py"

echo "[1/5] IMU/GPS noise and outlier validation"
"${PYTHON}" "${SCRIPT_DIR}/validate_eskf_noise.py" \
  --trials "${NOISE_TRIALS}" \
  --seed "${STRESS_SEED}"

echo "[2/5] GPS-denied validated-odometry-aided validation"
"${PYTHON}" "${SCRIPT_DIR}/validate_eskf_odometry.py" \
  --trials "${ODOMETRY_TRIALS}" \
  --seed "${STRESS_SEED}"

echo "[3/5] GPS-denied SLAM-aided validation"
"${PYTHON}" "${SCRIPT_DIR}/validate_eskf_slam.py" \
  --trials "${SLAM_TRIALS}" \
  --seed "${STRESS_SEED}"

echo "[4/5] Known failure-seed regression validation"
for seed in \
  20260732 20260735 20260740 20260749 20260758 20260921 \
  20260830 20260832 20260878 20260884 20260891 20260894 \
  20260905 20260910 20260923 20260927; do
  "${PYTHON}" "${SCRIPT_DIR}/stress_eskf_slam.py" \
    --iterations 1 \
    --seed "${seed}" \
    --stop-on-failure \
    --progress-every 1 \
    --results "${RESULTS_DIR}/regression_${seed}.json"
done

echo "[5/5] Randomized stress validation"
"${PYTHON}" "${SCRIPT_DIR}/stress_eskf_slam.py" \
  --iterations "${STRESS_ITERATIONS}" \
  --seed "${STRESS_SEED}" \
  --stop-on-failure \
  --progress-every 10 \
  --results "${RESULTS_DIR}/stress_latest.json"

echo "All ESKF validations passed. Results: ${RESULTS_DIR}"
