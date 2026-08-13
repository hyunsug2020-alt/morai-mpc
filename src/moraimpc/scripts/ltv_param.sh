#!/usr/bin/env bash
set -euo pipefail

NS="${NS:-/path_follower_node}"

usage() {
  cat <<EOF
Usage:
  $0 show
  $0 load [yaml_file]
  $0 PARAM VALUE

Examples:
  $0 kappa_gain 1.35
  $0 max_steer_rate 65
  $0 curve_speed_alpha 16
  $0 curve_alat_max 2.4
  $0 target_vel 30
  $0 load

Namespace defaults to ${NS}. Override with:
  NS=/path_follower_node $0 PARAM VALUE
EOF
}

if [[ $# -lt 1 ]]; then
  usage
  exit 0
fi

case "$1" in
  show)
    rosparam get "${NS}"
    ;;
  load)
    file="${2:-$(rospack find moraimpc)/config/ltv_tuning.yaml}"
    rosparam load "${file}" "${NS}"
    echo "[ltv_param] loaded ${file} -> ${NS}"
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    if [[ $# -ne 2 ]]; then
      usage
      exit 2
    fi
    rosparam set "${NS}/$1" "$2"
    echo -n "[ltv_param] ${NS}/$1 = "
    rosparam get "${NS}/$1"
    ;;
esac
