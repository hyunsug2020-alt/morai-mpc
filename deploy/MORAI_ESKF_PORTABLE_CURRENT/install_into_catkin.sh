#!/usr/bin/env bash

set -euo pipefail

bundle_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
target_ws="${1:-${HOME}/catkin_ws}"
target_src="${target_ws}/src"
backup_stamp="$(date +%Y%m%d_%H%M%S)"
backup_dir="${target_ws}/portable_backup_${backup_stamp}"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "오류: /opt/ros/noetic/setup.bash가 없음. ROS Noetic을 먼저 설치해야 함." >&2
  exit 1
fi

mkdir -p "${target_src}"

backup_and_copy() {
  local package_name="$1"
  local source_dir="${bundle_dir}/src/${package_name}"
  local destination="${target_src}/${package_name}"
  if [[ ! -d "${source_dir}" ]]; then
    echo "오류: 배포본에 ${package_name} 패키지가 없음" >&2
    exit 1
  fi
  if [[ -e "${destination}" ]]; then
    mkdir -p "${backup_dir}"
    local backup="${backup_dir}/${package_name}"
    echo "기존 ${package_name}을 ${backup}으로 백업함"
    mv "${destination}" "${backup}"
  fi
  cp -a "${source_dir}" "${destination}"
}

backup_and_copy eskf
backup_and_copy morai_eskf_runtime

if [[ -f "${target_src}/morai_msgs/msg/GPSMessage.msg" \
      && -f "${target_src}/morai_msgs/msg/EgoVehicleStatus.msg" ]]; then
  echo "기존 morai_msgs가 필요한 메시지를 제공하므로 유지함"
else
  backup_and_copy morai_msgs
fi

set +u
source /opt/ros/noetic/setup.bash
set -u

cd "${target_ws}"
# ROS Noetic catkin의 오래된 최상위 CMake 정책을 CMake 4.x에서도 허용함.
catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5

echo "설치와 빌드가 완료됐음"
echo "다음 명령을 실행하면 됨:"
echo "source ${target_ws}/devel/setup.bash"
echo "roslaunch morai_eskf_runtime morai_eskf.launch"
