#!/usr/bin/env bash
set -Eeuo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo=$(cd -- "${script_dir}/../.." && pwd)
labels="${repo}/datasets/morai_lidar_pedestrian_far_v2_20260802/labels"
log_file="${repo}/logs/far_v2/morai_health_supervisor.log"
target=35000
stale_checks=0
previous=-1
mkdir -p "$(dirname -- "${log_file}")"

count_frames() {
  find "${labels}" -maxdepth 1 -type f -name '*.json' -printf '.' | wc -c
}

log() {
  printf '%s %s\n' "$(date --iso-8601=seconds)" "$*" >> "${log_file}"
}

log "MORAI health supervisor started target=${target}"
while true; do
  count="$(count_frames)"
  (( count < target )) || { log "target reached frames=${count}"; exit 0; }

  if ! pgrep -f '^/home/david/.*/Simulator\.x86_64([[:space:]]|$)' >/dev/null; then
    log "simulator missing at frames=${count}; starting recovery"
    "${script_dir}/recover_morai.sh" >> "${log_file}" 2>&1 || true
    previous="$(count_frames)"
    stale_checks=0
    sleep 60
    continue
  fi

  if (( count > previous )); then
    stale_checks=0
  else
    stale_checks=$((stale_checks + 1))
    log "collection stale check=${stale_checks}/6 frames=${count}"
  fi
  previous="${count}"

  # Six unchanged one-minute samples indicate a live-but-broken simulator or
  # LiDAR sender. Restarting is safe because raw files are paired atomically.
  if (( stale_checks >= 6 )); then
    log "collection stalled for six minutes; restarting simulator"
    mapfile -t simulator_pids < <(pgrep -f '^/home/david/.*/Simulator\.x86_64([[:space:]]|$)' || true)
    ((${#simulator_pids[@]} == 0)) || kill -TERM "${simulator_pids[@]}" 2>/dev/null || true
    sleep 8
    mapfile -t simulator_pids < <(pgrep -f '^/home/david/.*/Simulator\.x86_64([[:space:]]|$)' || true)
    ((${#simulator_pids[@]} == 0)) || kill -KILL "${simulator_pids[@]}" 2>/dev/null || true
    "${script_dir}/recover_morai.sh" >> "${log_file}" 2>&1 || true
    previous="$(count_frames)"
    stale_checks=0
  fi
  sleep 60
done
