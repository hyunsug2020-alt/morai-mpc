#!/usr/bin/env bash
set -euo pipefail

project_root="/home/david/morai-mpc-agent-morai-lio-gps-integration"
mode="${1:-test}"
yolo_bin="$project_root/.venv-camera/bin/yolo"
data_config="$project_root/src/camera_vehicle_training/config/morai_camera_vehicle_yolo.yaml"
model="$project_root/yolo26s.pt"

if [[ ! -x "$yolo_bin" ]]; then
  printf 'error: YOLO environment not found: %s\n' "$yolo_bin" >&2
  exit 1
fi

"$project_root/scripts/prepare_camera_vehicle_yolo_dataset.py" >/dev/null

case "$mode" in
  test)
    epochs=10
    image_size=1280
    batch_size=8
    run_name="morai_camera_vehicle_test"
    ;;
  train)
    epochs=100
    image_size=1280
    batch_size=8
    run_name="morai_camera_vehicle"
    ;;
  *)
    printf 'usage: %s [test|train] [additional yolo arguments...]\n' "$0" >&2
    exit 2
    ;;
esac
shift || true

exec "$yolo_bin" detect train \
  model="$model" \
  data="$data_config" \
  epochs="$epochs" \
  imgsz="$image_size" \
  batch="$batch_size" \
  device=0 \
  workers=8 \
  rect=True \
  patience=20 \
  project="$project_root/trained_models" \
  name="$run_name" \
  "$@"
