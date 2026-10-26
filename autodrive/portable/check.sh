#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/common.sh"
set +u
source /opt/ros/noetic/setup.bash
source "$STUDENT_MPC_WS/devel/setup.bash"
set -u
export PYTHONPATH="$STUDENT_MPC_WS/devel/lib/python3/dist-packages:${PYTHONPATH:-}"
"$STUDENT_PYTHON" "$BUNDLE_ROOT/autodrive/portable/check_runtime.py" student
"$STUDENT_VEHICLE_PYTHON" "$BUNDLE_ROOT/autodrive/portable/check_runtime.py" vehicles
"${COMPOSE[@]}" config --quiet
if [[ "${1:-}" != --host-only ]]; then
  "${COMPOSE[@]}" run --rm --no-deps semantic python -c \
    'import torch,onnxruntime as o; assert torch.cuda.is_available(); s=o.InferenceSession("/models/best_fp16.onnx",providers=["CUDAExecutionProvider"]); assert s.get_providers()[0]=="CUDAExecutionProvider"; print("Semantic CUDA session OK")'
fi
echo 'Runtime validation passed; no driving command was sent.'
