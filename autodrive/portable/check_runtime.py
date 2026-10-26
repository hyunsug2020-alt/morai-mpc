#!/usr/bin/env python3
"""Validate existing models and CUDA runtimes without starting a ROS drive."""
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'autodrive'))
sys.path.insert(0, str(ROOT/'autodrive/student_v3_claude'))


def require(path):
    path = Path(path)
    if not path.is_file() or not path.stat().st_size:
        raise SystemExit('Required existing model/config missing: '+str(path))
    return path


def main():
    import numpy as np
    import torch
    if not torch.cuda.is_available():
        raise SystemExit('CUDA unavailable in '+sys.executable)
    from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
    if sys.argv[1] == 'student':
        from model import load_checkpoint
        model, checkpoint = load_checkpoint(require(os.environ['STUDENT_CHECKPOINT']), 'cuda')
        import gui
        # E2E: the v6.3 judgement model and the packages the launch imports before the car moves.
        import v6, v6.shadow, v6.shadow_child, v6_ext_child, model_avoidance, tunnel_static_portable  # noqa: F401,E401
        require(os.environ.get('STUDENT_V6_CHECKPOINT') or ROOT/'autodrive/student_v3_claude/v6_model.pt')
        from competition_profiles import profile
        profile('single_pc')
        from competition_speed import load_state, targets
        if targets(load_state()) != (55., 4.2):
            raise SystemExit('Speed settings do not match the verified run: expected 55 km/h and 4.2 m/s^2')
        semantic = Path(os.environ['SEMANTIC_MODEL_DIR'])
        require(semantic/'best_fp16.onnx')
        meta = json.loads(require(semantic/'best_fp16.onnx.json').read_text())
        require(semantic/'classes.yaml')
        for filename in (meta.get('view_exclusion_masks') or {}).values():
            require(semantic/filename)
        print('Student CUDA model, E2E v6 model + planner imports, single_pc profile and speed settings OK')
    elif sys.argv[1] == 'vehicles':
        from pointpillar_runtime import PointPillarRuntime
        directory = Path(os.environ['STUDENT_VEHICLE_MODEL_DIR'])
        runtime = PointPillarRuntime(require(directory/'pointpillar_morai.yaml'),
            require(directory/'morai_dataset.yaml'), require(directory/'best_model_epoch_20.pth'),
            strict_checkpoint=True)
        x, y = np.meshgrid(np.linspace(5., 35., 32), np.linspace(-8., 8., 16))
        points = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size), np.ones(x.size))).astype(np.float32)
        boxes, scores, labels = runtime.infer(points, 'portable-check')
        if not all(np.isfinite(value).all() for value in (boxes, scores, labels)):
            raise SystemExit('PointPillars returned nonfinite output')
        print('PointPillars strict model load and CUDA inference OK')
    else:
        raise SystemExit('Expected student or vehicles')


if __name__ == '__main__':
    main()
