#!/usr/bin/env python3
"""Export a trained MORAI EfficientViT checkpoint to verified ONNX."""

from __future__ import annotations

import argparse
import json
import shutil
import math
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch

from semantic_common import MODEL_ROOT, read_latest
from model import build_model


class DeploymentModel(torch.nn.Module):
    def __init__(self, model, threshold=0.5, general_only=False):
        super().__init__()
        self.model = model
        self.threshold = math.log(threshold / (1.0 - threshold))
        self.general_only = general_only

    def forward(self, inputs):
        logits = self.model(inputs)
        scores = logits
        if self.general_only:
            # The traffic-light head has no supervision in general-only runs.
            active = torch.tensor([True, True, True, False, True, True], device=logits.device)
            scores = scores.masked_fill(~active[None, :, None, None], -torch.inf)
        confidence, classes = scores.max(dim=1)
        masks = torch.where(confidence >= self.threshold, classes, torch.full_like(classes, 255)).to(torch.uint8)
        return logits, masks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, help="default: latest run/best.pt")
    parser.add_argument("--output", type=Path, help="default: checkpoint directory/best.onnx")
    parser.add_argument("--opset", type=int, default=17)
    args = parser.parse_args()
    checkpoint_path = (
        args.checkpoint.expanduser().resolve()
        if args.checkpoint
        else read_latest(MODEL_ROOT) / "best.pt"
    )
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    if (checkpoint.get("model_family") != "efficientvit_segmentation"
            or checkpoint.get("mode") != "foreground_sigmoid"
            or checkpoint.get("backbone") not in ("efficientvit_b0.r224_in1k", "efficientvit_b1.r256_in1k")
            or checkpoint.get("num_classes") != 6):
        raise ValueError("checkpoint is not a six-class EfficientViT-B0/B1 model")
    model = build_model(
        backbone_name=checkpoint["backbone"],
        classes=int(checkpoint["num_classes"]),
        head_channels=int(checkpoint["head_channels"]),
        pretrained=False,
    )
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    deployment_model = DeploymentModel(model, general_only=checkpoint.get('general_only', False)).eval()
    width = int(checkpoint["input_width"])
    height = int(checkpoint["input_height"])
    dummy = torch.randn(1, 3, height, width, generator=torch.Generator().manual_seed(42))
    output = (
        args.output.expanduser().resolve()
        if args.output
        else checkpoint_path.with_name("best.onnx")
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        deployment_model,
        dummy,
        str(output),
        input_names=["input"],
        output_names=["logits", "masks"],
        dynamic_axes={
            "input": {0: "batch"},
            "logits": {0: "batch"},
            "masks": {0: "batch"},
        },
        opset_version=args.opset,
        do_constant_folding=True,
    )
    graph = onnx.load(str(output))
    onnx.checker.check_model(graph)
    providers = ["CPUExecutionProvider"]
    session = ort.InferenceSession(str(output), providers=providers)
    with torch.no_grad():
        expected_logits, expected_masks = deployment_model(dummy)
        expected_logits = expected_logits.numpy()
        expected_masks = expected_masks.numpy()
    actual_logits, actual_masks = session.run(["logits", "masks"], {"input": dummy.numpy()})
    max_abs = float(np.max(np.abs(expected_logits - actual_logits)))
    # CPU ONNX Runtime can differ from PyTorch by a few 1e-4 after convolution
    # fusion.  Keep the mask equality check below strict, while allowing this
    # harmless floating-point drift in logits.
    parity_rtol = 1e-3
    parity_atol = 1e-3
    if not np.allclose(expected_logits, actual_logits, rtol=parity_rtol, atol=parity_atol):
        raise RuntimeError(f"ONNX parity failed: max_abs={max_abs}")
    if not np.array_equal(expected_masks, actual_masks):
        raise RuntimeError("ONNX mask parity failed")
    metadata = {
        "checkpoint": str(checkpoint_path),
        "model_family": checkpoint["model_family"],
        "mode": checkpoint["mode"],
        "threshold": 0.5,
        "backbone": checkpoint["backbone"],
        "head_channels": checkpoint["head_channels"],
        "num_classes": checkpoint["num_classes"],
        "class_names": checkpoint["class_names"],
        "camera_policy": checkpoint.get("camera_policy"),
        "general_only": checkpoint.get("general_only", False),
        "front_exclusion_mask": checkpoint.get("front_exclusion_mask"),
        "dataset": checkpoint.get("dataset"),
        "input_width": width,
        "input_height": height,
        "output": "masks",
        "outputs": ["logits", "masks"],
        "opset": args.opset,
        "max_abs_error": max_abs,
        "parity_rtol": parity_rtol,
        "parity_atol": parity_atol,
    }
    if metadata['front_exclusion_mask']:
        source = checkpoint_path.parent / metadata['front_exclusion_mask']
        if not source.is_file():
            raise ValueError('missing calibrated front exclusion mask')
        destination = output.parent / source.name
        if source.resolve() != destination.resolve():
            shutil.copy2(source, destination)
    output.with_suffix(output.suffix + ".json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(metadata, ensure_ascii=False, indent=2))
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
