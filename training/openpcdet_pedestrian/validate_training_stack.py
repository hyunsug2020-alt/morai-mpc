#!/usr/bin/env python3
"""Run ten real CUDA train steps before starting the long pedestrian job."""

import argparse
import datetime as dt
import json
import logging
import math
import os
from pathlib import Path
import subprocess
import sys
import time


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
OPENPCDET_ROOT = REPO_ROOT / "third_party" / "OpenPCDet"
TOOLS_DIR = OPENPCDET_ROOT / "tools"
DEFAULT_CONFIG = SCRIPT_DIR / "pointpillar_pedestrian.yaml"
DEFAULT_PRETRAINED = (
    REPO_ROOT
    / "trained_models"
    / "morai_pointpillar_2026-07-31"
    / "best_model_epoch_39.pth"
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rounds", type=int, default=10)
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--pretrained", type=Path, default=DEFAULT_PRETRAINED)
    parser.add_argument("--allow-unsupported-bios", action="store_true")
    return parser.parse_args()


def nvidia_query():
    fields = "driver_version,name,temperature.gpu,power.draw,power.limit,memory.used"
    output = subprocess.check_output(
        ["nvidia-smi", "--query-gpu={}".format(fields), "--format=csv,noheader,nounits"],
        text=True,
    ).strip()
    values = [item.strip() for item in output.split(",")]
    return {
        "driver": values[0],
        "gpu": values[1],
        "temperature_c": float(values[2]),
        "power_w": float(values[3]),
        "power_limit_w": float(values[4]),
        "memory_used_mib": float(values[5]),
    }


def atomic_json_dump(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def main():
    args = parse_args()
    if args.rounds != 10:
        raise ValueError("validation policy requires exactly 10 rounds")
    bios_text = Path("/sys/class/dmi/id/bios_version").read_text().strip()
    if int(bios_text) < 2603 and not args.allow_unsupported_bios:
        raise RuntimeError("BIOS {} requires protected override".format(bios_text))
    no_turbo = Path("/sys/devices/system/cpu/intel_pstate/no_turbo").read_text().strip()
    initial_gpu = nvidia_query()
    if no_turbo != "1" or initial_gpu["power_limit_w"] > 250.5:
        raise RuntimeError(
            "hardware guard missing: no_turbo={} gpu={}".format(no_turbo, initial_gpu)
        )
    if not args.pretrained.is_file():
        raise FileNotFoundError(args.pretrained)

    os.environ.setdefault("CUDA_VISIBLE_DEVICES", "0")
    os.environ.setdefault("CUDA_MODULE_LOADING", "LAZY")
    os.environ.setdefault("OMP_NUM_THREADS", "4")
    os.environ.setdefault("MKL_NUM_THREADS", "4")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "4")
    os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "max_split_size_mb:128")
    sys.path.insert(0, str(TOOLS_DIR))
    sys.path.insert(0, str(OPENPCDET_ROOT))
    os.chdir(TOOLS_DIR)

    import numpy as np
    import torch
    from pcdet.config import cfg, cfg_from_yaml_file
    from pcdet.datasets import build_dataloader
    from pcdet.models import build_network, model_fn_decorator
    from train_utils.optimization import build_optimizer

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable")
    torch.manual_seed(666)
    torch.cuda.manual_seed_all(666)
    np.random.seed(666)
    cfg_from_yaml_file(str(args.config.resolve()), cfg)
    logger = logging.getLogger("morai-pedestrian-stack-validation")
    logger.setLevel(logging.INFO)
    logger.addHandler(logging.StreamHandler(sys.stdout))
    dataset, loader, _ = build_dataloader(
        dataset_cfg=cfg.DATA_CONFIG,
        class_names=cfg.CLASS_NAMES,
        batch_size=args.batch_size,
        dist=False,
        workers=0,
        logger=logger,
        training=True,
        seed=666,
    )
    if len(loader) < args.rounds:
        raise RuntimeError("not enough training batches")
    model = build_network(
        model_cfg=cfg.MODEL,
        num_class=len(cfg.CLASS_NAMES),
        dataset=dataset,
    ).cuda()
    model.load_params_from_file(str(args.pretrained.resolve()), logger=logger, to_cpu=False)
    optimizer = build_optimizer(model, cfg.OPTIMIZATION)
    model.train()
    model_func = model_fn_decorator()

    rounds = []
    started = time.monotonic()
    iterator = iter(loader)
    for index in range(1, args.rounds + 1):
        batch = next(iterator)
        optimizer.zero_grad()
        before = time.monotonic()
        result = model_func(model, batch)
        if not torch.isfinite(result.loss):
            raise RuntimeError("non-finite loss at round {}".format(index))
        result.loss.backward()
        grad_norm = torch.nn.utils.clip_grad_norm_(
            model.parameters(), cfg.OPTIMIZATION.GRAD_NORM_CLIP
        )
        if not math.isfinite(float(grad_norm)):
            raise RuntimeError("non-finite gradient at round {}".format(index))
        optimizer.step()
        torch.cuda.synchronize()
        gpu = nvidia_query()
        if gpu["temperature_c"] >= 80:
            raise RuntimeError("GPU reached stop threshold: {}".format(gpu))
        rounds.append(
            {
                "round": index,
                "loss": float(result.loss.detach().cpu()),
                "grad_norm": float(grad_norm),
                "seconds": round(time.monotonic() - before, 3),
                "gpu": gpu,
            }
        )
        print(
            "VALIDATION {:02d}/10 PASS loss={:.6f} temp={:.0f}C power={:.1f}W".format(
                index, rounds[-1]["loss"], gpu["temperature_c"], gpu["power_w"]
            ),
            flush=True,
        )

    payload = {
        "status": "PASS",
        "timestamp": dt.datetime.now().astimezone().isoformat(),
        "bios_version": bios_text,
        "unsupported_bios_override": args.allow_unsupported_bios,
        "cpu_no_turbo": no_turbo,
        "python": sys.version,
        "torch": torch.__version__,
        "torch_cuda": torch.version.cuda,
        "config": str(args.config.resolve()),
        "pretrained_backbone": str(args.pretrained.resolve()),
        "dataset_samples": len(dataset),
        "batch_size": args.batch_size,
        "elapsed_seconds": round(time.monotonic() - started, 3),
        "rounds": rounds,
    }
    result_path = SCRIPT_DIR / "validation_logs" / "validation_{}.json".format(
        dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    )
    atomic_json_dump(result_path, payload)
    print("VALIDATION PASS: {}".format(result_path), flush=True)


if __name__ == "__main__":
    main()
