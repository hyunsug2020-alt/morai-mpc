#!/usr/bin/env python3
"""Exercise the exact MORAI PointPillars training path without changing checkpoints."""

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
DEFAULT_CONFIG = SCRIPT_DIR / "pointpillar_morai.yaml"
DEFAULT_CKPT_DIR = (
    OPENPCDET_ROOT
    / "output/custom_models/pointpillar_morai/morai_10k/ckpt"
)
DEFAULT_RESULT_DIR = SCRIPT_DIR / "validation_logs"


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rounds", type=int, default=10)
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--allow-unsupported-bios", action="store_true")
    return parser.parse_args()


def latest_epoch_checkpoint():
    candidates = []
    for path in DEFAULT_CKPT_DIR.glob("checkpoint_epoch_*.pth"):
        try:
            epoch = int(path.stem.rsplit("_", 1)[1])
        except ValueError:
            continue
        candidates.append((epoch, path))
    if not candidates:
        raise RuntimeError(f"완료된 epoch 체크포인트가 없음: {DEFAULT_CKPT_DIR}")
    return max(candidates)[1]


def nvidia_query():
    fields = "driver_version,name,temperature.gpu,power.draw,power.limit,memory.used"
    output = subprocess.check_output(
        [
            "nvidia-smi",
            f"--query-gpu={fields}",
            "--format=csv,noheader,nounits",
        ],
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
        raise ValueError("검증 정책상 --rounds는 정확히 10이어야 함")
    if args.batch_size < 1:
        raise ValueError("batch size는 1 이상이어야 함")

    bios_text = Path("/sys/class/dmi/id/bios_version").read_text().strip()
    try:
        bios_version = int(bios_text)
    except ValueError as exc:
        raise RuntimeError(f"BIOS 버전을 숫자로 판독할 수 없음: {bios_text}") from exc
    if bios_version < 2603 and not args.allow_unsupported_bios:
        raise RuntimeError(
            f"BIOS {bios_version}은 i9-14900KS 최소 지원 버전 2603보다 낮음"
        )

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
        raise RuntimeError("PyTorch에서 CUDA를 사용할 수 없음")
    torch.manual_seed(666)
    torch.cuda.manual_seed_all(666)
    np.random.seed(666)

    config_path = args.config.resolve()
    cfg_from_yaml_file(str(config_path), cfg)
    checkpoint = (args.checkpoint or latest_epoch_checkpoint()).resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(checkpoint)

    logger = logging.getLogger("morai-stack-validation")
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
    if len(dataset) < 1 or len(loader) < args.rounds:
        raise RuntimeError(
            f"데이터가 10회 검증에 부족함: samples={len(dataset)}, batches={len(loader)}"
        )

    model = build_network(
        model_cfg=cfg.MODEL,
        num_class=len(cfg.CLASS_NAMES),
        dataset=dataset,
    ).cuda()
    optimizer = build_optimizer(model, cfg.OPTIMIZATION)
    loaded_iter, loaded_epoch = model.load_params_with_optimizer(
        str(checkpoint),
        to_cpu=False,
        optimizer=optimizer,
        logger=logger,
    )
    model.train()
    model_func = model_fn_decorator()

    started = time.monotonic()
    rounds = []
    iterator = iter(loader)
    for index in range(1, args.rounds + 1):
        batch = next(iterator)
        optimizer.zero_grad()
        before = time.monotonic()
        result = model_func(model, batch)
        if not torch.isfinite(result.loss):
            raise RuntimeError(f"{index}회차 loss가 유한하지 않음: {result.loss}")
        result.loss.backward()
        grad_norm = torch.nn.utils.clip_grad_norm_(
            model.parameters(), cfg.OPTIMIZATION.GRAD_NORM_CLIP
        )
        if not math.isfinite(float(grad_norm)):
            raise RuntimeError(f"{index}회차 gradient가 유한하지 않음: {grad_norm}")
        optimizer.step()
        torch.cuda.synchronize()
        gpu = nvidia_query()
        if gpu["temperature_c"] >= 90:
            raise RuntimeError(f"{index}회차 GPU 온도가 너무 높음: {gpu}")
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
            f"VALIDATION {index:02d}/10 PASS "
            f"loss={rounds[-1]['loss']:.6f} "
            f"temp={gpu['temperature_c']:.0f}C power={gpu['power_w']:.1f}W",
            flush=True,
        )

    payload = {
        "status": "PASS",
        "timestamp": dt.datetime.now().astimezone().isoformat(),
        "bios_version": bios_text,
        "unsupported_bios_override": args.allow_unsupported_bios,
        "python": sys.version,
        "torch": torch.__version__,
        "torch_cuda": torch.version.cuda,
        "checkpoint": str(checkpoint),
        "checkpoint_epoch": loaded_epoch,
        "checkpoint_iter": loaded_iter,
        "dataset_samples": len(dataset),
        "batch_size": args.batch_size,
        "elapsed_seconds": round(time.monotonic() - started, 3),
        "rounds": rounds,
    }
    result_path = (
        DEFAULT_RESULT_DIR
        / f"validation_{dt.datetime.now().strftime('%Y%m%d-%H%M%S')}.json"
    )
    atomic_json_dump(result_path, payload)
    print(f"VALIDATION PASS: {result_path}", flush=True)


if __name__ == "__main__":
    main()

