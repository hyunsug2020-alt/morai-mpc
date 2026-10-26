#!/usr/bin/env python3
"""Train B0 on three general views and one traffic-light-only view."""

from __future__ import annotations

import argparse
import json
import os
import random
import shutil
import signal
from datetime import datetime
from pathlib import Path

os.environ.setdefault("NO_ALBUMENTATIONS_UPDATE", "1")
os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

import cv2
import numpy as np
import torch
import torch.distributed as dist
import torch.nn.functional as F
from torch.nn.parallel import DistributedDataParallel
from torch.utils.data import DataLoader, Dataset, DistributedSampler, WeightedRandomSampler

from semantic_common import (
    MODEL_ROOT,
    PROCESSED_ROOT,
    atomic_write_json,
    letterbox,
    load_class_config,
    read_latest,
    write_latest,
)
from model import build_model
from camera_config import ALL_CLASS_IDS, BACKBONES, CAMERA_PAIRS, supervised_class_ids


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, help="default: latest processed dataset")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--evaluate", type=Path)
    parser.add_argument("--init", type=Path, help="optional EfficientViT checkpoint initialization")
    parser.add_argument("--backbone", default="efficientvit_b1.r256_in1k")
    parser.add_argument("--head-channels", type=int, default=128)
    parser.add_argument("--no-pretrained", action="store_true")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=384)
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--lr", type=float, default=6e-5)
    parser.add_argument("--weight-decay", type=float, default=1e-2)
    parser.add_argument("--dice-weight", type=float, default=0.3)
    parser.add_argument("--patience", type=int, default=12)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--smoke", action="store_true", help="2 epochs and at most 96 samples per split")
    parser.add_argument("--general-only", action="store_true", help="train/evaluate general cameras while traffic-light collection is deferred")
    parser.add_argument("--no-amp", action="store_true")
    return parser.parse_args()


class SemanticDataset(Dataset):
    def __init__(self, root, split, width, height, ignore_index, training=False, limit=0):
        self.root = root
        self.width = width
        self.height = height
        self.ignore_index = ignore_index
        self.training = training
        stats = json.loads((root / 'statistics.json').read_text()) if (root / 'statistics.json').is_file() else {}
        exclusion_name = stats.get('front_exclusion_mask')
        self.front_exclusion = cv2.imread(str(root / exclusion_name), cv2.IMREAD_GRAYSCALE) if exclusion_name else None
        if exclusion_name and self.front_exclusion is None:
            raise ValueError('missing calibrated front exclusion mask')
        with (root / "manifest.jsonl").open(encoding="utf-8") as stream:
            records = [json.loads(line) for line in stream]
            self.records = [record for record in records if record.get("split") == split]
        if limit:
            chosen = random.Random(42).sample(self.records, min(limit, len(self.records)))
            self.records = chosen
        if not self.records:
            raise ValueError(f"no {split} samples in {root}")

    def __len__(self):
        return len(self.records)

    def __getitem__(self, index):
        record = self.records[index]
        image = cv2.imread(str(self.root / record["image"]), cv2.IMREAD_COLOR)
        mask = cv2.imread(str(self.root / record["mask"]), cv2.IMREAD_UNCHANGED)
        valid = cv2.imread(str(self.root / record["valid_mask"]), cv2.IMREAD_UNCHANGED) if "valid_mask" in record else None
        if image is None or mask is None or valid is None:
            raise ValueError(f"unreadable sample: {record}")
        exclusion = None
        if record['view'] == 'front' and self.front_exclusion is not None:
            if self.front_exclusion.shape != mask.shape:
                raise ValueError('front exclusion mask resolution mismatch')
            exclusion = self.front_exclusion.copy()
            mask[exclusion > 0] = self.ignore_index
            valid[exclusion > 0] = 0
            image[exclusion > 0] = 0
        if self.training and random.random() < 0.5:
            image = cv2.flip(image, 1)
            mask = cv2.flip(mask, 1)
            valid = cv2.flip(valid, 1)
            if exclusion is not None:
                exclusion = cv2.flip(exclusion, 1)
        if self.training:
            alpha = random.uniform(0.75, 1.25)
            beta = random.uniform(-22.0, 22.0)
            image = np.clip(image.astype(np.float32) * alpha + beta, 0, 255).astype(np.uint8)
            if random.random() < 0.2:
                image = cv2.GaussianBlur(image, (3, 3), 0)
            if random.random() < 0.2:
                noise = np.random.normal(0, 5, image.shape).astype(np.int16)
                image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        image, _ = letterbox(image, self.width, self.height, cv2.INTER_LINEAR, 0)
        mask, _ = letterbox(mask, self.width, self.height, cv2.INTER_NEAREST, self.ignore_index)
        valid, _ = letterbox(valid, self.width, self.height, cv2.INTER_NEAREST, 0)
        if exclusion is not None:
            exclusion, _ = letterbox(exclusion, self.width, self.height, cv2.INTER_NEAREST, 0)
            # Keep the hood constant even after brightness/noise augmentation.
            image[exclusion > 0] = 0
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        image = (image - np.asarray((0.485, 0.456, 0.406), np.float32)) / np.asarray(
            (0.229, 0.224, 0.225), np.float32
        )
        supervision = torch.tensor([class_id in supervised_class_ids(record) for class_id in ALL_CLASS_IDS])
        return torch.from_numpy(image.transpose(2, 0, 1).copy()), torch.from_numpy(mask.astype(np.int64)), supervision, torch.from_numpy(valid.astype(bool))


def load_matching(model, checkpoint):
    obj = torch.load(checkpoint, map_location="cpu", weights_only=False)
    source = obj.get("model_state_dict", obj) if isinstance(obj, dict) else obj
    target = model.state_dict()
    matching = {key: value for key, value in source.items() if key in target and value.shape == target[key].shape}
    target.update(matching)
    model.load_state_dict(target)
    return len(matching), len(target)


def class_weights(data_root, classes, ignore_index):
    stats_path = data_root / "statistics.json"
    if stats_path.is_file():
        stats = json.loads(stats_path.read_text(encoding="utf-8"))
        counts = np.asarray([max(0, int(item["pixels"])) for item in stats["classes"]], np.float64)
    else:
        counts = np.ones(classes, np.float64)
    present = counts > 0
    frequencies = counts / max(1.0, counts.sum())
    median = np.median(frequencies[present]) if present.any() else 1.0
    weights = np.ones(classes, np.float32)
    weights[present] = np.clip(np.sqrt(median / frequencies[present]), 0.25, 8.0)
    weights[~present] = 0.0
    return torch.tensor(weights, dtype=torch.float32)


def foreground_pos_weights(data_root, classes):
    stats = json.loads((data_root / "statistics.json").read_text(encoding="utf-8"))
    total = np.asarray(stats.get("supervised_pixels_per_class", [stats["total_pixels"]] * classes), np.float64)
    total = np.maximum(1, total)
    counts = np.asarray([max(1, int(row["pixels"])) for row in stats["classes"]], np.float64)
    return torch.tensor(np.clip((total - counts) / counts, 1.0, 30.0), dtype=torch.float32)


def foreground_truth(targets, classes):
    return torch.stack([(targets == class_id) for class_id in range(classes)], dim=1).float()


def foreground_bce_loss(logits, truth, weights, supervision, valid=None):
    losses = F.binary_cross_entropy_with_logits(logits, truth, pos_weight=weights[None, :, None, None], reduction="none")
    active = supervision[:, :, None, None].to(losses.dtype)
    if valid is not None:
        active = active * valid[:, None].to(losses.dtype)
    else:
        active = active.expand_as(losses)
    denominator = active.sum().clamp_min(1)
    return (losses * active).sum() / denominator


def foreground_dice_loss(logits, truth, supervision, valid=None):
    active_channels = supervision[:, :, None, None]
    if valid is not None:
        active_channels = active_channels & valid[:, None]
    probabilities = torch.sigmoid(logits) * active_channels
    truth = truth * active_channels
    intersection = (probabilities * truth).sum(dim=(0, 2, 3))
    denominator = probabilities.sum(dim=(0, 2, 3)) + truth.sum(dim=(0, 2, 3))
    active = truth.sum(dim=(0, 2, 3)) > 0
    losses = 1.0 - (2.0 * intersection + 1.0) / (denominator + 1.0)
    return losses[active].mean() if active.any() else logits.new_zeros(())


def foreground_metrics(counts):
    tp, fp, fn = counts.unbind(1)
    union = tp + fp + fn
    iou = torch.where(union > 0, tp.float() / union.clamp_min(1), torch.nan)
    return float(torch.nan_to_num(torch.nanmean(iou))), [None if torch.isnan(value) else float(value) for value in iou]


def dice_loss(logits, targets, classes, ignore_index):
    probabilities = torch.softmax(logits, dim=1)
    valid = targets != ignore_index
    losses = []
    for class_id in range(classes):
        truth = (targets == class_id) & valid
        if not truth.any():
            continue
        probability = probabilities[:, class_id] * valid
        intersection = (probability * truth).sum()
        denominator = probability.sum() + truth.sum()
        losses.append(1.0 - (2.0 * intersection + 1.0) / (denominator + 1.0))
    return torch.stack(losses).mean() if losses else logits.new_zeros(())


def update_confusion(confusion, logits, targets, classes, ignore_index):
    prediction = logits.argmax(1)
    valid = targets != ignore_index
    encoded = targets[valid] * classes + prediction[valid]
    confusion += torch.bincount(encoded, minlength=classes * classes).reshape(classes, classes)


def metrics(confusion):
    intersection = confusion.diag().float()
    union = confusion.sum(0) + confusion.sum(1) - intersection
    iou = torch.where(union > 0, intersection / union, torch.nan)
    return float(torch.nan_to_num(torch.nanmean(iou))), [None if torch.isnan(value) else float(value) for value in iou]


def run_epoch(
    model, loader, optimizer, scaler, device, weights, classes, ignore_index,
    dice_weight, training, amp, distributed, foreground=False,
):
    model.train(training)
    confusion = torch.zeros((classes, 3) if foreground else (classes, classes), dtype=torch.int64, device=device)
    loss_total = 0.0
    sample_total = 0
    context = torch.enable_grad if training else torch.no_grad
    with context():
        for batch in loader:
            images, targets, supervision = batch[:3]
            valid = batch[3].to(device, non_blocking=True) if len(batch) == 4 else torch.ones_like(targets, dtype=torch.bool, device=device)
            images = images.to(device, non_blocking=True)
            targets = targets.to(device, non_blocking=True)
            supervision = supervision.to(device, non_blocking=True)
            if training:
                optimizer.zero_grad(set_to_none=True)
            with torch.amp.autocast(device_type=device.type, enabled=amp):
                logits = model(images)
                if foreground:
                    truth = foreground_truth(targets, classes)
                    ce = foreground_bce_loss(logits, truth, weights, supervision, valid)
                    loss = ce + dice_weight * foreground_dice_loss(logits, truth, supervision, valid)
                else:
                    ce = F.cross_entropy(logits, targets, weight=weights, ignore_index=ignore_index)
                    loss = ce + dice_weight * dice_loss(logits, targets, classes, ignore_index)
            if training:
                scaler.scale(loss).backward()
                scaler.step(optimizer)
                scaler.update()
            loss_total += float(loss.detach()) * images.shape[0]
            sample_total += images.shape[0]
            if foreground:
                # Compare logits before sigmoid: FP16 sigmoid saturates and can
                # turn different high scores into ties with the wrong class.
                scores = logits.detach().float().masked_fill(~supervision[:, :, None, None], -torch.inf)
                confidence, prediction = scores.max(1)
                prediction = prediction.masked_fill(confidence < 0.0, ignore_index)
                for class_id in range(classes):
                    positive = targets == class_id
                    predicted = prediction == class_id
                    active = supervision[:, class_id, None, None] & valid
                    confusion[class_id, 0] += (positive & predicted & active).sum()
                    confusion[class_id, 1] += (~positive & predicted & active).sum()
                    confusion[class_id, 2] += (positive & ~predicted & active).sum()
            else:
                update_confusion(confusion, logits.detach(), targets, classes, ignore_index)
    totals = torch.tensor([loss_total, sample_total], dtype=torch.float64, device=device)
    if distributed:
        dist.all_reduce(confusion, op=dist.ReduceOp.SUM)
        dist.all_reduce(totals, op=dist.ReduceOp.SUM)
    if foreground:
        tp, fp, fn = confusion.unbind(1)
        run_epoch.details = {"precision": [float(a / max(1, int(a+b))) for a,b in zip(tp,fp)],
                             "recall": [float(a / max(1, int(a+b))) for a,b in zip(tp,fn)],
                             "counts_tp_fp_fn": confusion.cpu().tolist()}
    mean_iou, per_class = foreground_metrics(confusion) if foreground else metrics(confusion)
    return float(totals[0] / totals[1].clamp_min(1)), mean_iou, per_class


def main() -> int:
    args = parse_args()
    resume = torch.load(args.resume or args.evaluate, map_location="cpu", weights_only=False) if args.resume or args.evaluate else None
    if resume:
        for arg, key in (("backbone", "backbone"), ("head_channels", "head_channels"), ("width", "input_width"), ("height", "input_height")):
            setattr(args, arg, resume[key])
        args.general_only = resume.get('general_only', args.general_only)
    stop_requested = False
    def request_stop(*_):
        nonlocal stop_requested
        stop_requested = True
        print("중지 요청: 현재 epoch 완료 후 체크포인트를 저장합니다.", flush=True)
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    world_size = int(os.environ.get("WORLD_SIZE", "1"))
    rank = int(os.environ.get("RANK", "0"))
    local_rank = int(os.environ.get("LOCAL_RANK", "0"))
    distributed = world_size > 1
    if distributed:
        if not torch.cuda.is_available():
            raise RuntimeError("distributed semantic training requires CUDA")
        torch.cuda.set_device(local_rank)
        dist.init_process_group(backend="nccl")
    device = torch.device("cuda", local_rank) if torch.cuda.is_available() else torch.device("cpu")
    root = args.data.expanduser().resolve() if args.data else read_latest(PROCESSED_ROOT)
    stats = json.loads((root / "statistics.json").read_text())
    if stats.get("label_version") != 2:
        raise ValueError("기존 데이터는 유효 픽셀 마스크가 없습니다. 새 데이터 준비 또는 v2 가져오기를 먼저 실행하세요.")
    if not args.smoke and stats.get("split_method") != "session":
        raise ValueError("본 학습/평가는 독립 세션 분리가 필요합니다. 시간 분리 데이터는 짧은 학습 점검에만 사용하세요.")
    output = (args.resume or args.evaluate).parent if resume else args.output or (
        MODEL_ROOT / datetime.now().strftime("run_%Y%m%d_%H%M%S")
    )
    output = output.expanduser().resolve()
    if output.exists() and not resume:
        raise FileExistsError(f"output already exists: {output}")
    if rank == 0:
        output.mkdir(parents=True, exist_ok=bool(resume))
    if distributed:
        dist.barrier()
    config, class_items, _palette, _ids = load_class_config(root / "classes.yaml")
    classes = len(class_items)
    foreground = config.get("mode") == "foreground_sigmoid"
    if not foreground or args.backbone not in BACKBONES or classes != 6:
        raise ValueError("This project trains six-class EfficientViT-B0/B1 foreground models")
    ignore_index = int(config.get("ignore_index", 255))
    epochs = 2 if args.smoke else args.epochs
    limit = 96 if args.smoke else 0
    process_seed = args.seed + rank
    random.seed(process_seed)
    np.random.seed(process_seed)
    torch.manual_seed(process_seed)
    torch.cuda.manual_seed_all(args.seed)

    train_set = SemanticDataset(root, "train", args.width, args.height, ignore_index, True, limit)
    val_set = SemanticDataset(root, "val", args.width, args.height, ignore_index, False, limit)
    required_views = {'front', 'left', 'right'} if args.general_only else set(CAMERA_PAIRS)
    if not args.smoke and not args.evaluate and {row["view"] for row in train_set.records} != required_views:
        raise ValueError(f"학습 카메라 구성이 다릅니다: 필요한 카메라 {sorted(required_views)}")
    samplers = {
        "train": DistributedSampler(train_set, shuffle=True, seed=args.seed) if distributed else None,
        "val": DistributedSampler(val_set, shuffle=False) if distributed else None,
    }
    if not distributed:
        counts = {g: sum((row["view"] == "signal") == (g == "signal") for row in train_set.records) for g in ("general", "signal")}
        weights_sampling = [(1 if row["view"] == "signal" else 3) / max(1, counts["signal" if row["view"] == "signal" else "general"]) for row in train_set.records]
        samplers["train"] = WeightedRandomSampler(weights_sampling, len(train_set), replacement=True)
    loaders = {
        "train": DataLoader(
            train_set, batch_size=args.batch_size, shuffle=samplers["train"] is None,
            sampler=samplers["train"], num_workers=args.workers, pin_memory=device.type == "cuda",
            persistent_workers=args.workers > 0,
        ),
        "val": DataLoader(
            val_set, batch_size=args.batch_size, shuffle=False, sampler=samplers["val"],
            num_workers=args.workers, pin_memory=device.type == "cuda",
            persistent_workers=args.workers > 0,
        ),
    }
    model = build_model(
        backbone_name=args.backbone,
        classes=classes,
        head_channels=args.head_channels,
        pretrained=not args.no_pretrained and args.init is None and resume is None,
    ).to(device)
    if resume:
        model.load_state_dict(resume["model_state_dict"])
    if args.init:
        loaded, total = load_matching(model, args.init.expanduser().resolve())
        if rank == 0:
            print(f"initialization={args.init} tensors={loaded}/{total}")
    if distributed:
        model = DistributedDataParallel(model, device_ids=[local_rank], output_device=local_rank)
    weights = foreground_pos_weights(root, classes).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    scaler = torch.amp.GradScaler("cuda", enabled=device.type == "cuda" and not args.no_amp)
    amp = scaler.is_enabled()
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    if args.evaluate:
        test = SemanticDataset(root, "test", args.width, args.height, ignore_index)
        report = {"dataset": str(root), "backbone": args.backbone, "views": {},
                  "class_names": [item['name'] for item in class_items],
                  "general_only": args.general_only,
                  "front_exclusion_mask": stats.get('front_exclusion_mask')}
        for view in CAMERA_PAIRS:
            subset = torch.utils.data.Subset(test, [i for i,row in enumerate(test.records) if row['view'] == view])
            if not len(subset):
                continue
            loader = DataLoader(subset, batch_size=args.batch_size, num_workers=args.workers)
            loss, miou, ious = run_epoch(model, loader, optimizer, scaler, device, weights, classes, ignore_index, args.dice_weight, False, amp, False, True)
            report['views'][view] = {'samples': len(subset), 'loss': loss, 'miou': miou, 'iou': ious, **run_epoch.details}
        counts = torch.tensor([v['counts_tp_fp_fn'] for v in report['views'].values()]).sum(0)
        mean_iou, per_class = foreground_metrics(counts)
        report['overall'] = {'miou': mean_iou, 'iou': per_class, 'counts_tp_fp_fn': counts.tolist()}
        atomic_write_json(output / 'test_metrics.json', report)
        print(json.dumps(report), flush=True)
        return 0
    if resume:
        if Path(resume.get('dataset', '')).name != root.name:
            raise ValueError('재개 데이터셋이 원래 학습과 다릅니다.')
        optimizer.load_state_dict(resume['optimizer_state_dict'])
        scaler.load_state_dict(resume['scaler_state_dict'])
        scheduler.load_state_dict(resume['scheduler_state_dict'])
        scheduler.T_max = epochs
        if 'rng_state' in resume:
            torch.set_rng_state(resume['rng_state'])
            random.setstate(resume['python_rng_state'])
            np.random.set_state(resume['numpy_rng_state'])
            if device.type == 'cuda' and resume.get('cuda_rng_state') is not None:
                torch.cuda.set_rng_state_all(resume['cuda_rng_state'])
    history = json.loads((output / 'history.json').read_text()) if resume and (output / 'history.json').exists() else []
    best_iou = resume.get('best_miou', -1.0) if resume else -1.0
    stale = resume.get('stale', 0) if resume else 0
    start_epoch = resume['epoch'] + 1 if resume else 1
    if start_epoch > epochs:
        raise ValueError('재개하려면 최대 epoch를 완료된 epoch보다 크게 설정하세요.')
    if rank == 0:
        atomic_write_json(output / 'run.json', {'backbone': args.backbone, 'dataset': str(root), 'camera_policy': CAMERA_PAIRS,
                          'animal_verified': stats.get('animal_verified', False), 'input': [args.width, args.height], 'smoke': args.smoke,
                          'general_only': args.general_only, 'front_exclusion_mask': stats.get('front_exclusion_mask')})
        shutil.copy2(root / 'classes.yaml', output / 'classes.yaml')
        if stats.get('front_exclusion_mask'):
            shutil.copy2(root / stats['front_exclusion_mask'], output / 'front_exclusion_mask.png')
    for epoch in range(start_epoch, epochs + 1):
        if isinstance(samplers["train"], DistributedSampler):
            samplers["train"].set_epoch(epoch)
        train_loss, train_iou, _ = run_epoch(
            model, loaders["train"], optimizer, scaler, device, weights, classes,
            ignore_index, args.dice_weight, True, amp, distributed, foreground,
        )
        val_loss, val_iou, per_class = run_epoch(
            model, loaders["val"], optimizer, scaler, device, weights, classes,
            ignore_index, args.dice_weight, False, amp, distributed, foreground,
        )
        row = {"epoch": epoch, "train_loss": train_loss, "train_miou": train_iou, "val_loss": val_loss, "val_miou": val_iou}
        row.update(val_iou_per_class=per_class, **run_epoch.details)
        history.append(row)
        scheduler.step()
        if rank == 0:
            print(json.dumps(row))
        unwrapped_model = model.module if isinstance(model, DistributedDataParallel) else model
        checkpoint = {
            "epoch": epoch,
            "dataset": str(root),
            "camera_policy": CAMERA_PAIRS,
            "label_version": 2,
            "scheduler_state_dict": scheduler.state_dict(),
            "rng_state": torch.get_rng_state(),
            "python_rng_state": random.getstate(),
            "numpy_rng_state": np.random.get_state(),
            "cuda_rng_state": torch.cuda.get_rng_state_all() if device.type == 'cuda' else None,
            "best_miou": max(best_iou, val_iou),
            "model_family": "efficientvit_segmentation",
            "mode": "foreground_sigmoid",
            "backbone": args.backbone,
            "head_channels": args.head_channels,
            "num_classes": classes,
            "class_names": [item["name"] for item in class_items],
            "ignore_index": ignore_index,
            "general_only": args.general_only,
            "front_exclusion_mask": stats.get('front_exclusion_mask'),
            "input_width": args.width,
            "input_height": args.height,
            "world_size": world_size,
            "model_state_dict": unwrapped_model.state_dict(),
            "optimizer_state_dict": optimizer.state_dict(),
            "scaler_state_dict": scaler.state_dict(),
            "val_iou_per_class": per_class,
        }
        improved = val_iou > best_iou
        if improved:
            best_iou = val_iou
            stale = 0
        else:
            stale += 1
        checkpoint["stale"] = stale
        if rank == 0:
            torch.save(checkpoint, output / "last.pt.tmp")
            (output / "last.pt.tmp").replace(output / "last.pt")
            if improved:
                torch.save(checkpoint, output / "best.pt.tmp")
                (output / "best.pt.tmp").replace(output / "best.pt")
            atomic_write_json(output / "history.json", history)
        if stop_requested:
            break
        if stale >= args.patience:
            if rank == 0:
                print(f"early_stop epoch={epoch}")
            break
    if rank == 0:
        shutil.copy2(root / "classes.yaml", output / "classes.yaml")
        print(f"best_miou={best_iou:.6f}")
        write_latest(MODEL_ROOT, output)
        print(output)
    if distributed:
        dist.barrier()
        dist.destroy_process_group()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
