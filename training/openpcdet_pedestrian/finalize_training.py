#!/usr/bin/env python3
"""Select and archive the best completed pedestrian PointPillars checkpoint."""

import hashlib
import json
import os
import pickle
import re
import shutil
import sys
from datetime import datetime
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPOSITORY = SCRIPT_DIR.parents[1]
OPENPCDET = REPOSITORY / "third_party" / "OpenPCDet"
TOOLS = OPENPCDET / "tools"
OUTPUT = (
    OPENPCDET
    / "output"
    / "custom_models"
    / "pointpillar_pedestrian"
    / "morai_pedestrian_30k"
)
ARCHIVE = REPOSITORY / "trained_models" / "morai_pedestrian_pointpillar_2026-08-01"
CONFIG = SCRIPT_DIR / "pointpillar_pedestrian.yaml"


def atomic_write(path, text):
    temporary = Path(str(path) + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    os.replace(str(temporary), str(path))


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def epoch_number(path):
    match = re.search(r"epoch_(\d+)", str(path))
    if not match:
        raise RuntimeError("epoch number is missing from {}".format(path))
    return int(match.group(1))


def load_recall(log_text, epoch):
    marker = "Performance of EPOCH {}".format(epoch)
    start = log_text.find(marker)
    if start < 0:
        return {}
    following = log_text.find("Performance of EPOCH ", start + len(marker))
    block = log_text[start : following if following >= 0 else len(log_text)]
    return {
        threshold: float(value)
        for threshold, value in re.findall(
            r"recall_rcnn_([0-9.]+):\s*([0-9.]+)", block
        )
    }


def main():
    result_files = sorted(
        (OUTPUT / "eval" / "eval_with_train").glob("epoch_*/val/result.pkl"),
        key=epoch_number,
    )
    if len(result_files) < 5:
        raise RuntimeError(
            "at least five completed evaluation results are required, found {}".format(
                len(result_files)
            )
        )

    checkpoints = {
        epoch_number(path): path
        for path in (OUTPUT / "ckpt").glob("checkpoint_epoch_*.pth")
    }
    train_logs = sorted(OUTPUT.glob("train_*.log"), key=lambda item: item.stat().st_mtime)
    if not train_logs:
        raise RuntimeError("training log is missing")
    training_log = train_logs[-1]
    log_text = training_log.read_text(encoding="utf-8", errors="replace")
    if "End evaluation" not in log_text:
        raise RuntimeError("evaluation did not finish cleanly")

    os.chdir(str(TOOLS))
    sys.path.insert(0, str(OPENPCDET))
    from pcdet.config import cfg, cfg_from_yaml_file
    from pcdet.datasets.custom.custom_dataset import CustomDataset
    from pcdet.utils import common_utils

    cfg_from_yaml_file(str(CONFIG), cfg)
    dataset = CustomDataset(
        dataset_cfg=cfg.DATA_CONFIG,
        class_names=cfg.CLASS_NAMES,
        training=False,
        root_path=None,
        logger=common_utils.create_logger(log_file=None, rank=0),
    )

    evaluations = []
    for result_file in result_files:
        epoch = epoch_number(result_file)
        checkpoint = checkpoints.get(epoch)
        if checkpoint is None:
            raise RuntimeError("checkpoint is missing for epoch {}".format(epoch))
        with result_file.open("rb") as stream:
            predictions = pickle.load(stream)
        if len(predictions) != len(dataset):
            raise RuntimeError(
                "epoch {} prediction count {} != {}".format(
                    epoch, len(predictions), len(dataset)
                )
            )
        result_text, raw_metrics = dataset.evaluation(
            predictions, cfg.CLASS_NAMES, eval_metric="kitti"
        )
        metrics = {
            key.replace("Cyclist_", "Obstacle_"): float(value)
            for key, value in raw_metrics.items()
        }
        ranking_keys = [
            "Pedestrian_3d/moderate_R40",
            "Pedestrian_bev/moderate_R40",
            "Obstacle_3d/moderate_R40",
            "Obstacle_bev/moderate_R40",
        ]
        missing = [key for key in ranking_keys if key not in metrics]
        if missing:
            raise RuntimeError("ranking metrics are missing: {}".format(missing))
        score = sum(metrics[key] for key in ranking_keys) / len(ranking_keys)
        evaluations.append(
            {
                "epoch": epoch,
                "ranking_score": score,
                "ranking_metric": "mean strict moderate R40 3D+BEV for both classes",
                "recall_rcnn": load_recall(log_text, epoch),
                "metrics": metrics,
                "checkpoint": str(checkpoint),
                "result": str(result_file),
                "result_text": result_text,
            }
        )

    best = max(evaluations, key=lambda item: (item["ranking_score"], item["epoch"]))
    best_epoch = best["epoch"]
    ARCHIVE.mkdir(parents=True, exist_ok=True)
    evaluation_dir = ARCHIVE / "evaluation"
    evaluation_dir.mkdir(exist_ok=True)

    archived_checkpoint = ARCHIVE / "best_model_epoch_{}.pth".format(best_epoch)
    shutil.copy2(checkpoints[best_epoch], archived_checkpoint)
    best_link = ARCHIVE / "best_model.pth"
    temporary_link = ARCHIVE / "best_model.pth.tmp"
    temporary_link.unlink(missing_ok=True)
    temporary_link.symlink_to(archived_checkpoint.name)
    os.replace(str(temporary_link), str(best_link))
    shutil.copy2(CONFIG, ARCHIVE / CONFIG.name)
    shutil.copy2(SCRIPT_DIR / "morai_pedestrian_dataset.yaml", ARCHIVE / "morai_pedestrian_dataset.yaml")
    shutil.copy2(training_log, evaluation_dir / "training_and_evaluation.log")
    shutil.copy2(
        Path(best["result"]),
        evaluation_dir / "epoch_{}_result.pkl".format(best_epoch),
    )
    validation_logs = sorted((SCRIPT_DIR / "validation_logs").glob("validation_*.json"))
    if not validation_logs:
        raise RuntimeError("ten-round validation report is missing")
    shutil.copy2(validation_logs[-1], ARCHIVE / "validation_10_rounds.json")

    report_evaluations = []
    for item in evaluations:
        report_item = dict(item)
        report_item.pop("result_text")
        report_evaluations.append(report_item)
    report = {
        "status": "complete",
        "created_at": datetime.now().astimezone().isoformat(),
        "classes": ["Pedestrian", "Obstacle"],
        "train_samples": 24000,
        "validation_samples": 6000,
        "epochs": 40,
        "evaluated_epochs": [item["epoch"] for item in evaluations],
        "selection_policy": best["ranking_metric"],
        "best_epoch": best_epoch,
        "best_ranking_score": best["ranking_score"],
        "evaluations": report_evaluations,
        "source_output": str(OUTPUT),
        "archived_checkpoint": str(archived_checkpoint),
    }
    atomic_write(
        ARCHIVE / "training_summary.json",
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    )

    files_to_hash = [
        archived_checkpoint,
        ARCHIVE / CONFIG.name,
        ARCHIVE / "morai_pedestrian_dataset.yaml",
        ARCHIVE / "validation_10_rounds.json",
        ARCHIVE / "training_summary.json",
        evaluation_dir / "epoch_{}_result.pkl".format(best_epoch),
        evaluation_dir / "training_and_evaluation.log",
    ]
    checksum_text = "".join(
        "{}  {}\n".format(sha256(path), path.relative_to(ARCHIVE))
        for path in files_to_hash
    )
    atomic_write(ARCHIVE / "SHA256SUMS", checksum_text)

    metric_rows = []
    for item in evaluations:
        metrics = item["metrics"]
        metric_rows.append(
            "| {epoch} | {score:.4f} | {p3d:.4f} | {pb:.4f} | {o3d:.4f} | {ob:.4f} |".format(
                epoch=item["epoch"],
                score=item["ranking_score"],
                p3d=metrics["Pedestrian_3d/moderate_R40"],
                pb=metrics["Pedestrian_bev/moderate_R40"],
                o3d=metrics["Obstacle_3d/moderate_R40"],
                ob=metrics["Obstacle_bev/moderate_R40"],
            )
        )
    readme = """# MORAI 사람·장애물 PointPillars 모델

- 상태: 40 epoch 학습 및 마지막 체크포인트 평가 완료
- 클래스: `Pedestrian`, `Obstacle`
- 선택 모델: `checkpoint_epoch_{best_epoch}.pth`
- 선택 점수: {best_score:.4f}
- 학습/검증 데이터: 24,000 / 6,000프레임
- 기존 Car 모델과 완전히 별도 보관됨

## 체크포인트 비교

점수는 두 클래스의 strict moderate R40 3D·BEV AP 네 값의 평균임.

| epoch | 평균 | 사람 3D | 사람 BEV | 장애물 3D | 장애물 BEV |
|---:|---:|---:|---:|---:|---:|
{rows}

전체 지표와 Recall은 `training_summary.json`에 기록했으며, `SHA256SUMS`로
보관 파일 무결성을 검사할 수 있음.
""".format(
        best_epoch=best_epoch,
        best_score=best["ranking_score"],
        rows="\n".join(metric_rows),
    )
    atomic_write(ARCHIVE / "README.md", readme)
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
