#!/usr/bin/env python3
"""Select and archive the best far-range pedestrian checkpoint."""

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
OUTPUT = OPENPCDET / "output" / "custom_models" / "pointpillar_pedestrian_far_v2" / "morai_pedestrian_far_v2_30k"
ARCHIVE = REPOSITORY / "trained_models" / "morai_pedestrian_far_v2_pointpillar_30k_2026-08-02"
CONFIG = SCRIPT_DIR / "pointpillar_pedestrian_far_v2.yaml"
DATA_CONFIG = SCRIPT_DIR / "morai_pedestrian_far_v2_dataset.yaml"
PREPARED = REPOSITORY / "datasets" / "morai_pedestrian_far_v2_openpcdet"
RAW = REPOSITORY / "datasets" / "morai_lidar_pedestrian_far_v2_20260802"


def atomic_write(path, text):
    temporary = Path(str(path) + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    os.replace(str(temporary), str(path))


def epoch(path):
    match = re.search(r"epoch_(\d+)", str(path))
    if not match:
        raise RuntimeError("epoch missing from {}".format(path))
    return int(match.group(1))


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    candidate_epochs = set(range(26, 31))
    result_files = sorted(
        (
            path
            for path in (OUTPUT / "eval" / "eval_with_train").glob("epoch_*/val/result.pkl")
            if epoch(path) in candidate_epochs
        ),
        key=epoch,
    )
    found_epochs = {epoch(path) for path in result_files}
    if found_epochs != candidate_epochs:
        raise RuntimeError(
            "completed evaluations for epochs 26-30 required, found {}".format(
                sorted(found_epochs)
            )
        )
    checkpoints = {epoch(path): path for path in (OUTPUT / "ckpt").glob("checkpoint_epoch_*.pth")}
    train_logs = sorted(OUTPUT.glob("train_*.log"), key=lambda path: path.stat().st_mtime)
    if not train_logs:
        raise RuntimeError("training log missing")
    log_text = train_logs[-1].read_text(encoding="utf-8", errors="replace")
    if "End evaluation" not in log_text:
        raise RuntimeError("final evaluation incomplete")

    os.chdir(str(OPENPCDET / "tools"))
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
        number = epoch(result_file)
        checkpoint = checkpoints.get(number)
        if checkpoint is None:
            continue
        with result_file.open("rb") as stream:
            predictions = pickle.load(stream)
        if len(predictions) != len(dataset):
            raise RuntimeError("epoch {} prediction count mismatch".format(number))
        _text, metrics = dataset.evaluation(predictions, cfg.CLASS_NAMES, eval_metric="kitti")
        keys = ["Pedestrian_3d/moderate_R40", "Pedestrian_bev/moderate_R40"]
        missing = [key for key in keys if key not in metrics]
        if missing:
            raise RuntimeError("ranking metrics missing: {}".format(missing))
        score = sum(float(metrics[key]) for key in keys) / len(keys)
        evaluations.append({
            "epoch": number,
            "ranking_score": score,
            "metrics": {key: float(value) for key, value in metrics.items()},
            "checkpoint": str(checkpoint),
            "result": str(result_file),
        })
    if not evaluations:
        raise RuntimeError("no usable evaluation")
    best = max(evaluations, key=lambda item: (item["ranking_score"], item["epoch"]))

    ARCHIVE.mkdir(parents=True, exist_ok=True)
    evaluation_dir = ARCHIVE / "evaluation"
    evaluation_dir.mkdir(exist_ok=True)
    archived = ARCHIVE / "best_model_epoch_{}.pth".format(best["epoch"])
    shutil.copy2(best["checkpoint"], archived)
    temporary_link = ARCHIVE / "best_model.pth.tmp"
    temporary_link.unlink(missing_ok=True)
    temporary_link.symlink_to(archived.name)
    os.replace(str(temporary_link), str(ARCHIVE / "best_model.pth"))
    for path in (CONFIG, DATA_CONFIG):
        shutil.copy2(path, ARCHIVE / path.name)
    shutil.copy2(PREPARED / "preparation_summary.json", ARCHIVE / "preparation_summary.json")
    shutil.copy2(PREPARED / "coverage_report.json", ARCHIVE / "coverage_report.json")
    shutil.copy2(PREPARED / "training_split_distance_report.json", ARCHIVE / "training_split_distance_report.json")
    shutil.copy2(RAW / "raw_verification.json", ARCHIVE / "raw_verification.json")
    shutil.copy2(train_logs[-1], evaluation_dir / "training_and_evaluation.log")
    archived_results = []
    for evaluation in evaluations:
        destination = evaluation_dir / "epoch_{}_result.pkl".format(evaluation["epoch"])
        shutil.copy2(evaluation["result"], destination)
        archived_results.append(destination)
    preparation = json.loads((PREPARED / "preparation_summary.json").read_text(encoding="utf-8"))
    report = {
        "status": "complete",
        "created_at": datetime.now().astimezone().isoformat(),
        "class": "Pedestrian",
        "raw_target_frames": 35000,
        "minimum_prepared_frames": 30000,
        "prepared_frames": preparation["frames"],
        "train_frames": preparation["train_frames"],
        "validation_frames": preparation["validation_frames"],
        "sweeps": preparation["sweeps"],
        "epochs": 30,
        "best_epoch": best["epoch"],
        "selection_metric": "mean Pedestrian strict moderate R40 3D and BEV AP",
        "best_ranking_score": best["ranking_score"],
        "evaluations": evaluations,
        "archived_checkpoint": str(archived),
    }
    atomic_write(ARCHIVE / "training_summary.json", json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n")
    files = [archived, ARCHIVE / CONFIG.name, ARCHIVE / DATA_CONFIG.name, ARCHIVE / "raw_verification.json", ARCHIVE / "preparation_summary.json", ARCHIVE / "coverage_report.json", ARCHIVE / "training_split_distance_report.json", ARCHIVE / "training_summary.json", evaluation_dir / "training_and_evaluation.log", *archived_results]
    atomic_write(ARCHIVE / "SHA256SUMS", "".join("{}  {}\n".format(sha256(path), path.relative_to(ARCHIVE)) for path in files))
    atomic_write(ARCHIVE / "README.md", "# MORAI 원거리 사람 PointPillars v2 30k\n\n- 여러 거리·5개 장소에서 3.5만 raw 프레임을 수집하고, 자세 누락·정지 장면을 제외한 3만 프레임 이상을 자차 보정 3스윕 데이터로 학습함\n- 클래스: `Pedestrian`\n- 선택 epoch: `{}`\n- 기존 차량·사람 v1 모델과 별도 보관됨\n- 센서 연결 후 실행: `roslaunch lidar_detection pedestrian_far_v2_detection.launch`\n".format(best["epoch"]))
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
