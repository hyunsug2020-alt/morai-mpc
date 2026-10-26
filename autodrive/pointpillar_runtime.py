"""Strict PointPillars runtime for sensor-coordinate XYZI clouds."""
import logging
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = ROOT
OPENPCDET_ROOT = ROOT / "third_party/OpenPCDet"
sys.path.insert(0, str(OPENPCDET_ROOT))
import numpy as np
import torch
import yaml
from easydict import EasyDict
from pcdet.config import merge_new_config
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu

def _merge_dict(base, override):
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge_dict(result[key], value)
        else:
            result[key] = value
    return result


def load_runtime_config(model_config_path, data_config_path):
    """Load the archived model/data YAML without depending on the process cwd."""
    with open(data_config_path, "r", encoding="utf-8") as stream:
        data_document = yaml.safe_load(stream)
    with open(model_config_path, "r", encoding="utf-8") as stream:
        model_document = yaml.safe_load(stream)

    model_data = dict(model_document.get("DATA_CONFIG", {}))
    model_data.pop("_BASE_CONFIG_", None)
    model_document["DATA_CONFIG"] = _merge_dict(data_document, model_data)

    config = EasyDict()
    config.ROOT_DIR = OPENPCDET_ROOT
    config.LOCAL_RANK = 0
    merge_new_config(config, model_document)
    return config


class LivePointCloudDataset(DatasetTemplate):
    """Minimal OpenPCDet dataset adapter for points already held in memory."""

    def __init__(self, dataset_cfg, class_names, logger):
        super().__init__(
            dataset_cfg=dataset_cfg,
            class_names=class_names,
            training=False,
            root_path=PROJECT_ROOT / "datasets" / "morai_openpcdet",
            logger=logger,
        )

    def __len__(self):
        return 0

    def __getitem__(self, _index):
        raise IndexError("live dataset has no indexed samples")

    def prepare_cloud(self, points, frame_id):
        return self.prepare_data(
            data_dict={"points": points, "frame_id": str(frame_id)}
        )


class PointPillarRuntime:
    """Own the CUDA model and expose synchronous NumPy inference."""

    def __init__(self, model_config, data_config, checkpoint, strict_checkpoint=False):
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA GPU is required but torch.cuda.is_available() is false")

        self.logger = logging.getLogger("morai_pointpillar")
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            handler.setFormatter(logging.Formatter("[PointPillars] %(message)s"))
            self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)

        self.config = load_runtime_config(model_config, data_config)
        self.dataset = LivePointCloudDataset(
            self.config.DATA_CONFIG, self.config.CLASS_NAMES, self.logger
        )
        self.model = build_network(
            model_cfg=self.config.MODEL,
            num_class=len(self.config.CLASS_NAMES),
            dataset=self.dataset,
        )
        if strict_checkpoint:
            # OpenPCDet's inference loader permits partial weights. Unified
            # inference must not run with an uninitialized multi-class head.
            self.model.load_params_with_optimizer(
                filename=str(checkpoint), to_cpu=True, optimizer=None,
                logger=self.logger)
        else:
            self.model.load_params_from_file(
                filename=str(checkpoint), logger=self.logger, to_cpu=True
            )
        self.model.cuda()
        self.model.eval()

    def infer(self, points, frame_id):
        prepared = self.dataset.prepare_cloud(points, frame_id)
        batch = self.dataset.collate_batch([prepared])
        load_data_to_gpu(batch)
        with torch.inference_mode():
            predictions, _ = self.model.forward(batch)
        prediction = predictions[0]
        return (
            prediction["pred_boxes"].detach().cpu().numpy(),
            prediction["pred_scores"].detach().cpu().numpy(),
            prediction["pred_labels"].detach().cpu().numpy(),
        )

