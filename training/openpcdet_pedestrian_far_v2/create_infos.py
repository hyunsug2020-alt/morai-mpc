#!/usr/bin/env python3
"""Create OpenPCDet metadata and the Pedestrian GT database."""

import argparse
import sys
from pathlib import Path

import yaml
from easydict import EasyDict


def main():
    repository = Path(__file__).resolve().parents[2]
    openpcdet = repository / "third_party" / "OpenPCDet"
    sys.path.insert(0, str(openpcdet))
    from pcdet.datasets.custom.custom_dataset import create_custom_infos

    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-config", type=Path, default=Path(__file__).resolve().parent / "morai_pedestrian_far_v2_dataset.yaml")
    parser.add_argument("--data-path", type=Path, default=repository / "datasets" / "morai_pedestrian_far_v2_openpcdet")
    parser.add_argument("--workers", type=int, default=4)
    arguments = parser.parse_args()
    config = EasyDict(yaml.safe_load(arguments.dataset_config.read_text(encoding="utf-8")))
    path = arguments.data_path.resolve()
    create_custom_infos(
        dataset_cfg=config,
        class_names=["Pedestrian"],
        data_path=path,
        save_path=path,
        workers=max(1, arguments.workers),
    )


if __name__ == "__main__":
    main()
