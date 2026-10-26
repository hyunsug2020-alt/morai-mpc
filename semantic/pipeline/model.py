#!/usr/bin/env python3
"""EfficientViT backbone with a lightweight multi-scale segmentation head."""

from __future__ import annotations

import timm
import torch
import torch.nn.functional as F
from torch import nn


class ConvNormAct(nn.Sequential):
    def __init__(self, in_channels, out_channels, kernel_size=1, groups=1):
        super().__init__(
            nn.Conv2d(
                in_channels,
                out_channels,
                kernel_size,
                padding=kernel_size // 2,
                groups=groups,
                bias=False,
            ),
            nn.BatchNorm2d(out_channels),
            nn.Hardswish(inplace=True),
        )


class EfficientVitSegmentation(nn.Module):
    def __init__(
        self,
        backbone_name="efficientvit_b1.r256_in1k",
        classes=6,
        head_channels=128,
        pretrained=False,
    ):
        super().__init__()
        self.backbone_name = str(backbone_name)
        self.backbone = timm.create_model(
            self.backbone_name,
            pretrained=bool(pretrained),
            features_only=True,
        )
        channels = self.backbone.feature_info.channels()
        self.projections = nn.ModuleList(
            ConvNormAct(channel, head_channels) for channel in channels
        )
        self.fusion = nn.Sequential(
            ConvNormAct(head_channels * len(channels), head_channels),
            ConvNormAct(head_channels, head_channels, kernel_size=3, groups=head_channels),
            nn.Dropout2d(0.1),
            nn.Conv2d(head_channels, int(classes), kernel_size=1),
        )

    def forward(self, inputs):
        target_size = inputs.shape[-2:]
        features = self.backbone(inputs)
        feature_size = features[0].shape[-2:]
        projected = []
        for projection, feature in zip(self.projections, features):
            feature = projection(feature)
            if feature.shape[-2:] != feature_size:
                feature = F.interpolate(
                    feature,
                    size=feature_size,
                    mode="bilinear",
                    align_corners=False,
                )
            projected.append(feature)
        logits = self.fusion(torch.cat(projected, dim=1))
        return F.interpolate(
            logits,
            size=target_size,
            mode="bilinear",
            align_corners=False,
        )


def build_model(
    backbone_name="efficientvit_b1.r256_in1k",
    classes=6,
    head_channels=128,
    pretrained=False,
):
    return EfficientVitSegmentation(
        backbone_name=backbone_name,
        classes=classes,
        head_channels=head_channels,
        pretrained=pretrained,
    )
