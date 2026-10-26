"""Runtime reconstruction for morai-semantic-student-v1 checkpoints."""
from __future__ import annotations

import torch
from torch import nn


def conv_bn_act(cin, cout, kernel, stride, groups=1):
    return nn.Sequential(
        nn.Conv2d(cin, cout, kernel, stride=stride, padding=kernel // 2,
                  groups=groups, bias=False),
        nn.BatchNorm2d(cout),
        nn.SiLU(inplace=True),
    )


def depthwise_stage(cin, cout):
    return nn.Sequential(
        nn.Conv2d(cin, cin, 3, stride=2, padding=1, groups=cin, bias=False),
        nn.BatchNorm2d(cin),
        nn.SiLU(inplace=True),
        nn.Conv2d(cin, cout, 1, bias=False),
        nn.BatchNorm2d(cout),
        nn.SiLU(inplace=True),
    )


class MoraiStudent(nn.Module):
    def __init__(self, config):
        super().__init__()
        channels = tuple(config["visual_channels"])
        self.frames = int(config["frames"])
        self.cameras = int(config["cameras"])
        self.ego_steps = int(config["ego_steps"])
        self.route_points = int(config["route_points"])
        self.horizon = int(config["horizon"])
        self.register_buffer("ego_scale", torch.tensor([50., 50., 10., 1., 1., 30.]))
        self.register_buffer("route_scale", torch.tensor([100., 100., 10., 1.]))

        self.visual = nn.Sequential(
            conv_bn_act(3, channels[0], 3, 2),
            depthwise_stage(channels[0], channels[1]),
            depthwise_stage(channels[1], channels[2]),
            depthwise_stage(channels[2], channels[3]),
            depthwise_stage(channels[3], channels[4]),
            nn.AdaptiveAvgPool2d((4, 5)),
            nn.Flatten(),
            nn.Linear(channels[4] * 20, int(config["visual_dim"])),
            nn.SiLU(inplace=True),
            nn.Dropout(float(config["dropout"])),
        )
        self.ego = nn.Sequential(
            nn.Flatten(),
            nn.Linear(self.ego_steps * 6, int(config["state_dim"])),
            nn.SiLU(inplace=True),
            nn.Linear(int(config["state_dim"]), int(config["state_dim"])),
            nn.SiLU(inplace=True),
        )
        self.route = nn.Sequential(
            nn.Flatten(),
            nn.Linear(self.route_points * 4, int(config["state_dim"])),
            nn.SiLU(inplace=True),
            nn.Linear(int(config["state_dim"]), int(config["state_dim"])),
            nn.SiLU(inplace=True),
        )
        visual_total = self.frames * self.cameras * int(config["visual_dim"])
        fusion_input = visual_total + 2 * int(config["state_dim"]) + self.frames * self.cameras
        self.fusion = nn.Sequential(
            nn.Linear(fusion_input, int(config["fusion_dim"])),
            nn.LayerNorm(int(config["fusion_dim"])),
            nn.SiLU(inplace=True),
            nn.Dropout(float(config["dropout"])),
            nn.Linear(int(config["fusion_dim"]), int(config["latent_dim"])),
            nn.SiLU(inplace=True),
        )
        latent = int(config["latent_dim"])
        self.action_head = nn.Linear(latent, len(config.get("actions", range(8))))
        self.path_head = nn.Linear(latent, self.horizon * 3)
        self.scene_head = nn.Linear(latent, int(config["text_dim"]))
        self.concept_head = nn.Linear(latent, len(config.get("concepts", range(12))))

    def forward(self, images, ego, route, image_valid=None):
        # Training tensor convention: batch, time, camera, RGB, height, width.
        batch = images.shape[0]
        visual = self.visual(images.reshape(-1, *images.shape[-3:])).reshape(batch, -1)
        if image_valid is None:
            image_valid = torch.ones(
                batch, self.frames * self.cameras, device=images.device, dtype=images.dtype)
        state = self.ego(ego / self.ego_scale)
        route_state = self.route(route / self.route_scale)
        latent = self.fusion(torch.cat((visual, state, route_state, image_valid), dim=1))
        return {
            "action_logits": self.action_head(latent),
            "path": self.path_head(latent).reshape(batch, self.horizon, 3),
            "scene": self.scene_head(latent),
            "concept_logits": self.concept_head(latent),
        }


def load_checkpoint(path, device):
    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    if checkpoint.get("architecture") != "morai-semantic-student-v1":
        raise ValueError("morai-semantic-student-v1 체크포인트가 아님")
    config = dict(checkpoint["config"])
    config["actions"] = checkpoint["actions"]
    config["concepts"] = checkpoint["concepts"]
    model = MoraiStudent(config)
    model.load_state_dict(checkpoint["model"], strict=True)
    model.eval().to(device)
    return model, checkpoint
