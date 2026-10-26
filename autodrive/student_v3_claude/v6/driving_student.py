"""Bounded, fixed-shape semantic driving student. No pretrained download required."""
from dataclasses import asdict, dataclass

import torch
from torch import nn
from torch.nn import functional as F

ACTIONS = ["KEEP_LANE", "LEFT_CHANGE", "RIGHT_CHANGE", "TURN_LEFT", "TURN_RIGHT", "STOP", "YIELD", "UNKNOWN"]
CONCEPTS = ["vehicle_ahead", "pedestrian", "obstacle", "intersection", "crosswalk", "red_light", "green_light", "stop_sign", "left_turn_lane", "right_turn_lane", "lane_merge", "road_blocked"]


@dataclass
class Config:
    profile: str = "base"
    input_mode: str = "semantic"
    frames: int = 4
    cameras: int = 3
    height: int = 144
    width: int = 256
    classes: int = 19
    ego_steps: int = 16
    route_points: int = 20
    route_features: int = 4        # 4 = [앞, 옆, 0, 유효]; 10 = + HD맵 차선 정보 6칸 (lane_map.LANE_COLUMNS)
    horizon: int = 64
    text_dim: int = 256
    trajectory_dt: float = 0.1
    embedding_dim: int = 8
    visual_channels: tuple = (24, 48, 96, 128, 192)
    visual_dim: int = 192
    state_dim: int = 96
    fusion_dim: int = 384
    latent_dim: int = 256
    dropout: float = 0.1

    @classmethod
    def for_profile(cls, name):
        if name == "base":
            return cls()
        if name == "realtime_hq":
            return cls(
                profile=name,
                embedding_dim=16,
                visual_channels=(48, 96, 192, 320, 512),
                visual_dim=512,
                state_dim=256,
                fusion_dim=1024,
                latent_dim=768,
                dropout=0.15,
            )
        if name == "realtime_max":
            return cls(
                profile=name,
                input_mode="rgb",
                height=180,
                width=320,
                embedding_dim=24,
                visual_channels=(96, 192, 384, 640, 1024),
                visual_dim=1024,
                state_dim=512,
                fusion_dim=3072,
                latent_dim=1536,
                dropout=0.2,
            )
        if name == "semantic_max":
            # v6: 주행 sidecar 시맨틱 마스크 입력 (0–5 클래스, 6 배경, 7 내 차 영역, 8 = 무효 프레임)
            return cls(
                profile=name,
                input_mode="semantic",
                height=180,
                width=320,
                classes=8,
                embedding_dim=16,
                visual_channels=(64, 128, 256, 512, 768),
                visual_dim=768,
                state_dim=512,
                fusion_dim=2048,
                latent_dim=1024,
                dropout=0.2,
            )
        if name == "semantic_max_lane":
            # v6.2: semantic_max + 경로 20점마다 HD맵 차선 정보 (합법 좌/우 차선, 왼쪽 반대차선=노란선, 좌/우 여유폭, 우회 방향)
            c = cls.for_profile("semantic_max")
            c.profile, c.route_features = name, 10
            return c
        raise ValueError(f"Unknown student profile: {name}")


def conv(cin, cout, stride=1):
    return nn.Sequential(nn.Conv2d(cin, cout, 3, stride, 1, bias=False), nn.BatchNorm2d(cout), nn.SiLU())


def separable(cin, cout, stride=1):
    return nn.Sequential(nn.Conv2d(cin, cin, 3, stride, 1, groups=cin, bias=False), nn.BatchNorm2d(cin), nn.SiLU(), nn.Conv2d(cin, cout, 1, bias=False), nn.BatchNorm2d(cout), nn.SiLU())


class DrivingStudent(nn.Module):
    """Inputs: IDs [B,T,C,H,W], ego [B,16,6], route [B,20,4], valid [B,T,C]."""
    def __init__(self, config=None):
        super().__init__()
        self.config = config or Config()
        c = self.config
        channels = tuple(c.visual_channels)
        if c.input_mode == "semantic":
            self.embedding = nn.Embedding(c.classes + 1, c.embedding_dim)
            visual_input = c.embedding_dim
        elif c.input_mode == "rgb":
            self.embedding = None
            visual_input = 3
        else:
            raise ValueError(f"Unsupported input_mode: {c.input_mode}")
        visual = [conv(visual_input, channels[0], 2)]
        visual.extend(separable(cin, cout, 2) for cin, cout in zip(channels, channels[1:]))
        visual.extend((nn.AdaptiveAvgPool2d((5, 4)), nn.Flatten(), nn.Linear(channels[-1] * 5 * 4, c.visual_dim), nn.SiLU()))
        self.visual = nn.Sequential(*visual)
        self.ego = nn.Sequential(nn.Flatten(), nn.Linear(c.ego_steps * 6, c.state_dim), nn.SiLU(), nn.Linear(c.state_dim, c.state_dim), nn.SiLU())
        self.route = nn.Sequential(nn.Flatten(), nn.Linear(c.route_points * c.route_features, c.state_dim), nn.SiLU(), nn.Linear(c.state_dim, c.state_dim), nn.SiLU())
        fused_input = c.frames * c.cameras * (c.visual_dim + 1) + c.state_dim * 2
        self.fusion = nn.Sequential(nn.Linear(fused_input, c.fusion_dim), nn.LayerNorm(c.fusion_dim), nn.SiLU(), nn.Dropout(c.dropout), nn.Linear(c.fusion_dim, c.latent_dim), nn.SiLU())
        self.action_head = nn.Linear(c.latent_dim, len(ACTIONS))
        self.path_head = nn.Linear(c.latent_dim, c.horizon * 3)
        self.scene_head = nn.Linear(c.latent_dim, c.text_dim)
        self.concept_head = nn.Linear(c.latent_dim, len(CONCEPTS))
        self.register_buffer("ego_scale", torch.tensor([50., 50., 10., 1., 1., 30.]))
        # 차선 정보 6칸: 좌/우 합법차선·반대차선 0/1, 좌/우 여유폭 m(10), 우회 방향 -1/0/1
        self.register_buffer("route_scale", torch.tensor([100., 100., 10., 1., 1., 1., 1., 10., 10., 1.][:c.route_features]))

    def forward(self, vision, ego_history, route, camera_valid):
        c = self.config
        batch = vision.shape[0]
        if c.input_mode == "rgb":
            visual_dtype = self.visual[0][0].weight.dtype
            x = vision.reshape(-1, 3, c.height, c.width).to(visual_dtype).div(255.0)
            mean = x.new_tensor((0.485, 0.456, 0.406)).view(1, 3, 1, 1)
            std = x.new_tensor((0.229, 0.224, 0.225)).view(1, 3, 1, 1)
            x = (x - mean) / std
        else:
            ids = vision.reshape(-1, c.height, c.width).long().clamp(0, c.classes)
            x = self.embedding(ids).permute(0, 3, 1, 2).contiguous()
        x = self.visual(x).reshape(batch, c.frames * c.cameras, c.visual_dim)
        valid = camera_valid.reshape(batch, -1, 1).to(x.dtype)
        x = torch.cat([x * valid, valid], -1).flatten(1)
        state = self.ego(ego_history / self.ego_scale)
        map_features = self.route((route / self.route_scale) * route[..., 3:4])
        fused = self.fusion(torch.cat([x, state, map_features], -1))
        path = self.path_head(fused).reshape(batch, c.horizon, 3).cumsum(1) * c.trajectory_dt
        return self.action_head(fused), path, F.normalize(self.scene_head(fused), dim=-1, eps=1e-6), self.concept_head(fused)


def example_inputs(config=None, batch=1, device="cpu"):
    c = config or Config()
    vision_shape = (batch, c.frames, c.cameras, 3, c.height, c.width) if c.input_mode == "rgb" else (batch, c.frames, c.cameras, c.height, c.width)
    return (torch.zeros(*vision_shape, dtype=torch.uint8, device=device), torch.zeros(batch, c.ego_steps, 6, device=device), torch.zeros(batch, c.route_points, c.route_features, device=device), torch.ones(batch, c.frames, c.cameras, device=device))


def checkpoint_payload(model, **extra):
    return {"format_version": 1, "architecture": "morai-semantic-student-v1", "config": asdict(model.config), "model": model.state_dict(), "actions": ACTIONS, "concepts": CONCEPTS, **extra}


def load_checkpoint(path, device="cpu"):
    payload = torch.load(path, map_location="cpu", weights_only=True)
    if payload.get("architecture") != "morai-semantic-student-v1":
        raise ValueError("Unsupported student checkpoint")
    model = DrivingStudent(Config(**payload["config"]))
    model.load_state_dict(payload["model"])
    return model.to(device).eval(), payload
