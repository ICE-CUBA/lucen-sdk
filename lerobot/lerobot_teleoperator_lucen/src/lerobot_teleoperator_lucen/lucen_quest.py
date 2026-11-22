"""`LucenQuest`: a Meta Quest's hand pose drives the Lucen arm, by IK on its model (needs LeRobot).

The pose arrives at :class:`~.quest_server.QuestPoseServer`: loopback by
default (the headset reaches it over USB with `adb reverse`), a secret per
session, one paired headset, finite numbers only. Holding the grip (the
deadman) engages the clutch; the trigger closes the gripper. The mapping and
the IK are :mod:`.quest`. Every target is held to the arm's soft limits and a
step limit here, and clamped and slewed again by the runtime. Not yet run with
a headset: the button mapping is confirmed with the Quest on the bench.
"""

from __future__ import annotations

import logging
from functools import cached_property
from pathlib import Path
from typing import Any

import numpy as np
from lerobot.teleoperators import Teleoperator

from lerobot_teleoperator_lucen.config_lucen_leader import LucenQuestConfig
from lerobot_teleoperator_lucen.quest import ArmIK, QuestMapper
from lerobot_teleoperator_lucen.quest_server import QuestPoseServer

logger = logging.getLogger(__name__)


class LucenQuest(Teleoperator):
    config_class = LucenQuestConfig
    name = "lucen_quest"

    def __init__(self, config: LucenQuestConfig) -> None:
        super().__init__(config)
        self.config = config
        if not config.mjcf or not Path(config.mjcf).is_file():
            raise ValueError(
                "Name the arm's model with --teleop.mjcf (pull it: lucen pull lucen/laika-arm)."
            )
        self.source: Any = None
        self.mapper: QuestMapper | None = None

    @cached_property
    def action_features(self) -> dict[str, type]:
        names = [
            *self.config.joints,
            *([self.config.gripper_joint] if self.config.gripper_joint else []),
        ]
        return {f"{joint}.pos": float for joint in names}

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.source is not None

    def connect(self, calibrate: bool = True) -> None:
        cfg = self.config
        ik = ArmIK(
            Path(cfg.mjcf),
            cfg.joints,
            tool_body=cfg.tool_body,
            tool_offset=cfg.tool_offset,
            limits=cfg.limits,
        )
        seed = np.array([float(cfg.seed.get(j, 0.0)) for j in cfg.joints])
        gripper = cfg.limits.get(cfg.gripper_joint or "") if cfg.gripper_joint else None
        self.mapper = QuestMapper(
            ik,
            np.clip(seed, ik.lower, ik.upper),
            scale=cfg.scale,
            max_step_rad=cfg.max_step_rad,
            gripper=(float(gripper[0]), float(gripper[1])) if gripper else None,
        )
        self.source = QuestPoseServer(cfg.host, cfg.port, allow_remote=cfg.allow_remote).start()
        print(
            f"Open {self.source.page_url} in the Quest's browser "
            f"(over USB first: adb reverse tcp:{self.source.port} tcp:{self.source.port}).",
            flush=True,
        )

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return None

    def configure(self) -> None:
        return None

    def get_action(self) -> dict[str, Any]:
        assert self.mapper is not None and self.source is not None
        pose, buttons = self.source.latest()
        q = self.mapper.step(pose, buttons)
        action: dict[str, Any] = {
            f"{j}.pos": float(v) for j, v in zip(self.config.joints, q, strict=True)
        }
        if self.config.gripper_joint and self.mapper.gripper_value is not None:
            action[f"{self.config.gripper_joint}.pos"] = self.mapper.gripper_value
        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None

    def disconnect(self) -> None:
        if self.source is not None:
            self.source.stop()
        self.source = None
