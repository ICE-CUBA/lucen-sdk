"""`LucenEndpointConfig` (`--policy.type=lucen_endpoint`): which endpoint, for how long.

There is no hub address here on purpose. A policy config can be somebody else's
file, and the hub it names would receive this computer's credential; the hub is
the one this computer registered with (`lucen_device.rollout_link`).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from lerobot.configs.policies import PreTrainedConfig


@PreTrainedConfig.register_subclass("lucen_endpoint")
@dataclass
class LucenEndpointConfig(PreTrainedConfig):
    #: The hub endpoint that serves the policy (`ep_...`, from `lucen endpoints create`).
    endpoint: str = ""
    #: The minutes the approval request asks a person for, when this run has to ask.
    minutes: int = 10
    #: How long to wait for a person to approve (and for the endpoint to start).
    wait_s: float = 600.0
    #: A local cap on the run, shorter than the approval's window.
    max_run_seconds: float | None = None
    #: The served policy's action names, in its order (filled from the endpoint's contract);
    #: LeRobot maps each action to the robot's feature of that name.
    action_feature_names: list[str] | None = None
    #: The model runs on the hub: nothing is put on a GPU here.
    device: str | None = "cpu"

    @property
    def observation_delta_indices(self) -> None:
        return None

    @property
    def action_delta_indices(self) -> None:
        return None

    @property
    def reward_delta_indices(self) -> None:
        return None

    def get_optimizer_preset(self) -> Any:
        raise NotImplementedError("a hub endpoint is trained on the hub, not in LeRobot")

    def get_scheduler_preset(self) -> None:
        return None

    def validate_features(self) -> None:
        return None


__all__ = ["LucenEndpointConfig"]
