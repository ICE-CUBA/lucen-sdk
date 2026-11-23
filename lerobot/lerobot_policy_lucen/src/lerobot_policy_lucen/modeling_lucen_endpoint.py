"""`LucenEndpointPolicy`: LeRobot's policy interface over a hub endpoint session.

Loading the policy is where consent happens, before LeRobot connects the
robot: :class:`lucen_device.rollout_link.RolloutLink` finds (or asks for) a
person's approval for this computer on this endpoint and opens the session the
driver's ordinary endpoint path opens. Nothing here holds weights.

`select_action` is called once per control tick. While a chunk lasts it hands
out the next action; when it runs out, the tick's observation goes to the
endpoint and the tick waits for the next chunk, the way LeRobot waits for a
local ACT to infer. A chunk that misses its deadline is never applied: the
tick answers a hold (each joint's target is where the robot measures it), and
so does every tick while the link is down. When the session ends (its window,
a person's Stop, a link that does not come back), the next call raises, and
LeRobot's own teardown runs.
"""

from __future__ import annotations

import atexit
import logging
from typing import Any

import numpy as np
import torch
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.pretrained import PreTrainedPolicy

from lerobot_policy_lucen.configuration_lucen_endpoint import LucenEndpointConfig
from lucen_cli.errors import CLIError
from lucen_device.adapters.lerobot_rollout import EndpointStopped
from lucen_device.rollout_link import RolloutLink

logger = logging.getLogger(__name__)

STATE_KEY = "observation.state"


class LucenEndpointPolicy(PreTrainedPolicy):
    config_class = LucenEndpointConfig
    name = "lucen_endpoint"

    def __init__(self, config: LucenEndpointConfig, **kwargs: Any) -> None:
        super().__init__(config)
        self.config: LucenEndpointConfig = config
        if not config.endpoint:
            # Here and not in the config: LeRobot's parser hides a config's own error.
            raise ValueError(
                "Name the hub endpoint: --policy.type=lucen_endpoint --policy.endpoint=ep_..."
            )
        self._link = RolloutLink(
            config.endpoint,
            minutes=config.minutes,
            wait_s=config.wait_s,
            max_run_seconds=config.max_run_seconds,
            log=logger.info,
        )
        try:
            self._channel = self._link.start()
        except CLIError as exc:
            raise RuntimeError(" ".join(filter(None, (exc.message, exc.hint)))) from None
        atexit.register(self._link.finish)
        contract = self._channel.contract
        self._cameras = [c for c in contract.get("cameras") or [] if isinstance(c, dict)]
        self._describe(contract)
        logger.info(
            "Policy from hub endpoint %s: %d actions a chunk at %s Hz; %s",
            config.endpoint,
            int(contract.get("n_action_steps") or contract.get("action_horizon") or 0),
            contract.get("action_rate_hz"),
            ", ".join(c.get("key", "?") for c in self._cameras) or "no camera",
        )

    def _describe(self, contract: dict[str, Any]) -> None:
        """What the endpoint takes and gives, so LeRobot checks the robot against it."""
        inputs: dict[str, PolicyFeature] = {
            STATE_KEY: PolicyFeature(type=FeatureType.STATE, shape=(int(contract["state_dim"]),))
        }
        for camera in self._cameras:
            height, width = camera.get("height"), camera.get("width")
            shape = (3, int(height), int(width)) if height and width else (3, 0, 0)
            inputs[str(camera["key"])] = PolicyFeature(type=FeatureType.VISUAL, shape=shape)
        self.config.input_features = inputs
        self.config.output_features = {
            "action": PolicyFeature(type=FeatureType.ACTION, shape=(int(contract["action_dim"]),))
        }
        self.config.action_feature_names = [str(n) for n in contract.get("action_names") or []]

    @classmethod
    def from_pretrained(
        cls,
        pretrained_name_or_path: Any = None,
        *,
        config: LucenEndpointConfig | None = None,
        **_: Any,
    ) -> LucenEndpointPolicy:
        """There are no weights to load: the endpoint is named by the config."""
        if config is None:
            raise ValueError(
                "Name the hub endpoint: --policy.type=lucen_endpoint --policy.endpoint=ep_..."
            )
        return cls(config)

    # -- LeRobot's policy interface ---------------------------------------------

    def get_optim_params(self) -> dict[str, Any]:
        return {}

    def reset(self) -> None:
        return None

    def forward(self, batch: dict[str, torch.Tensor]) -> tuple[torch.Tensor, dict[str, Any] | None]:
        raise NotImplementedError("a hub endpoint is trained on the hub, not in LeRobot")

    def predict_action_chunk(self, batch: dict[str, torch.Tensor], **kwargs: Any) -> torch.Tensor:
        raise NotImplementedError(
            "lucen_endpoint runs with LeRobot's synchronous inference (--inference.type=sync)"
        )

    @torch.no_grad()  # type: ignore[untyped-decorator]
    def select_action(self, batch: dict[str, Any], **kwargs: Any) -> torch.Tensor:
        state = batch[STATE_KEY][0].detach().to("cpu", dtype=torch.float32).numpy()
        try:
            action = self._channel.next_action(self._observation(batch, state), state)
        except EndpointStopped as exc:
            outcome = self._link.finish(reason=str(exc))
            why = str(exc)
            if outcome is not None and outcome.reason:
                why = outcome.reason + (f": {outcome.detail}" if outcome.detail else "")
            raise RuntimeError(
                f"The hub endpoint session ended ({why}). The rollout stops here."
            ) from exc
        return torch.from_numpy(np.ascontiguousarray(action)).unsqueeze(0)

    def _observation(self, batch: dict[str, Any], state: np.ndarray) -> dict[str, Any]:
        """openpi's observation dict: the state, each camera as uint8 HxWx3, the task."""
        observation: dict[str, Any] = {"observation/state": state}
        for camera in self._cameras:
            key = str(camera["key"])
            image = batch.get(key)
            if image is None:
                raise KeyError(
                    f"the robot gives no `{key}`: name its camera as the policy was trained"
                )
            chw = image[0].detach().to("cpu", dtype=torch.float32).clamp(0.0, 1.0)
            hwc = (chw * 255.0).round().to(torch.uint8).permute(1, 2, 0).contiguous()
            observation[str(camera.get("send_as") or key.replace(".", "/"))] = hwc.numpy()
        task = batch.get("task")
        if isinstance(task, list | tuple):
            task = task[0] if task else ""
        if isinstance(task, str) and task:
            observation["prompt"] = task
        return observation


__all__ = ["LucenEndpointPolicy"]
