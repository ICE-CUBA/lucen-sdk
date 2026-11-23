"""The pre/post-processors of `lucen_endpoint`: none.

The endpoint normalises, resizes and unnormalises with the checkpoint's own
statistics on the hub; LeRobot hands the policy the robot's raw observation
and gets back actions already in the robot's units.
"""

from __future__ import annotations

from typing import Any

from lerobot.processor import PolicyAction, PolicyProcessorPipeline
from lerobot.processor.factory import make_policy_processor_pipelines

from lerobot_policy_lucen.configuration_lucen_endpoint import LucenEndpointConfig


def make_lucen_endpoint_pre_post_processors(
    config: LucenEndpointConfig,
    dataset_stats: dict[str, Any] | None = None,
) -> tuple[
    PolicyProcessorPipeline[dict[str, Any], dict[str, Any]],
    PolicyProcessorPipeline[PolicyAction, PolicyAction],
]:
    del config, dataset_stats  # nothing to normalise here
    pipelines: tuple[Any, Any] = make_policy_processor_pipelines(input_steps=[], output_steps=[])
    return pipelines


__all__ = ["make_lucen_endpoint_pre_post_processors"]
