"""A leader arm's positions as the follower's targets, holding through a bad reading.

No LeRobot here, so the rule is tested in the workspace: a reading that is
missing or not a finite number holds that joint at the last good target, and a
joint with no good reading yet stops the leader with a sentence. NaN or
infinity never becomes a target.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, MutableMapping, Sequence
from typing import Any

from lerobot_robot_lucen.runtime import RuntimeRefused


def leader_action(
    q: Mapping[str, Any],
    joints: Sequence[str],
    followers: Sequence[str],
    last: MutableMapping[str, float],
) -> dict[str, float]:
    action: dict[str, float] = {}
    for leader, follower in zip(joints, followers, strict=True):
        raw = q.get(leader)
        value: float | None = None
        if isinstance(raw, int | float) and not isinstance(raw, bool) and math.isfinite(raw):
            value = float(raw)
        else:
            value = last.get(follower)
        if value is None:
            raise RuntimeRefused(f"The leader gave no usable position for {leader} yet.")
        action[f"{follower}.pos"] = value
        last[follower] = value
    return action


__all__ = ["leader_action"]
