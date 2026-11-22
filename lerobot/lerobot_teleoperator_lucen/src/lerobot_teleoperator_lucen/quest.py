"""A hand's pose to the arm's joints: the Quest teleoperator's mapping and inverse kinematics.

No LeRobot and no headset here, so it is tested on the arm's own model:

- **The pose** comes from a source (:mod:`.quest_server`, the loopback endpoint
  the headset's WebXR page posts to, or a test's). A pose is a 4 by 4 matrix in
  WebXR's frame (x right, y up, z towards the person).
- **The clutch**: nothing moves while the grip (the deadman) is released. When
  it is pressed, the hand's pose and the tool's are captured, and from then on
  the tool follows the hand's displacement, scaled, in the robot's frame
  (x forward, y left, z up). Releasing holds the arm where it is.
- **Inverse kinematics** on the arm's MJCF with MuJoCo: damped least squares on
  the tool's position, its orientation held to the one at the clutch, every
  solution inside the arm's soft limits, and no joint stepping more than
  `max_step_rad` per call. The runtime on the robot clamps and slews again.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Final, Protocol

import numpy as np

#: WebXR (x right, y up, z back) to the robot (x forward, y left, z up).
WEBXR_TO_ROBOT: Final[np.ndarray] = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


class PoseSource(Protocol):
    def latest(self) -> tuple[np.ndarray | None, dict[str, Any]]:
        """The newest 4 by 4 pose (or None) and the buttons that came with it."""


class ArmIK:
    """Forward and inverse kinematics of the arm's tool, on its MJCF."""

    def __init__(
        self,
        mjcf: Path,
        joints: Sequence[str],
        *,
        tool_body: str,
        tool_offset: Sequence[float] = (0.0, 0.0, 0.0),
        limits: Mapping[str, Sequence[float]] | None = None,
    ) -> None:
        import mujoco

        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(mjcf))
        self.data = mujoco.MjData(self.model)
        self.joints = list(joints)
        self.qadr = [int(self.model.jnt_qposadr[self.model.joint(j).id]) for j in self.joints]
        self.dadr = [int(self.model.jnt_dofadr[self.model.joint(j).id]) for j in self.joints]
        self.body = int(self.model.body(tool_body).id)
        self.offset = np.asarray(tool_offset, dtype=float)
        lo, hi = [], []
        for j in self.joints:
            jid = self.model.joint(j).id
            bound = (limits or {}).get(j)
            if bound is not None:
                lo.append(float(bound[0]))
                hi.append(float(bound[1]))
            else:
                lo.append(float(self.model.jnt_range[jid][0]))
                hi.append(float(self.model.jnt_range[jid][1]))
        self.lower, self.upper = np.array(lo), np.array(hi)

    def _set(self, q: np.ndarray) -> None:
        for addr, value in zip(self.qadr, q, strict=True):
            self.data.qpos[addr] = value
        self.mj.mj_kinematics(self.model, self.data)
        self.mj.mj_comPos(self.model, self.data)

    def tool(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """The tool's position and orientation (3 by 3) at joint positions `q`."""
        self._set(q)
        rot = self.data.xmat[self.body].reshape(3, 3).copy()
        return self.data.xpos[self.body] + rot @ self.offset, rot

    def solve(
        self,
        q: np.ndarray,
        position: np.ndarray,
        rotation: np.ndarray | None = None,
        *,
        iterations: int = 40,
        damping: float = 0.05,
        rotation_weight: float = 0.2,
    ) -> np.ndarray:
        """Joint positions near `q` that put the tool at `position` (and `rotation`), in limits."""
        q = np.clip(np.asarray(q, dtype=float).copy(), self.lower, self.upper)
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        for _ in range(iterations):
            here, rot = self.tool(q)
            err = position - here
            rows = [err]
            if rotation is not None:
                skew = rotation @ rot.T
                rows.append(
                    rotation_weight
                    * 0.5
                    * np.array(
                        [skew[2, 1] - skew[1, 2], skew[0, 2] - skew[2, 0], skew[1, 0] - skew[0, 1]]
                    )
                )
            if np.linalg.norm(err) < 1e-4 and rotation is None:
                break
            point = here
            self.mj.mj_jac(self.model, self.data, jacp, jacr, point, self.body)
            j = jacp[:, self.dadr]
            if rotation is not None:
                j = np.vstack([j, rotation_weight * jacr[:, self.dadr]])
            e = np.concatenate(rows)
            step = j.T @ np.linalg.solve(j @ j.T + damping**2 * np.eye(j.shape[0]), e)
            q = np.clip(q + step, self.lower, self.upper)
        return q


@dataclass
class QuestMapper:
    """The clutch, the frame change and the step limit, around :class:`ArmIK`."""

    ik: ArmIK
    q: np.ndarray
    scale: float = 1.0
    max_step_rad: float = 0.05
    gripper: tuple[float, float] | None = None
    deadman: str = "move"
    trigger: str = "scale"
    _origin_hand: np.ndarray | None = None
    _origin_tool: tuple[np.ndarray, np.ndarray] | None = None
    gripper_value: float | None = field(default=None)

    def step(self, pose: np.ndarray | None, buttons: Mapping[str, Any]) -> np.ndarray:
        """The joint targets for this call: held while the deadman is released.

        A pose or a button that is not a finite number reads as the deadman
        released, and the arm holds: `np.clip` lets NaN through, so the clamps
        alone would not stop it. A solution that is not finite is never taken.
        """
        engaged = (
            pose is not None
            and _all_finite(pose, buttons)
            and buttons.get(self.deadman, False) is True
        )
        if not engaged:
            self._origin_hand = None
            return self._held()
        assert pose is not None
        hand = np.asarray(pose, dtype=float)[:3, 3]
        if self._origin_hand is None:
            self._origin_hand = hand.copy()
            self._origin_tool = self.ik.tool(self.q)
        assert self._origin_tool is not None
        start, rot = self._origin_tool
        target = start + self.scale * (WEBXR_TO_ROBOT @ (hand - self._origin_hand))
        solved = self.ik.solve(self.q, target, rot)
        if not np.all(np.isfinite(solved)):
            return self._held()
        stepped = self.q + np.clip(solved - self.q, -self.max_step_rad, self.max_step_rad)
        self.q = np.clip(stepped, self.ik.lower, self.ik.upper)
        if self.gripper is not None and self.trigger in buttons:
            pull = float(np.clip(float(buttons[self.trigger]), 0.0, 1.0))
            low, high = self.gripper
            self.gripper_value = high - pull * (high - low)  # a full pull closes
        return self._held()

    def _held(self) -> np.ndarray:
        """The targets as they stand, refused outright if anything made them not finite."""
        if not np.all(np.isfinite(self.q)) or (
            self.gripper_value is not None and not math.isfinite(self.gripper_value)
        ):
            raise FloatingPointError("A joint target is not a finite number; nothing is sent.")
        return self.q.copy()


def _all_finite(pose: np.ndarray, buttons: Mapping[str, Any]) -> bool:
    if not np.all(np.isfinite(np.asarray(pose, dtype=float))):
        return False
    for value in buttons.values():
        if isinstance(value, bool):
            continue
        try:
            if not math.isfinite(float(value)):
                return False
        except (TypeError, ValueError):
            return False
    return True


__all__ = ["WEBXR_TO_ROBOT", "ArmIK", "PoseSource", "QuestMapper"]
