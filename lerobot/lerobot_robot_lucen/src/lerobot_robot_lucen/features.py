"""The Lucen arm's names and limits, as LeRobot features (no LeRobot needed here).

Feature names follow the hub robot card's `interface.joint_order` (`lucen/laika-arm`:
`joint1` … `joint6`, then `gripper_joint`), each as `<joint>.pos`. So a dataset recorded
with this plugin names its state and actions the way the hub's embodiment and limit
checks read them. Positions are radians in the robot's own joint coordinates, the
runtime's.

The limits are what the SDK's own clients send the runtime (`Arm/Arm.yaml`): the
`soft_range` where the yaml states one (joint2 stops at 2.9: the model's limit is pi,
where the bridge's angle wraps to -pi), the joint's range elsewhere. The hub card carries
the same `soft_lower` / `soft_upper` for joint2. The runtime clamps every target to the
limits the client names, so these are what a session sends it.
"""

from __future__ import annotations

from typing import Final

#: The hub card's joint order, the gripper last.
LUCEN_ARM_JOINTS: Final[tuple[str, ...]] = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "gripper_joint",
)
LUCEN_ARM_SOFT_LIMITS: Final[dict[str, tuple[float, float]]] = {
    "joint1": (-2.35619, 2.35619),
    "joint2": (0.0, 2.9),
    "joint3": (0.0, 2.61799),
    "joint4": (-1.5708, 1.5708),
    "joint5": (-1.5708, 1.5708),
    "joint6": (-2.0944, 2.0944),
    "gripper_joint": (0.0, 1.9548),
}
#: A Lucen Arm pair's bridge port (the SDK's `Arm/Arm_pair.yaml`: a follower and a leader on
#: one MC02, one bridge, joint names prefixed `follower_` / `leader_`).
BENCH_PORT: Final[int] = 9881
#: The robot on the MouseMouse hub this type records for.
HUB_ROBOT: Final[str] = "lucen/laika-arm"
ROBOT_TYPE: Final[str] = "lucen_arm"
#: The arm's camera, by the hub card's name for it. The camera itself (a device on
#: the robot's computer) is LeRobot's `cameras` config, named this.
LUCEN_ARM_CAMERAS: Final[tuple[str, ...]] = ("wrist",)

# Lucen Humanoid Arm, `lucen/humanoid-arm` (the SDK's `humanoid_arm/humanoid_arm.yaml`): two
# arms of eight motors, left ids 1-8 then right 9-16, each from the shoulder to the gripper.
# Its runtime is the same bridge under that config, on port 9879. The config states no soft
# range for it, so the limits are the joints' ranges, as on the hub card.
LUCEN_HUMANOID_ARM_JOINTS: Final[tuple[str, ...]] = (
    "l_hip_pitch_joint",
    "l_hip_roll_joint",
    "l_hip_yaw_joint",
    "l_knee_joint",
    "l_link4_joint",
    "l_link5_joint",
    "l_link6_joint",
    "l_gripper_joint",
    "r_hip_pitch_joint",
    "r_hip_roll_joint",
    "r_hip_yaw_joint",
    "r_knee_joint",
    "r_link4_joint",
    "r_link5_joint",
    "r_link6_joint",
    "r_gripper_joint",
)
_HUMANOID_ARM_SIDE: Final[dict[str, tuple[float, float]]] = {
    "hip_pitch_joint": (-1.309, 1.309),
    "hip_yaw_joint": (-1.047, 1.047),
    "knee_joint": (-1.9199, 0.0),
    "link4_joint": (-1.5708, 1.5708),
    "link5_joint": (-1.5708, 1.5708),
    "link6_joint": (-2.0944, 2.0944),
    "gripper_joint": (0.0, 1.9548),
}
LUCEN_HUMANOID_ARM_LIMITS: Final[dict[str, tuple[float, float]]] = {
    **{f"l_{k}": v for k, v in _HUMANOID_ARM_SIDE.items()},
    **{f"r_{k}": v for k, v in _HUMANOID_ARM_SIDE.items()},
    # The shoulder's roll ranges mirror each other.
    "l_hip_roll_joint": (-1.222, 0.349),
    "r_hip_roll_joint": (-0.349, 1.222),
}
HUMANOID_ARM_HUB_ROBOT: Final[str] = "lucen/humanoid-arm"
HUMANOID_ARM_TYPE: Final[str] = "lucen_humanoid_arm"
HUMANOID_ARM_PORT: Final[int] = 9879
#: Its cameras by the card's names: a wrist camera on each hand, and the optional top one.
LUCEN_HUMANOID_ARM_CAMERAS: Final[tuple[str, ...]] = ("left_wrist", "right_wrist", "top")
LUCEN_HUMANOID_ARM_OPTIONAL_CAMERAS: Final[tuple[str, ...]] = ("top",)


def motor_features(joints: tuple[str, ...] | list[str]) -> dict[str, type]:
    return {f"{joint}.pos": float for joint in joints}


def camera_features(cameras: dict[str, tuple[int, int]]) -> dict[str, tuple[int, int, int]]:
    """`{name: (height, width)}` → LeRobot's image feature shapes."""
    return {name: (height, width, 3) for name, (height, width) in cameras.items()}


__all__ = [
    "BENCH_PORT",
    "HUB_ROBOT",
    "HUMANOID_ARM_HUB_ROBOT",
    "HUMANOID_ARM_PORT",
    "HUMANOID_ARM_TYPE",
    "LUCEN_ARM_CAMERAS",
    "LUCEN_ARM_JOINTS",
    "LUCEN_ARM_SOFT_LIMITS",
    "LUCEN_HUMANOID_ARM_CAMERAS",
    "LUCEN_HUMANOID_ARM_JOINTS",
    "LUCEN_HUMANOID_ARM_LIMITS",
    "LUCEN_HUMANOID_ARM_OPTIONAL_CAMERAS",
    "ROBOT_TYPE",
    "camera_features",
    "motor_features",
]
