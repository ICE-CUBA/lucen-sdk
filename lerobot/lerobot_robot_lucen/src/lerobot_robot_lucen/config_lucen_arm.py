"""`LucenArmConfig` (`lucen_arm`), `LucenHumanoidArmConfig` (`lucen_humanoid_arm`)"""

from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig

from lerobot_robot_lucen.features import (
    HUMANOID_ARM_PORT,
    LUCEN_ARM_JOINTS,
    LUCEN_ARM_SOFT_LIMITS,
    LUCEN_HUMANOID_ARM_JOINTS,
    LUCEN_HUMANOID_ARM_LIMITS,
)
from lerobot_robot_lucen.runtime import DEFAULT_ADDRESS


@RobotConfig.register_subclass("lucen_arm")
@dataclass(kw_only=True)
class LucenArmConfig(RobotConfig):
    #: Where the arm's runtime listens, HOST:PORT. Named `port` so `--robot.port` means
    #: where the robot is, for LeRobot's tools and the Lucen bridge alike.
    port: str = DEFAULT_ADDRESS
    joints: list[str] = field(default_factory=lambda: list(LUCEN_ARM_JOINTS))
    #: Sent to the runtime in the hello; it clamps every target to them.
    limits: dict[str, list[float]] = field(
        default_factory=lambda: {k: list(v) for k, v in LUCEN_ARM_SOFT_LIMITS.items()}
    )
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    #: LeRobot's own guard: the most a target may move from the present position (rad).
    max_relative_target: float | None = None
    connect_timeout_s: float = 6.0
    #: A Lucen Arm pair (`Arm/Arm_pair.yaml`: one bridge on 9881 for a follower and a leader):
    #: the runtime's name prefix of this arm, `follower_`. Set, the arm shares one session
    #: with every other Lucen arm or leader of this process (`lucen_leader` with its own
    #: prefix) and is parked before it is let go. Empty: its own runtime, as before.
    prefix: str = ""
    #: With a prefix: fold to the stowed pose on disconnect (the arm has no brake; a released
    #: arm falls). Off only for an arm that is held some other way.
    park_on_disconnect: bool = True
    #: The arm's yaml in the Lucen SDK (`~/lucen-sdk/Arm/Arm_pair.yaml`). Set,
    #: the bridge is started here when nothing listens on the yaml's port (and stopped when
    #: the session ends), and `lerobot-calibrate` / `lerobot-setup-motors` zero and check the
    #: arm through it. The yaml's port is then where the arm is; `port` is not read.
    arm_config: str = ""
    #: The Python that runs a bridge started here (default: this one; it needs python-can,
    #: pyserial and MuJoCo for the gravity feed-forward).
    bridge_python: str = ""
    #: Passed to a bridge started here as --grav-scale (default: the yaml's gravity_ff.scale).
    grav_scale: float | None = None
    #: The yaml's gain profile a bridge started here runs (`mirror`, the bridge's default).
    bridge_profile: str = "mirror"
    #: `lerobot-calibrate`: the joints to zero (card names, e.g. ["gripper_joint"]); empty: all.
    zero_joints: list[str] = field(default_factory=list)


@RobotConfig.register_subclass("lucen_arm_pair")
@dataclass(kw_only=True)
class LucenArmPairConfig(RobotConfig):
    """A Lucen Arm pair's two 6+1 arms as one bimanual robot (both follow a policy or a
    teleoperator).

    `arms` names each side's runtime prefix; the default puts the `leader_` arm on the left
    and the `follower_` arm on the right, as a bimanual station does. Features
    are `left_joint1.pos` … `right_gripper_joint.pos`; cameras by the card names
    `left_wrist`, `right_wrist`, `top`.
    """

    port: str = "127.0.0.1:9881"
    arms: dict[str, str] = field(default_factory=lambda: {"left": "leader_", "right": "follower_"})
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    max_relative_target: float | None = None
    park_on_disconnect: bool = True
    #: The arm's yaml in the Lucen SDK (`~/lucen-sdk/Arm/Arm_pair.yaml`). Set,
    #: the bridge is started here when nothing listens on the yaml's port (and stopped when
    #: the session ends), and `lerobot-calibrate` / `lerobot-setup-motors` zero and check the
    #: arm through it. The yaml's port is then where the arm is; `port` is not read.
    arm_config: str = ""
    #: The Python that runs a bridge started here (default: this one; it needs python-can,
    #: pyserial and MuJoCo for the gravity feed-forward).
    bridge_python: str = ""
    #: Passed to a bridge started here as --grav-scale (default: the yaml's gravity_ff.scale).
    grav_scale: float | None = None
    #: The yaml's gain profile a bridge started here runs (`mirror`, the bridge's default).
    bridge_profile: str = "mirror"
    #: `lerobot-calibrate`: the joints to zero (card names, e.g. ["gripper_joint"]); empty: all.
    zero_joints: list[str] = field(default_factory=list)


@RobotConfig.register_subclass("lucen_humanoid_arm")
@dataclass(kw_only=True)
class LucenHumanoidArmConfig(LucenArmConfig):
    """Lucen Humanoid Arm: 16 joints in the hub card's order, its runtime on
    9879. Its cameras, named as the card names them: `left_wrist`, `right_wrist`, and the
    optional `top`."""

    port: str = f"127.0.0.1:{HUMANOID_ARM_PORT}"
    joints: list[str] = field(default_factory=lambda: list(LUCEN_HUMANOID_ARM_JOINTS))
    limits: dict[str, list[float]] = field(
        default_factory=lambda: {k: list(v) for k, v in LUCEN_HUMANOID_ARM_LIMITS.items()}
    )
