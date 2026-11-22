"""`LucenLeaderConfig` (`lucen_leader`) and `LucenQuestConfig` (`lucen_quest`), needs LeRobot."""

from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.teleoperators import TeleoperatorConfig
from lerobot_robot_lucen.features import LUCEN_ARM_JOINTS, LUCEN_ARM_SOFT_LIMITS


@TeleoperatorConfig.register_subclass("lucen_leader")
@dataclass(kw_only=True)
class LucenLeaderConfig(TeleoperatorConfig):
    #: The leader arm's runtime, HOST:PORT. It is only ever watched: a hand moves it.
    #: 9881, not the humanoid arm's 9879, so the two can run on one computer.
    port: str = "127.0.0.1:9881"
    #: The leader's joint names, in its runtime's configuration.
    joints: list[str] = field(default_factory=lambda: list(LUCEN_ARM_JOINTS))
    #: The follower joint each leader joint drives, in order (default: the same names).
    follower_joints: list[str] = field(default_factory=lambda: list(LUCEN_ARM_JOINTS))
    connect_timeout_s: float = 6.0
    #: A Lucen Arm pair: the arm's yaml in the Lucen SDK (`~/lucen-sdk/Arm/Arm_pair.yaml`).
    #: Set, the leader is held by torque and moved by hand through the SDK's LeadFollow
    #: (`tools/arm_lead_follow.py`), on the
    #: session it shares with `lucen_arm --robot.prefix=follower_`; the action is the
    #: follower's target. Empty: a leader with its own runtime, watched limp, as before.
    arm_config: str = ""
    #: Which arm of the yaml's `lead_follow.arms` the hand moves.
    leader: str = "leader"
    #: LeadFollow's thresholds (rad): how far a hand pulls a joint before the target follows
    #: it, and how far back counts as letting go. Its own defaults.
    band: float = 0.03
    retreat: float = 0.008
    #: The yaml's `safety_profiles` entry whose `vel_max` caps the follower (slow = 1 rad/s).
    safety: str = "slow"
    #: The bridge's gain profile, for LeadFollow's thresholds (the bridge runs `mirror`).
    profile: str = "mirror"
    #: Skip a follower step that would make it hit itself in the model (needs MuJoCo).
    collide: bool = True
    #: At connect, both arms go to the yaml's two calibration poses, hold, and come home
    #: (about 20 s) so LeadFollow learns what the gravity feed-forward misses: the leader is
    #: then much lighter to move. They move on their own: clear the bench first.
    gravity_calibration: bool = False
    #: Refuse to take over when a joint reads more than 0.3 rad outside its range (a zero or
    #: a sign is wrong, and the target clamp would drive it back on its own).
    range_check: bool = True
    #: Fold the leader to its stowed pose before letting it go (it has no brake).
    park_on_disconnect: bool = True
    #: A bridge started here from `arm_config` (when nothing listens on its port): the Python
    #: that runs it (default: this one) and its --grav-scale (default: the yaml's).
    bridge_python: str = ""
    grav_scale: float | None = None
    #: `lerobot-calibrate`: the joints to zero (card names, e.g. ["gripper_joint"]); empty: all.
    zero_joints: list[str] = field(default_factory=list)


@TeleoperatorConfig.register_subclass("lucen_quest")
@dataclass(kw_only=True)
class LucenQuestConfig(TeleoperatorConfig):
    #: The arm's MJCF (`lucen pull lucen/laika-arm`): the model the hand's pose is solved on.
    mjcf: str = ""
    joints: list[str] = field(default_factory=lambda: list(LUCEN_ARM_JOINTS[:-1]))
    gripper_joint: str | None = "gripper_joint"
    limits: dict[str, list[float]] = field(
        default_factory=lambda: {k: list(v) for k, v in LUCEN_ARM_SOFT_LIMITS.items()}
    )
    #: The tool frame: Arm/Arm.yaml's end effector (link6, 14 cm along -x, between the
    #: fingers; the yaml calls the point a placeholder until the gripper's TCP is measured).
    tool_body: str = "link6"
    tool_offset: list[float] = field(default_factory=lambda: [-0.14, 0.0, 0.0])
    #: Where the teleoperator believes the arm starts (Arm/Arm.yaml `ready_pose`).
    seed: dict[str, float] = field(
        default_factory=lambda: {"joint2": 1.3, "joint3": 2.17, "joint4": -0.87}
    )
    scale: float = 1.0
    max_step_rad: float = 0.05
    #: Where the headset's poses arrive. Loopback by default: the headset reaches it over
    #: USB with `adb reverse`. Another address needs `allow_remote` and is said loudly.
    host: str = "127.0.0.1"
    port: int = 8777
    allow_remote: bool = False
