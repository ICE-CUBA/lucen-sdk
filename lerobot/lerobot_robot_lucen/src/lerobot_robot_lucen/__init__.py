"""`lerobot_robot_lucen`: Lucen robots as LeRobot robots (LeRobot's third-party plugin convention).

LeRobot imports every installed distribution whose name starts with
`lerobot_robot_`, which registers `lucen_arm` (Lucen Arm, `lucen/laika-arm`),
`lucen_humanoid_arm` (Lucen Humanoid Arm, `lucen/humanoid-arm`) and `lucen_arm_pair`
(a Lucen Arm pair's two 6+1 arms as one bimanual robot) here; `lerobot-record
--robot.type=lucen_arm --robot.port=HOST:PORT` then works unchanged. Each arm is
a thin client of the robot's own runtime (:mod:`.runtime`): the motors, their
limits and the stop stay on the robot.

`runtime`, `features` and `fake_runtime` need no LeRobot; the LeRobot classes
load only where LeRobot is installed.
"""

from lerobot_robot_lucen.features import (
    HUB_ROBOT,
    HUMANOID_ARM_HUB_ROBOT,
    LUCEN_ARM_JOINTS,
    LUCEN_ARM_SOFT_LIMITS,
    LUCEN_HUMANOID_ARM_JOINTS,
    LUCEN_HUMANOID_ARM_LIMITS,
)
from lerobot_robot_lucen.runtime import RuntimeBus, RuntimeLink, RuntimeRefused
from lerobot_robot_lucen.shared import PrefixedBus, SharedRuntime

__all__ = [
    "HUB_ROBOT",
    "HUMANOID_ARM_HUB_ROBOT",
    "LUCEN_ARM_JOINTS",
    "LUCEN_ARM_SOFT_LIMITS",
    "LUCEN_HUMANOID_ARM_JOINTS",
    "LUCEN_HUMANOID_ARM_LIMITS",
    "PrefixedBus",
    "RuntimeBus",
    "RuntimeLink",
    "RuntimeRefused",
    "SharedRuntime",
]

try:
    from lerobot_robot_lucen.config_lucen_arm import (
        LucenArmConfig,
        LucenArmPairConfig,
        LucenHumanoidArmConfig,
    )
    from lerobot_robot_lucen.lucen_arm import LucenArm, LucenArmPair, LucenHumanoidArm
except ModuleNotFoundError as exc:  # LeRobot is not installed here: only the client is
    if not (exc.name or "").startswith("lerobot"):
        raise
else:
    __all__ += [
        "LucenArm",
        "LucenArmConfig",
        "LucenArmPair",
        "LucenArmPairConfig",
        "LucenHumanoidArm",
        "LucenHumanoidArmConfig",
    ]
