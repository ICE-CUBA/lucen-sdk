"""`LucenArm`: the Lucen arm as a LeRobot robot, a thin client of its runtime (needs LeRobot).

`lerobot-record`, `lerobot-teleoperate`, `lerobot-replay` and `lerobot-rollout`
use it unchanged, and so does the Lucen bridge's LeRobot backend: it reads the
same `bus` (:class:`~lerobot_robot_lucen.runtime.RuntimeBus`) a Feetech arm has.
The safety layer stays on the robot: the runtime clamps, slews and disables on
silence; this class only asks. LeRobot's own `max_relative_target` applies too.
"""

from __future__ import annotations

import json
import logging
import math
from functools import cached_property
from pathlib import Path
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots import Robot
from lerobot.robots.utils import ensure_safe_goal_position

from lerobot_robot_lucen.config_lucen_arm import (
    LucenArmConfig,
    LucenArmPairConfig,
    LucenHumanoidArmConfig,
)
from lerobot_robot_lucen.features import (
    BENCH_PORT,
    HUMANOID_ARM_PORT,
    LUCEN_ARM_JOINTS,
    LUCEN_ARM_SOFT_LIMITS,
    motor_features,
)
from lerobot_robot_lucen.runtime import RuntimeBus, RuntimeRefused, with_port
from lerobot_robot_lucen.shared import PrefixedBus, SharedRuntime, session_for
from lerobot_robot_lucen.zero import check_arm, zero_arm

logger = logging.getLogger(__name__)


class LucenArm(Robot):
    config_class = LucenArmConfig
    name = "lucen_arm"
    #: The port of this robot's runtime when `--robot.port` names only a host.
    default_port = 9880

    def __init__(self, config: LucenArmConfig) -> None:
        super().__init__(config)
        self.config = config
        self.bus: RuntimeBus | PrefixedBus
        self.session: SharedRuntime | None = None
        if config.prefix or config.arm_config:
            # A Lucen Arm pair (one bridge for a follower and a leader, shared by this process),
            # or any Lucen arm whose bridge this process starts from its yaml.
            self.session = session_for(
                config.port,
                BENCH_PORT,
                config.arm_config,
                python=config.bridge_python,
                profile=config.bridge_profile,
                grav_scale=config.grav_scale,
            )
            self.bus = PrefixedBus(
                self.session,
                f"{self.name}:{config.prefix}",
                config.prefix,
                config.joints,
                config.limits,
                park_on_disconnect=config.park_on_disconnect,
            )
        else:
            self.bus = RuntimeBus(
                with_port(config.port, self.default_port),
                config.joints,
                config.limits,
                connect_timeout_s=config.connect_timeout_s,
            )
        self.cameras = make_cameras_from_configs(config.cameras)

    # -- what it records -------------------------------------------------------

    @property
    def _motors_ft(self) -> dict[str, type]:
        return motor_features(self.config.joints)

    @property
    def _cameras_ft(self) -> dict[str, tuple[int, int, int]]:
        return {
            name: (int(cfg.height), int(cfg.width), 3) for name, cfg in self.config.cameras.items()
        }

    @cached_property
    def observation_features(self) -> dict[str, Any]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    # -- life ------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()
        self.bus.enable_torque()
        for cam in self.cameras.values():
            cam.connect()
        where = self.session.address if self.session is not None else self.config.port
        logger.info("%s connected to the arm runtime at %s.", self, where)

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        """Zero the arm at its stowed pose when this process knows its bridge's yaml.

        The bridge owns calibration (each joint's sign and offset, in its yaml): with
        `arm_config` this asks for the stowed pose, writes the new offsets there, restarts a
        bridge started here and checks the arm reads zero (:mod:`~lerobot_robot_lucen.zero`).
        Either way it writes `<calibration_dir>/<id>.json` naming the runtime and the joints,
        so a tool that looks for a calibration file (the Lucen bridge) finds this arm's.
        """
        if self.session is not None and self.config.arm_config:
            joints = pick_joints(self.config.joints, self.config.zero_joints)
            zero_session_arms(
                self.session, [(self.config.prefix, joints, str(self))], self.config.arm_config
            )
        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def setup_motors(self) -> None:
        """`lerobot-setup-motors`: the motors' ids are set at the factory; this checks that every
        one answers and reads inside its range (a wrong zero shows here)."""
        if self.session is None or not self.config.arm_config:
            raise RuntimeRefused(
                "Name the bridge's yaml (--robot.arm_config=...) to check the motors."
            )
        check_session_arms(
            self.session, [(self.config.prefix, self.config.joints, str(self))], self.config.limits
        )

    def _load_calibration(self, fpath: Path | None = None) -> None:
        path = fpath or self.calibration_fpath
        self.runtime_calibration = json.loads(Path(path).read_text(encoding="utf-8"))

    def _save_calibration(self, fpath: Path | None = None) -> None:
        path = Path(fpath or self.calibration_fpath)
        path.parent.mkdir(parents=True, exist_ok=True)
        record = {
            "robot_type": self.name,
            "runtime": self.config.port,
            "joints": list(self.config.joints),
            "note": "The arm's runtime owns calibration; this file only names it.",
        }
        path.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")

    def configure(self) -> None:
        return None

    # -- the loop ----------------------------------------------------------------

    def get_observation(self) -> dict[str, Any]:
        q = self.bus.sync_read("Present_Position")
        observation: dict[str, Any] = {f"{joint}.pos": value for joint, value in q.items()}
        for name, cam in self.cameras.items():
            observation[name] = cam.async_read()
        return observation

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        goal = {
            key.removesuffix(".pos"): float(value)
            for key, value in action.items()
            if key.endswith(".pos") and key.removesuffix(".pos") in self.bus.motors
        }
        bad = [joint for joint, value in goal.items() if not math.isfinite(value)]
        if bad:
            # Never forwarded: the runtime's clamp lets NaN through to the motors.
            raise ValueError(f"Refusing an action that is not a number for {', '.join(bad)}.")
        if self.config.max_relative_target is not None:
            present = self.bus.sync_read("Present_Position")
            pairs = {joint: (value, present[joint]) for joint, value in goal.items()}
            goal = ensure_safe_goal_position(pairs, self.config.max_relative_target)
        self.bus.sync_write("Goal_Position", goal)
        return {f"{joint}.pos": value for joint, value in goal.items()}

    def disconnect(self) -> None:
        try:
            self.bus.disconnect()  # with a prefix: parks the arm first
        finally:
            for cam in self.cameras.values():
                cam.disconnect()
        logger.info("%s disconnected; the runtime damps and disables the arm.", self)


class LucenArmPair(Robot):
    """A Lucen Arm pair's two 6+1 arms as one bimanual robot, through one shared bridge session.

    Each side is a :class:`~lerobot_robot_lucen.shared.PrefixedBus` on the same
    session; both are parked, one after the other, before the session ends. Neither
    arm is a leader here: a policy or a bimanual teleoperator drives both.
    """

    config_class = LucenArmPairConfig
    name = "lucen_arm_pair"

    def __init__(self, config: LucenArmPairConfig) -> None:
        super().__init__(config)
        self.config = config
        if len(set(config.arms.values())) != len(config.arms):
            raise ValueError("Each side of lucen_arm_pair names its own runtime prefix.")
        shared = session_for(
            config.port,
            BENCH_PORT,
            config.arm_config,
            python=config.bridge_python,
            profile=config.bridge_profile,
            grav_scale=config.grav_scale,
        )
        self.session = shared
        self.buses = {
            side: PrefixedBus(
                shared,
                f"{self.name}:{prefix}",
                prefix,
                LUCEN_ARM_JOINTS,
                LUCEN_ARM_SOFT_LIMITS,
                park_on_disconnect=config.park_on_disconnect,
            )
            for side, prefix in config.arms.items()
        }
        self.cameras = make_cameras_from_configs(config.cameras)

    @cached_property
    def observation_features(self) -> dict[str, Any]:
        motors = {
            f"{side}_{key}": t
            for side in self.buses
            for key, t in motor_features(LUCEN_ARM_JOINTS).items()
        }
        cams = {name: (int(c.height), int(c.width), 3) for name, c in self.config.cameras.items()}
        return {**motors, **cams}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {
            f"{side}_{key}": t
            for side in self.buses
            for key, t in motor_features(LUCEN_ARM_JOINTS).items()
        }

    @property
    def is_connected(self) -> bool:
        return all(b.is_connected for b in self.buses.values()) and all(
            cam.is_connected for cam in self.cameras.values()
        )

    @property
    def is_calibrated(self) -> bool:
        return all(b.is_calibrated for b in self.buses.values())

    def connect(self, calibrate: bool = True) -> None:
        for bus in self.buses.values():
            bus.connect()
        for cam in self.cameras.values():
            cam.connect()
        logger.info("%s connected to the bench bridge at %s.", self, self.config.port)

    def calibrate(self) -> None:
        """Zero both arms at their stowed pose, one after the other (needs `arm_config`)."""
        if not self.config.arm_config:
            raise RuntimeRefused(
                "Name the bridge's yaml (--robot.arm_config=...) to zero the arms."
            )
        arms = [
            (prefix, list(LUCEN_ARM_JOINTS), f"the {side} arm")
            for side, prefix in self.config.arms.items()
        ]
        picked = pick_joints(list(LUCEN_ARM_JOINTS), self.config.zero_joints)
        zero_session_arms(
            self.session, [(p, picked, name) for p, _, name in arms], self.config.arm_config
        )

    def setup_motors(self) -> None:
        if not self.config.arm_config:
            raise RuntimeRefused(
                "Name the bridge's yaml (--robot.arm_config=...) to check the motors."
            )
        arms = [
            (prefix, list(LUCEN_ARM_JOINTS), f"the {side} arm")
            for side, prefix in self.config.arms.items()
        ]
        limits = {k: list(v) for k, v in LUCEN_ARM_SOFT_LIMITS.items()}
        check_session_arms(self.session, arms, limits)

    def configure(self) -> None:
        return None

    def get_observation(self) -> dict[str, Any]:
        observation: dict[str, Any] = {}
        for side, bus in self.buses.items():
            for joint, value in bus.sync_read("Present_Position").items():
                observation[f"{side}_{joint}.pos"] = value
        for name, cam in self.cameras.items():
            observation[name] = cam.async_read()
        return observation

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        sent: dict[str, Any] = {}
        for side, bus in self.buses.items():
            head = f"{side}_"
            goal = {
                key[len(head) :].removesuffix(".pos"): float(value)
                for key, value in action.items()
                if key.startswith(head) and key.endswith(".pos")
            }
            goal = {joint: value for joint, value in goal.items() if joint in bus.motors}
            bad = [joint for joint, value in goal.items() if not math.isfinite(value)]
            if bad:
                raise ValueError(
                    f"Refusing an action that is not a number for {side} {', '.join(bad)}."
                )
            if self.config.max_relative_target is not None and goal:
                present = bus.sync_read("Present_Position")
                pairs = {joint: (value, present[joint]) for joint, value in goal.items()}
                goal = ensure_safe_goal_position(pairs, self.config.max_relative_target)
            if goal:
                bus.sync_write("Goal_Position", goal)
            sent.update({f"{head}{joint}.pos": value for joint, value in goal.items()})
        return sent

    def disconnect(self) -> None:
        try:
            for bus in self.buses.values():  # one arm at a time: between them nothing is checked
                bus.disconnect()
        finally:
            for cam in self.cameras.values():
                cam.disconnect()
        logger.info("%s disconnected; both arms parked, the bridge damps and disables them.", self)


def pick_joints(joints: list[str], wanted: list[str]) -> list[str]:
    """`wanted` (card names) out of `joints`, in the arm's order; all of them when empty."""
    unknown = [j for j in wanted if j not in joints]
    if unknown:
        raise RuntimeRefused(f"No joint {', '.join(unknown)} on this arm ({', '.join(joints)}).")
    return [j for j in joints if j in wanted] if wanted else list(joints)


def zero_session_arms(
    session: SharedRuntime, arms: list[tuple[str, list[str], str]], arm_config: str
) -> None:
    """Zero each (prefix, joints, name) at the stowed pose, through the session's bridge."""
    bridge = session.bridge
    if session.opened and not session.closed:
        print("The motors turn off now so the arm can be moved by hand: hold it.")
        session.close("zeroing", stop_bridge=False)
    elif bridge is not None:
        bridge.ensure()
    for prefix, joints, name in arms:
        zero_arm(session.address, prefix, joints, arm_config, bridge=bridge, arm_name=name)


def check_session_arms(
    session: SharedRuntime,
    arms: list[tuple[str, list[str], str]],
    limits: dict[str, list[float]],
) -> None:
    if session.bridge is not None:
        session.bridge.ensure()
    problems: list[str] = []
    for prefix, joints, name in arms:
        problems += check_arm(session.address, prefix, joints, limits, arm_name=name)
    if problems:
        raise RuntimeRefused("Not ready: " + "; ".join(problems))
    print("Every motor answers and reads inside its range.")


class LucenHumanoidArm(LucenArm):
    """Lucen Humanoid Arm, `lucen/humanoid-arm`: two arms through one runtime, the same client.

    The runtime is the SDK's bridge under the humanoid arm's config. It serves one client
    at a time, so this type and the SDK's own front ends (lead by hand, mouse drag, the
    sliders, the headset) cannot hold the arm together.
    """

    config_class = LucenHumanoidArmConfig
    name = "lucen_humanoid_arm"
    default_port = HUMANOID_ARM_PORT
