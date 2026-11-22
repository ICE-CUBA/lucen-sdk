"""`LucenLeader`: a Lucen leader arm (needs LeRobot).

Two kinds of leader:

- its own runtime, watched (`arm_config` empty): the runtime enables no motor, a
  person moves the limp arm, and its positions are the follower's targets, joint for
  joint (`follower_joints`). Nothing here can drive that leader.
- a Lucen Arm pair (`arm_config` names `Arm/Arm_pair.yaml`): the leader is as heavy as
  the follower and holds itself by torque; the SDK's LeadFollow lets a hand move
  it and stops it where the hand lets go, and the follower's target is the action
  (:mod:`~lerobot_teleoperator_lucen.bench_leader`). It shares one bridge session with
  the follower robot, `lucen_arm --robot.prefix=follower_`.
"""

from __future__ import annotations

import json
from functools import cached_property
from pathlib import Path
from typing import Any

from lerobot.teleoperators import Teleoperator
from lerobot_robot_lucen.features import BENCH_PORT
from lerobot_robot_lucen.runtime import RuntimeLink, RuntimeRefused
from lerobot_robot_lucen.shared import session_for
from lerobot_robot_lucen.zero import check_arm, zero_arm

from lerobot_teleoperator_lucen.bench_leader import BenchLeader, load_arm_config, load_lead_follow
from lerobot_teleoperator_lucen.config_lucen_leader import LucenLeaderConfig
from lerobot_teleoperator_lucen.leader_map import leader_action


class LucenLeader(Teleoperator):
    config_class = LucenLeaderConfig
    name = "lucen_leader"

    def __init__(self, config: LucenLeaderConfig) -> None:
        super().__init__(config)
        self.config = config
        if len(config.joints) != len(config.follower_joints):
            raise ValueError("joints and follower_joints name the same number of joints.")
        self._last: dict[str, float] = {}
        self.bench: BenchLeader | None = None
        self.link: RuntimeLink | None = None
        if config.arm_config:
            cfg = load_arm_config(config.arm_config)
            profiles = cfg.get("safety_profiles") or {}
            if config.safety not in profiles:
                raise RuntimeRefused(
                    f"No safety profile {config.safety!r} in {config.arm_config} "
                    f"(it has {', '.join(profiles) or 'none'})."
                )
            session = session_for(
                config.port,
                BENCH_PORT,
                config.arm_config,
                python=config.bridge_python,
                profile=config.profile,
                grav_scale=config.grav_scale,
            )
            self.bench = BenchLeader(
                session,
                cfg,
                load_lead_follow(config.arm_config),
                user=f"{self.name}:{config.leader}",
                leader=config.leader,
                band=config.band,
                retreat=config.retreat,
                vel_max=float(profiles[config.safety]["vel_max"]),
                collide=config.collide,
                profile=config.profile,
                range_check=config.range_check,
                park_on_disconnect=config.park_on_disconnect,
            )
            prefix = self.bench.follower_prefix
            self._follower_names = {
                runtime: runtime[len(prefix) :] for runtime in self.bench.follower_names
            }
            stray = [
                card for card in self._follower_names.values() if card not in config.follower_joints
            ]
            if stray:
                raise RuntimeRefused(
                    f"The follower in {config.arm_config} has joints this teleoperator does not "
                    f"name: {', '.join(stray)}."
                )
        else:
            self.link = RuntimeLink(
                config.port, joints=config.joints, connect_timeout_s=config.connect_timeout_s
            )

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in self.config.follower_joints}

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        if self.bench is not None:
            return self.bench.connected and self.bench.shared.alive()
        assert self.link is not None
        return self.link.connected

    def connect(self, calibrate: bool = True) -> None:
        if self.bench is not None:
            # calibrate=False (LeRobot's calibrate script, the Lucen bridge): a joint far out
            # of range is refused at the first action instead, so it can be zeroed first.
            self.bench.connect(
                gravity_calibration=self.config.gravity_calibration and calibrate,
                defer_refusal=not calibrate,
            )
            return
        assert self.link is not None
        self.link.open("watch")

    @property
    def is_calibrated(self) -> bool:
        return True  # the leader's runtime owns its calibration

    def calibrate(self) -> None:
        """Bench: zero the leader at its stowed pose (the yaml's offsets, through its bridge)."""
        if self.bench is None:
            return None
        bench, session = self.bench, self.bench.shared
        if session.opened and not session.closed:
            print("The motors turn off now so the leader can be moved by hand: hold it.")
            session.close("zeroing", stop_bridge=False)
        elif session.bridge is not None:
            session.bridge.ensure()
        bench.connected = False
        joints = [n[len(bench.leader_prefix) :] for n in bench.leader_names]
        if self.config.zero_joints:
            unknown = [j for j in self.config.zero_joints if j not in joints]
            if unknown:
                raise RuntimeRefused(f"No joint {', '.join(unknown)} on the leader.")
            joints = [j for j in joints if j in self.config.zero_joints]
        zero_arm(
            session.address,
            bench.leader_prefix,
            joints,
            self.config.arm_config,
            bridge=session.bridge,
            arm_name="the leader",
        )
        return None

    def setup_motors(self) -> None:
        """Bench: every leader motor answers and reads inside its range (ids are factory-set)."""
        if self.bench is None:
            raise RuntimeRefused(
                "Name the arm's yaml (--teleop.arm_config=...) to check the motors."
            )
        bench, session = self.bench, self.bench.shared
        if session.bridge is not None:
            session.bridge.ensure()
        joints = [n[len(bench.leader_prefix) :] for n in bench.leader_names]
        limits = {j: list(bench.lf.limits[bench.leader_prefix + j]) for j in joints}
        problems = check_arm(
            session.address, bench.leader_prefix, joints, limits, arm_name="the leader"
        )
        if problems:
            raise RuntimeRefused("Not ready: " + "; ".join(problems))
        print("Every leader motor answers and reads inside its range.")

    def _load_calibration(self, fpath: Path | None = None) -> None:
        path = fpath or self.calibration_fpath
        self.runtime_calibration = json.loads(Path(path).read_text(encoding="utf-8"))

    def configure(self) -> None:
        return None

    def get_action(self) -> dict[str, Any]:
        """The follower's targets. Bench: LeadFollow's; watched: the leader's joints, a reading
        that is not finite holding the last good one."""
        if self.bench is not None:
            target = self.bench.follower_action()
            return {
                f"{self._follower_names[n]}.pos": v
                for n, v in target.items()
                if n in self._follower_names
            }
        assert self.link is not None
        q = self.link.positions()  # already finite: the client drops anything else
        return dict(leader_action(q, self.config.joints, self.config.follower_joints, self._last))

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None

    def disconnect(self) -> None:
        if self.bench is not None:
            self.bench.disconnect()
            return
        assert self.link is not None
        self.link.close()
