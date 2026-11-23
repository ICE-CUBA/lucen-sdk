"""A Lucen Arm pair's leader, held by torque and moved by hand, on the shared session (no LeRobot).

On a Lucen Arm pair (`Arm/Arm_pair.yaml`) the leader is not a limp arm a hand moves in
watch mode: it is as heavy as the follower and would fall. It holds itself with
the position loop and the bridge's gravity feed-forward; when a hand pulls a joint
off its target by more than a threshold, the target follows the hand, and when the
hand lets go (or pulls back half a degree) the joint stops where it is. The
follower copies the leader's measured angles, speed-capped, and a step that would
make it hit itself in the model is not taken.

That logic is the Lucen SDK's, `LeadFollow` in its `tools/arm_lead_follow.py` (the same
checkout as the bridge and the yaml). It is imported
from there, not copied: the thresholds, the gravity learning and the collision
check are tuned on the real arms and live with them. This module only runs it at
its own 50 Hz on the shared session: the leader's targets go straight to the bridge;
the follower's are handed to LeRobot as the action, which the follower robot
(`lucen_arm --robot.prefix=follower_`) sends — so what is recorded is what the
follower was told. The leader's joints are claimed without the session's leash; the
follower's keep it, and the targets LeadFollow computes for them are vouched for
(`SharedRuntime.vouch`), so they alone go out with their gravity compensation intact.
"""

from __future__ import annotations

import importlib.util
import logging
import os
import sys
import threading
import time
from collections.abc import Callable, Mapping
from pathlib import Path
from types import ModuleType
from typing import Any, Final

from lerobot_robot_lucen.runtime import RuntimeRefused
from lerobot_robot_lucen.sdk import sdk_tool
from lerobot_robot_lucen.shared import SharedRuntime

logger = logging.getLogger(__name__)

#: LeadFollow's own loop rate (`LOOP_HZ` in arm_lead_follow.py).
LOOP_HZ: Final[float] = 50.0
#: How long `connect` waits for the gravity calibration's two poses and the way home.
CALIBRATION_TIMEOUT_S: Final[float] = 90.0


def load_arm_config(path: str | os.PathLike[str]) -> dict[str, Any]:
    import yaml

    config_path = Path(path).expanduser()
    if not config_path.is_file():
        raise RuntimeRefused(
            f"No arm configuration at {config_path}. Name the arm's yaml in the Lucen SDK, "
            "e.g. --teleop.arm_config=~/lucen-sdk/Arm/Arm_pair.yaml."
        )
    loaded = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict) or "joints" not in loaded:
        raise RuntimeRefused(f"{config_path} is not a bridge configuration (no joints).")
    return loaded


def load_lead_follow(config_path: str | os.PathLike[str]) -> ModuleType:
    """`tools/arm_lead_follow.py` of the Lucen SDK the yaml belongs to."""
    source = sdk_tool(config_path, "arm_lead_follow.py")
    tools = str(source.parent)
    if tools not in sys.path:
        sys.path.insert(0, tools)  # it imports arm_jog beside it
    spec = importlib.util.spec_from_file_location("lucen_bench_arm_lead_follow", source)
    if spec is None or spec.loader is None:
        raise RuntimeRefused(f"Cannot load {source}.")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not hasattr(module, "LeadFollow"):
        raise RuntimeRefused(f"{source} has no LeadFollow.")
    return module


class BenchLeader:
    """LeadFollow at 50 Hz on the shared session; `follower_action()` is the follower's target."""

    def __init__(
        self,
        shared: SharedRuntime,
        cfg: Mapping[str, Any],
        lead_follow_module: Any,  # the module: `LeadFollow` and `out_of_range`
        *,
        user: str,
        leader: str = "leader",
        band: float = 0.03,
        retreat: float = 0.008,
        vel_max: float = 1.0,
        collide: bool = True,
        profile: str = "mirror",
        range_check: bool = True,
        park_on_disconnect: bool = True,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.shared = shared
        self.park_on_disconnect = park_on_disconnect
        self.user = user
        self.module = lead_follow_module
        self.range_check = range_check
        self.clock = clock
        try:
            self.lf = lead_follow_module.LeadFollow(
                dict(cfg),
                leader=leader,
                band=band,
                retreat=retreat,
                vel_max=vel_max,
                collide=collide,
                profile=profile,
            )
        except SystemExit as exc:  # LeadFollow says what is wrong with the yaml this way
            raise RuntimeRefused(f"LeadFollow refused the configuration: {exc}") from None
        lf = self.lf
        self.leader_prefix = lf.pre[lf.leader]
        self.follower_prefix = lf.pre[lf.follower]
        self.leader_names = [n for n in lf.names if n.startswith(self.leader_prefix)]
        self.follower_names = [n for n in lf.names if n.startswith(self.follower_prefix)]
        self.home = {n: float(v) for n, v in (cfg.get("home_pose") or {}).items()}
        self.rest = dict((cfg.get("routine") or {}).get("rest") or {}) or None
        # LeadFollow bounds its own targets (speed cap, collision pre-check, gripper lead), as in
        # the SDK's lead-follow tool, and adds the gravity sag it learns to every one of them, the
        # follower's too; a session leash would cut that compensation (with the bridge at 0.35 of
        # the model's gravity the hint read +1.00, its cap, for +0.65). The leader's joints are
        # this teleoperator's alone: claimed without the leash. The follower's keep it: what
        # LeadFollow computes for them is vouched for in `_run` and goes out as it is, and any
        # other target there (a policy's, while this leader is connected) is leashed.
        others = [n for n in lf.names if n not in self.leader_names]
        shared.claim(
            user,
            self.leader_names,
            lf.limits,
            grippers=[n for n in lf.grippers if n in self.leader_names],
            leash=False,
        )
        shared.claim(user, others, lf.limits, grippers=[n for n in lf.grippers if n in others])
        self._lock = threading.Lock()
        self._follower: dict[str, float] = {}
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.error: str | None = None
        self.refusal: str | None = None
        self.connected = False

    def connect(self, gravity_calibration: bool = False, *, defer_refusal: bool = False) -> None:
        """Take over both arms where they stand. A joint far outside its range refuses the
        take-over; with `defer_refusal` the refusal waits for the first `follower_action`, so
        a caller may zero the arm first (LeRobot's calibrate connects with calibrate=False)."""
        self.refusal = None
        self.shared.open()
        q = self.shared.positions(self.lf.names)
        if self.range_check:
            bad = self.module.out_of_range(q, self.lf.limits)
            if bad:
                rows = "; ".join(
                    f"{n} at {v:+.2f} outside [{lo:+.2f}, {hi:+.2f}]" for n, v, lo, hi in bad
                )
                # Not one target was sent: end the session now, so the bridge damps and
                # disables at once, as the SDK's tool does (not when the process exits).
                self.shared.close(
                    "refused: a joint reads far outside its range", stop_bridge=not defer_refusal
                )
                message = (
                    "Not taking over: these joints read more than 0.3 rad outside their range, "
                    f"so a zero or a sign is wrong ({rows}). Zero the arm: lerobot-calibrate with "
                    "the same --teleop flags (or tools/calib_arm.py --zero-here <joint> with the "
                    "bridge stopped; a gripper's zero is fully open)."
                )
                if defer_refusal:
                    self.refusal = message
                    return
                raise RuntimeRefused(message)
        self.lf.start(q)
        if self.park_on_disconnect:  # parked before the session ends, whoever ends it
            self.shared.hold(self.user, self.leader_prefix, self.home, self.rest)
        follower = {n: self.lf.tgt[n] for n in self.follower_names if n in self.lf.tgt}
        self.shared.vouch(follower)
        with self._lock:
            self._follower = follower
        if gravity_calibration:
            self.lf.calibrate()  # both arms go to two poses, hold a few seconds, come home
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="lucen-bench-leader", daemon=True)
        self._thread.start()
        self.connected = True
        if gravity_calibration:
            t0 = self.clock()
            deadline = t0 + CALIBRATION_TIMEOUT_S
            while self.lf.mode == "pose" and self.clock() < deadline and not self.error:
                time.sleep(0.05)
            if self.error:
                raise RuntimeRefused(self.error)
            logger.info("%s", self.calibration_summary(self.clock() - t0))

    def calibration_summary(self, seconds: float) -> str:
        """What the gravity calibration learned, as the SDK's tool reports it."""
        lf = self.lf
        learned = lf.learned() if hasattr(lf, "learned") else None
        joints = len(getattr(lf, "all_arm", ()) or ())
        hint = lf.ff_hint() if hasattr(lf, "ff_hint") else None
        text = f"Gravity calibration done in {seconds:.1f} s"
        if learned is not None:
            text += f": learned {learned}/{joints} joints"
        if hint is not None and abs(hint) > 0.03:
            text += f"; feed-forward hint {hint:+.2f} (add it to the bridge's --grav-scale)"
        return text + "."

    def _run(self) -> None:
        dt = 1.0 / LOOP_HZ
        last = self.clock()
        while not self._stop.is_set():
            tick = self.clock()
            try:
                q = self.shared.positions(self.lf.names)
                cmd = self.lf.tick(q, max(1e-3, min(0.1, tick - last)))
                last = tick
                follower = {n: cmd[n] for n in self.follower_names if n in cmd}
                # Vouched for before anything can send it (this loop in pose mode, or LeRobot
                # through the follower robot): LeadFollow's own follower targets, and only
                # those, go out without the follower's leash.
                self.shared.vouch(follower)
                if self.lf.mode == "pose":  # calibrating, or going to a pose: drive both arms
                    self.shared.set_target(cmd)
                else:
                    self.shared.set_target({n: cmd[n] for n in self.leader_names if n in cmd})
                with self._lock:
                    self._follower = follower
            except RuntimeRefused as exc:
                self.error = str(exc)
                self.shared.forget(self.follower_names)
                return
            except Exception as exc:
                self.error = f"the leader loop stopped: {type(exc).__name__}: {exc}"
                self.shared.forget(self.follower_names)
                return
            time.sleep(max(0.0, dt - (self.clock() - tick)))

    def follower_action(self) -> dict[str, float]:
        """The follower's target, runtime names (`follower_joint1` …)."""
        if self.refusal:
            raise RuntimeRefused(self.refusal)
        if self.error:
            raise RuntimeRefused(self.error)
        with self._lock:
            return dict(self._follower)

    def disconnect(self, park: bool | None = None) -> None:
        park = self.park_on_disconnect if park is None else park
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self.shared.forget(self.follower_names)  # the follower's present target is leashed again
        try:
            if park and self.connected and self.shared.alive():
                self.shared.park(self.leader_prefix, self.home, self.rest)
        finally:
            self.connected = False
            self.shared.release(self.user)


__all__ = ["BenchLeader", "load_arm_config", "load_lead_follow"]
