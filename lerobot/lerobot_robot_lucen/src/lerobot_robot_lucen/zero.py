"""Zero a Lucen arm at its stowed pose, and check that every motor answers (no LeRobot needed).

`lerobot-calibrate` and `lerobot-setup-motors` land here for the Lucen arms. SO-101
needs a sweep of every joint because its servos count relative to where they powered
up; a RobStride motor's encoder is absolute within a turn, so one pose is enough: the
stowed pose (upper arm down on its stop, forearm folded back on top of it, wrist
straight, gripper fully open), the zero pose of every Lucen Arm (`Arm/README.md`).

The math is the SDK's: the bridge reports `q = sign · wrap(a - offset)` for a motor
angle `a` (`bridge.to_sim`), and `tools/calib_arm.py --zero-here` sets the offset
to `wrap(a)` with the arm at zero. So the new offset is `wrap(offset + sign · q)` with q
read through the bridge in watch mode (the motors limp, nothing enabled), which works
with the bridge running. The yaml is written by `calib_arm.patch_yaml` from the Lucen
SDK (one line per joint, a `.bak` beside it), and the bridge is restarted so it
loads the new zero, then read again.
"""

from __future__ import annotations

import importlib.util
import math
import re
import shutil
import statistics
import sys
import time
from collections.abc import Callable, Iterable, Mapping
from pathlib import Path
from types import ModuleType
from typing import Any, Final

from lerobot_robot_lucen.bridge import ManagedBridge
from lerobot_robot_lucen.runtime import RuntimeLink, RuntimeRefused
from lerobot_robot_lucen.sdk import sdk_tool

#: Positions are read this long and the median taken (calib_arm takes the median of five).
SAMPLE_S: Final[float] = 0.6
#: After zeroing, every joint should read within this of zero at the same pose.
VERIFY_TOLERANCE_RAD: Final[float] = 0.05
#: A joint further than this outside its range is a wrong zero or sign (LeadFollow's RANGE_SLACK).
RANGE_SLACK_RAD: Final[float] = 0.30
STOWED_POSE: Final[str] = (
    "its stowed pose: the upper arm down on its stop, the forearm folded back on top of it, "
    "the wrist straight, the gripper fully open"
)


def wrap_pi(a: float) -> float:
    """Into (-pi, pi], as the bridge's `wrap_pi`."""
    return -((-a + math.pi) % (2.0 * math.pi) - math.pi)


def new_offset(offset: float, sign: int, q: float) -> float:
    """The offset that makes the present reading zero: `wrap(offset + sign · q)`."""
    return wrap_pi(offset + sign * q)


def joint_rows(config_path: str | Path) -> dict[str, dict[str, Any]]:
    import yaml

    loaded: Any = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
    return {row["name"]: row for row in (loaded or {}).get("joints", []) if "name" in row}


def load_calib_arm(config_path: str | Path) -> ModuleType:
    """`tools/calib_arm.py` of the SDK the yaml belongs to (for `patch_yaml`)."""
    source = sdk_tool(config_path, "calib_arm.py")
    repo = source.parent.parent
    if str(repo) not in sys.path:
        sys.path.insert(0, str(repo))
    spec = importlib.util.spec_from_file_location("lucen_bench_calib_arm", source)
    if spec is None or spec.loader is None:
        raise RuntimeRefused(f"Cannot load {source}.")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read_median(address: str, names: Iterable[str], seconds: float = SAMPLE_S) -> dict[str, float]:
    """Each joint's median position over `seconds`, read in watch mode (no motor enabled)."""
    names = list(names)
    link = RuntimeLink(address, joints=names)
    link.open("watch")
    samples: dict[str, list[float]] = {n: [] for n in names}
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            q = link.positions()
            for n in names:
                if n in q:
                    samples[n].append(q[n])
            time.sleep(1 / 30)
    finally:
        link.close()
    silent = [n for n, s in samples.items() if not s]
    if silent:
        raise RuntimeRefused(
            f"No position from {', '.join(silent)}. Check those motors: 48 V, the CAN plug, "
            "L/H not swapped, the 120 ohm terminator at the end of the chain."
        )
    return {n: statistics.median(s) for n, s in samples.items()}


def zero_arm(
    address: str,
    prefix: str,
    joints: Iterable[str],
    config_path: str | Path,
    *,
    bridge: ManagedBridge | None = None,
    ask: Callable[[str], str] = input,
    say: Callable[[str], None] = print,
    patch: Callable[..., Any] | None = None,
    arm_name: str = "the arm",
) -> dict[str, tuple[float, float]]:
    """Zero `prefix` + `joints` at the stowed pose; returns {joint: (old offset, new offset)}."""
    config = Path(config_path).expanduser().resolve()
    rows = joint_rows(config)
    names = [prefix + j for j in joints]
    missing = [n for n in names if n not in rows]
    if missing:
        raise RuntimeRefused(f"{config.name} has no joint {', '.join(missing)} (prefix?).")
    if patch is None:
        patch = load_calib_arm(config).patch_yaml
    say(f"Zeroing {arm_name}. Its motors are off: it is limp and moves by hand.")
    ask(f"Fold {arm_name} to {STOWED_POSE}. Let it rest there, then press Enter. ")
    q = read_median(address, names)
    plan: dict[str, tuple[float, float]] = {}
    say(f"{'joint':<24}{'reads':>9}{'offset':>11}{'new offset':>12}")
    for n in names:
        sign, offset = int(rows[n]["sign"]), float(rows[n]["offset"])
        plan[n] = (offset, new_offset(offset, sign, q[n]))
        say(f"{n:<24}{q[n]:>+9.3f}{offset:>+11.5f}{plan[n][1]:>+12.5f}")
    answer = ask(f"Write these offsets to {config.name}? [y/N] ").strip().lower()
    if answer not in ("y", "yes"):
        say("Nothing written.")
        return {}
    # calib_arm.patch_yaml copies the file to .bak before each joint, so after seven joints the
    # .bak holds the sixth step. Keep the whole file as it was, once, beside it.
    backup = config.with_name(f"{config.name}.before-zero-{time.strftime('%Y%m%d-%H%M%S')}")
    shutil.copy2(config, backup)
    for n, (_, offset) in plan.items():
        patch(str(config), n, offset=offset)
    say(f"Written to {config} (as it was before: {backup.name}).")
    if bridge is not None and bridge.started:
        say("Restarting the bridge so it loads the new zero...")
        bridge.restart()
        after = read_median(address, names)
        off = {n: v for n, v in after.items() if abs(v) > VERIFY_TOLERANCE_RAD}
        if off:
            rows_text = ", ".join(f"{n} {v:+.3f}" for n, v in off.items())
            raise RuntimeRefused(f"After zeroing, these still read off zero: {rows_text}.")
        say(
            f"Checked: every joint of {arm_name} now reads within {VERIFY_TOLERANCE_RAD} rad "
            "of zero."
        )
    else:
        say("Restart the bridge (Ctrl-C in its terminal, run it again) so it loads the new zero.")
    if re.search(r"(?m)^calibrated:[ \t]*false\b", config.read_text(encoding="utf-8")):
        say(
            f"{config.name} still says calibrated: false, so the bridge will not enable the "
            "motors. "
            "Check every joint's direction (tools/calib_arm.py --signs), then --mark-calibrated."
        )
    return plan


def check_arm(
    address: str,
    prefix: str,
    joints: Iterable[str],
    limits: Mapping[str, tuple[float, float] | list[float]],
    *,
    say: Callable[[str], None] = print,
    arm_name: str = "the arm",
) -> list[str]:
    """Every motor answers and reads inside its range; returns the problems (empty: all good)."""
    joints = list(joints)
    names = [prefix + j for j in joints]
    problems: list[str] = []
    try:
        q = read_median(address, names, seconds=1.0)
    except RuntimeRefused as exc:
        return [str(exc)]
    say(f"{arm_name}: {len(names)} motors answer.")
    for joint, name in zip(joints, names, strict=True):
        lo, hi = (float(v) for v in limits[joint])
        mark = "ok"
        if not lo - RANGE_SLACK_RAD <= q[name] <= hi + RANGE_SLACK_RAD:
            mark = "OUTSIDE its range: zero it (lerobot-calibrate)"
            problems.append(f"{name} reads {q[name]:+.3f}, outside [{lo:+.3f}, {hi:+.3f}]")
        say(f"  {name:<24}{q[name]:>+8.3f}  [{lo:+.2f}, {hi:+.2f}]  {mark}")
    return problems


__all__ = ["check_arm", "new_offset", "read_median", "wrap_pi", "zero_arm"]
