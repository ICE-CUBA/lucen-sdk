"""Where the robot's own software is, given its yaml (no LeRobot needed).

The plugins run the Lucen SDK's code rather than copies of it: its bridge, its
LeadFollow (`tools/arm_lead_follow.py`) and its calibration writer
(`tools/calib_arm.py`). Two layouts carry them:

- the Lucen SDK (github.com/ICE-CUBA/lucen-sdk), what an owner clones: the yaml in a
  robot folder (`Arm/Arm_pair.yaml`), `bridge.py` and `tools/` at the root;
- an older checkout of the robot software: the yaml at the root, `legs_bridge.py` and
  `tools/`.

Model paths inside a yaml are relative to that root, so every tool runs from there.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Final

from lerobot_robot_lucen.runtime import RuntimeRefused

#: The bridge's file name: the SDK's first, then an older checkout's.
BRIDGE_SCRIPTS: Final[tuple[str, ...]] = ("bridge.py", "legs_bridge.py")


def find_root(config_path: str | os.PathLike[str]) -> Path:
    """The SDK root above a yaml: the nearest folder holding `tools/` and a bridge."""
    config = Path(config_path).expanduser().resolve()
    for folder in [config.parent, *config.parent.parents]:
        if (folder / "tools").is_dir() and any((folder / s).is_file() for s in BRIDGE_SCRIPTS):
            return folder
    raise RuntimeRefused(
        f"{config} is not inside the Lucen SDK (no folder above it holds tools/ and bridge.py). "
        "Name the yaml in your clone, e.g. ~/lucen-sdk/Arm/Arm_pair.yaml."
    )


def bridge_script(root: Path) -> str:
    for name in BRIDGE_SCRIPTS:
        if (root / name).is_file():
            return name
    raise RuntimeRefused(f"No bridge ({' or '.join(BRIDGE_SCRIPTS)}) in {root}.")


def sdk_tool(config_path: str | os.PathLike[str], name: str) -> Path:
    """`tools/<name>` of the SDK the yaml belongs to."""
    source = find_root(config_path) / "tools" / name
    if not source.is_file():
        raise RuntimeRefused(f"No {source}: this SDK is older than the plugins expect.")
    return source


__all__ = ["BRIDGE_SCRIPTS", "bridge_script", "find_root", "sdk_tool"]
