"""The arm's bridge as a child process: plugging in the MC02 is the whole setup (no LeRobot).

With `--robot.arm_config` (or `--teleop.arm_config`) naming the arm's yaml in the Lucen
SDK (`~/lucen-sdk/Arm/Arm_pair.yaml`), the first arm of the process that connects starts
the SDK's bridge itself (`bridge.py --config … --profile mirror`) unless one already listens
on the yaml's port, and the session's end stops it again. It stays a process of its
own on purpose: when LeRobot crashes, the bridge still sees its client go and damps
and disables the motors, and when the bridge dies, the MC02's watchdog (100 ms) and
each motor's CAN timeout (500 ms) do. A bridge somebody started by hand is used as it
is and never stopped from here.
"""

from __future__ import annotations

import atexit
import contextlib
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, ClassVar, Final

from lerobot_robot_lucen.runtime import RuntimeRefused
from lerobot_robot_lucen.sdk import bridge_script, find_root

#: How long the bridge may take to find the MC02 and enable nothing yet (it listens last).
START_TIMEOUT_S: Final[float] = 20.0
#: How long a SIGINT has to damp, disable and exit before the process is killed.
STOP_TIMEOUT_S: Final[float] = 6.0
LOG_DIR: Final[Path] = Path.home() / ".cache" / "lucen"


def listening(port: int) -> bool:
    """Something on this computer listens on `port`, found without connecting to it (the bridge
    serves one client at a time, and a probe would be one)."""
    try:
        import psutil  # type: ignore[import-untyped]  # a LeRobot dependency

        return any(
            c.status == psutil.CONN_LISTEN and c.laddr and c.laddr.port == port
            for c in psutil.net_connections(kind="tcp")
        )
    except Exception:
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            probe.bind(("0.0.0.0", port))
        except OSError:
            return True
        finally:
            probe.close()
        return False


def read_bridge_port(config_path: str | os.PathLike[str]) -> int:
    import yaml

    loaded: Any = yaml.safe_load(Path(config_path).expanduser().read_text(encoding="utf-8"))
    return int(((loaded or {}).get("bridge") or {}).get("port", 9877))


class ManagedBridge:
    """The bridge for one yaml: started when nothing listens, stopped only if started here."""

    _registry: ClassVar[dict[int, ManagedBridge]] = {}

    @classmethod
    def for_config(
        cls,
        config_path: str | os.PathLike[str],
        *,
        python: str = "",
        profile: str = "mirror",
        grav_scale: float | None = None,
        script: str = "",
    ) -> ManagedBridge:
        config = Path(config_path).expanduser().resolve()
        if not config.is_file():
            raise RuntimeRefused(
                f"No bridge configuration at {config}. Name the arm's yaml in the robot's "
                "SDK, e.g. --robot.arm_config=~/lucen-sdk/Arm/Arm_pair.yaml."
            )
        port = read_bridge_port(config)
        bridge = cls._registry.get(port)
        if bridge is None:
            bridge = cls._registry[port] = cls(config, port, python, profile, grav_scale, script)
        elif bridge.config != config:
            raise RuntimeRefused(
                f"Two configurations want the bridge on port {port}: {bridge.config} and {config}."
            )
        return bridge

    def __init__(
        self,
        config: Path,
        port: int,
        python: str,
        profile: str,
        grav_scale: float | None,
        script: str,
    ) -> None:
        self.config = config
        #: The SDK root the bridge runs from (model paths in the yaml are relative to it).
        self.root = find_root(config)
        script = script or bridge_script(self.root)
        self.port = port
        self.python = python or sys.executable
        self.profile = profile
        self.grav_scale = grav_scale
        self.script = script
        self.proc: subprocess.Popen[bytes] | None = None
        self.log_path = LOG_DIR / f"bridge-{port}.log"

    @property
    def address(self) -> str:
        return f"127.0.0.1:{self.port}"

    @property
    def started(self) -> bool:
        """This process started the bridge, and it is still running."""
        return self.proc is not None and self.proc.poll() is None

    def command(self) -> list[str]:
        cmd = [
            self.python,
            "-u",
            self.script,
            "--config",
            str(self.config),
            "--port",
            str(self.port),
        ]
        cmd += ["--profile", self.profile]
        if self.grav_scale is not None:
            cmd += ["--grav-scale", f"{self.grav_scale:g}"]
        return cmd

    def ensure(self) -> bool:
        """Start the bridge unless something listens on its port; True when it was started now."""
        if self.started or listening(self.port):
            return False
        script = self.root / self.script
        if not script.is_file():
            raise RuntimeRefused(f"No {script}: the bridge comes from the Lucen SDK.")
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        log = self.log_path.open("ab")
        log.write(
            f"\n=== {time.strftime('%Y-%m-%d %H:%M:%S')} {' '.join(self.command())}\n".encode()
        )
        log.flush()
        env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "PYTHONHOME")}
        self.proc = subprocess.Popen(
            self.command(),
            cwd=self.root,
            stdout=log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            env=env,
            start_new_session=True,  # Ctrl-C in LeRobot's terminal stops LeRobot, which stops this
        )
        log.close()
        deadline = time.monotonic() + START_TIMEOUT_S
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeRefused(
                    f"The bridge exited ({self.proc.returncode}) before it listened on "
                    f"{self.port}. "
                    f"Its log, {self.log_path}, ends with:\n{self.tail()}"
                )
            if listening(self.port):
                return True
            time.sleep(0.2)
        self.stop()
        raise RuntimeRefused(
            f"The bridge did not listen on {self.port} within {START_TIMEOUT_S:.0f} s (is the MC02 "
            f"plugged in and the arm powered?). Its log, {self.log_path}, ends with:\n{self.tail()}"
        )

    def stop(self) -> None:
        """Stop the bridge if this process started it: SIGINT (it damps and disables), then kill."""
        proc, self.proc = self.proc, None
        if proc is None or proc.poll() is not None:
            return
        with contextlib.suppress(ProcessLookupError):
            os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=STOP_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=2)

    def restart(self) -> None:
        """Load a changed yaml: only a bridge started here can be restarted from here."""
        if not self.started:
            raise RuntimeRefused(
                "The bridge was started by hand: restart it to load the new configuration "
                "(Ctrl-C in its terminal, then run it again)."
            )
        self.stop()
        deadline = time.monotonic() + 5.0
        while listening(self.port) and time.monotonic() < deadline:
            time.sleep(0.1)
        self.ensure()

    def tail(self, lines: int = 12) -> str:
        try:
            return "\n".join(self.log_path.read_text(errors="replace").splitlines()[-lines:])
        except OSError:
            return "(no log)"


def _stop_all_at_exit() -> None:
    for bridge in list(ManagedBridge._registry.values()):
        with contextlib.suppress(Exception):
            bridge.stop()


atexit.register(_stop_all_at_exit)


__all__ = ["ManagedBridge", "listening", "read_bridge_port"]
