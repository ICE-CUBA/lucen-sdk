"""A client of the Lucen arm's runtime: the process on the robot that owns its motors.

The runtime is the robot's own bridge to its CAN bus (the Lucen SDK's `bridge.py
--config Arm/Arm.yaml`, port 9880; `Arm/Arm_pair.yaml`, a follower and a leader on one
board, is 9881). It speaks newline-delimited
JSON over TCP, one client at a time:

- client → runtime: ``{"hello": {"mode": "watch"|"send", "joints": [...],
  "limits": {joint: [lo, hi]}}}``, then ``{"target": {joint: rad}}`` as the
  heartbeat and the goal (an empty target only keeps the session alive);
- runtime → client: ``{"matched": [...], "unknown": [...]}``, in send mode
  ``{"q0": {...}}`` once the motors are enabled where they stand, then
  ``{"q": {...}}`` about 30 times a second.

The runtime keeps the safety layer: it clamps every target to the limits the
hello named, slews toward it at its own speed cap, and when the client goes
quiet for 1.5 s (3 s before the first target), or the connection drops, it damps
and disables the motors. Watch mode enables nothing: the arm is limp and can be
moved by hand, and positions still stream.

So this client never touches a motor: it names joints, sends targets and reads
positions. :class:`RuntimeBus` dresses it as the small part of LeRobot's motor
bus the Lucen bridge and LeRobot's own tools use, so the arm records through
both with no special case.
"""

from __future__ import annotations

import contextlib
import json
import math
import socket
import threading
import time
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any, Final

DEFAULT_ADDRESS: Final[str] = "127.0.0.1:9880"
#: The runtime gives up on a quiet client after 1.5 s; send well inside it.
HEARTBEAT_S: Final[float] = 0.2
#: Positions older than this mean the runtime stopped answering.
STALE_S: Final[float] = 1.0
#: The runtime takes up to 4 s to enable the motors on a send hello.
CONNECT_TIMEOUT_S: Final[float] = 6.0


class RuntimeRefused(RuntimeError):
    """The runtime refused, or stopped answering; the message says what to do."""


def parse_address(address: str) -> tuple[str, int]:
    """`host:port` (the default port 9880 when only a host is given)."""
    host, _, port = address.strip().rpartition(":")
    if not host:
        return address.strip() or "127.0.0.1", 9880
    try:
        return host, int(port)
    except ValueError as exc:
        raise RuntimeRefused(f"{address!r} is not HOST:PORT.") from exc


def with_port(address: str, port: int) -> str:
    """`address` with `port` added when it names only a host. Each robot's runtime has
    its own port (the arm 9880, the humanoid arm 9879), so a bare host means that robot's."""
    address = address.strip()
    return address if ":" in address else f"{address or '127.0.0.1'}:{port}"


@dataclass
class _Line:
    sock: socket.socket
    buffer: bytes = b""

    def send(self, message: Mapping[str, Any]) -> None:
        # NaN and infinity are never written: the runtime's clamp lets NaN through.
        self.sock.sendall((json.dumps(message, allow_nan=False) + "\n").encode())

    def read(self, timeout: float) -> dict[str, Any] | None:
        """The next message, or None when none arrived within `timeout`."""
        deadline = time.monotonic() + timeout
        while b"\n" not in self.buffer:
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            self.sock.settimeout(left)
            try:
                chunk = self.sock.recv(65536)
            except TimeoutError:
                return None
            if not chunk:
                raise RuntimeRefused("The arm's runtime closed the connection.")
            self.buffer += chunk
        line, self.buffer = self.buffer.split(b"\n", 1)
        message: dict[str, Any] = json.loads(line) if line.strip() else {}
        return message


def _is_finite(value: Any) -> bool:
    """A real number that is finite: not NaN, not infinity, not a string or a bool."""
    if isinstance(value, bool) or not isinstance(value, int | float):
        return False
    return math.isfinite(value)


def finite(values: Mapping[str, Any]) -> dict[str, float]:
    """The values that are finite numbers; NaN, infinity and non-numbers are dropped."""
    return {k: float(v) for k, v in values.items() if _is_finite(v)}


class RuntimeLink:
    """One session with the runtime: a hello, then targets out and positions in."""

    def __init__(
        self,
        address: str = DEFAULT_ADDRESS,
        *,
        joints: Iterable[str],
        limits: Mapping[str, tuple[float, float] | list[float]] | None = None,
        connect_timeout_s: float = CONNECT_TIMEOUT_S,
        heartbeat_s: float = HEARTBEAT_S,
    ) -> None:
        self.address = address
        self.joints = list(joints)
        self.limits = {k: [float(v[0]), float(v[1])] for k, v in (limits or {}).items()}
        self.connect_timeout_s = connect_timeout_s
        self.heartbeat_s = heartbeat_s
        self.mode: str | None = None
        self.q: dict[str, float] = {}
        self.q_at = 0.0
        self.q0: dict[str, float] = {}
        self._line: _Line | None = None
        self._target: dict[str, float] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self.error: str | None = None

    @property
    def connected(self) -> bool:
        return self._line is not None and self.error is None

    def open(self, mode: str) -> dict[str, float]:
        """Say hello in `mode` (`watch` or `send`); returns the positions the arm starts from."""
        if mode not in ("watch", "send"):
            raise ValueError("mode is watch or send")
        self.close()
        host, port = parse_address(self.address)
        try:
            sock = socket.create_connection((host, port), timeout=self.connect_timeout_s)
        except OSError as exc:
            raise RuntimeRefused(
                f"No arm runtime answers at {host}:{port}. Start it on the robot "
                "(the robot's own bridge, with the arm's configuration), then try again."
            ) from exc
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        line = _Line(sock)
        line.send({"hello": {"mode": mode, "joints": self.joints, "limits": self.limits}})
        reply = self._expect(line, "matched")
        unknown = list(reply.get("unknown") or [])
        if unknown:
            sock.close()
            raise RuntimeRefused(
                f"The arm's runtime does not know {', '.join(unknown)}. Its configuration and "
                "this robot's joint names must agree."
            )
        start: dict[str, float] = {}
        if mode == "send":
            start = finite(self._expect(line, "q0").get("q0") or {})
        else:
            # Watch streams positions at once: they are where the arm is. A runtime started a
            # moment ago reports some motors later than others (the bench, 2026-11-22:
            # follower_joint5 and joint6 a beat after the rest), so the start gathers until
            # every joint has reported, for as long as the hello may take.
            start = finite(self._expect(line, "q").get("q") or {})
            deadline = time.monotonic() + self.connect_timeout_s
            while time.monotonic() < deadline and any(j not in start for j in self.joints):
                message = line.read(deadline - time.monotonic())
                if message is None:
                    break
                if "q" in message:
                    start.update(finite(message["q"]))
            missing = [j for j in self.joints if j not in start]
            if missing:
                sock.close()
                raise RuntimeRefused(
                    f"The runtime enabled no reading for {', '.join(missing)} in time. "
                    "Check those motors, then connect again."
                )
        self._line = line
        self.mode = mode
        self.error = None
        self.q0 = dict(start) if mode == "send" else {}
        with self._lock:
            self.q = dict(start)
            self.q_at = time.monotonic() if start else 0.0
            self._target = dict(start)
        self._stop.clear()
        self._threads = [
            threading.Thread(target=self._read, name="lucen-runtime-read", daemon=True)
        ]
        if mode == "send":
            self._threads.append(
                threading.Thread(target=self._beat, name="lucen-runtime-beat", daemon=True)
            )
        for thread in self._threads:
            thread.start()
        return start

    def _expect(self, line: _Line, key: str) -> dict[str, Any]:
        deadline = time.monotonic() + self.connect_timeout_s
        while time.monotonic() < deadline:
            message = line.read(deadline - time.monotonic())
            if message is None:
                break
            if key in message:
                return message
            if "q" in message:
                with self._lock:
                    self.q.update(finite(message["q"]))
                    self.q_at = time.monotonic()
        raise RuntimeRefused(f"The arm's runtime did not answer the hello ({key}) in time.")

    def _read(self) -> None:
        line = self._line
        assert line is not None
        while not self._stop.is_set():
            try:
                message = line.read(0.2)
            except (OSError, RuntimeRefused, ValueError) as exc:
                if not self._stop.is_set():
                    self.error = f"lost the runtime: {exc}"
                return
            if message and "q" in message:
                with self._lock:
                    self.q.update(finite(message["q"]))
                    self.q_at = time.monotonic()

    def _beat(self) -> None:
        while not self._stop.wait(self.heartbeat_s):
            with self._lock:
                target = dict(self._target)
            try:
                if self._line is not None:
                    self._line.send({"target": target})
            except OSError as exc:
                self.error = f"lost the runtime: {exc}"
                return

    def positions(self) -> dict[str, float]:
        """The latest joint positions (radians); refuses ones the runtime stopped sending."""
        if self.error:
            raise RuntimeRefused(self.error)
        with self._lock:
            age = time.monotonic() - self.q_at if self.q_at else float("inf")
            q = dict(self.q)
        if age > STALE_S and self.mode is not None:
            raise RuntimeRefused(f"The arm's runtime sent no position for {age:.1f} s.")
        return q

    def target(self, goal: Mapping[str, float]) -> None:
        """Ask the runtime for `goal` (radians). It clamps and slews; this never moves a motor."""
        if self.mode != "send":
            raise RuntimeRefused("Enable the arm first: targets are taken in send mode only.")
        unknown = [k for k in goal if k not in self.joints]
        if unknown:
            raise RuntimeRefused(f"No joint named {', '.join(unknown)} on this arm.")
        bad = [k for k, v in goal.items() if not _is_finite(v)]
        if bad:
            # Defence in depth: the robot's runtime clamps with min/max, which NaN passes.
            raise RuntimeRefused(f"Refusing a target that is not a number for {', '.join(bad)}.")
        with self._lock:
            self._target.update({k: float(v) for k, v in goal.items()})
            target = dict(self._target)
        if self._line is not None:
            self._line.send({"target": target})

    def close(self) -> None:
        """End the session: the runtime damps and disables the motors."""
        self._stop.set()
        line, self._line = self._line, None
        if line is not None:
            with contextlib.suppress(OSError):
                line.sock.close()
        for thread in self._threads:
            thread.join(timeout=1)
        self._threads = []
        self.mode = None


class RuntimeBus:
    """The runtime, as the part of LeRobot's motor bus the bridge and LeRobot's tools read.

    `connect` opens a watch session (nothing enabled), `enable_torque` a send
    session (the motors hold where they stand), `disable_torque` returns to
    watch (the runtime damps and disables), `disconnect` ends it. Positions are
    radians in the robot's own joint coordinates; there is no load register.
    """

    def __init__(
        self,
        address: str,
        joints: Iterable[str],
        limits: Mapping[str, tuple[float, float] | list[float]] | None = None,
        *,
        connect_timeout_s: float = CONNECT_TIMEOUT_S,
    ) -> None:
        self.link = RuntimeLink(
            address, joints=joints, limits=limits, connect_timeout_s=connect_timeout_s
        )
        self.motors = {joint: SimpleNamespace(norm_mode="radians") for joint in self.link.joints}
        self.port_handler = None
        self._known = False

    @property
    def is_connected(self) -> bool:
        return self.link.connected

    @property
    def is_calibrated(self) -> bool:
        """The runtime owns calibration; it knew every joint this arm names."""
        return self._known

    def connect(self, handshake: bool = True) -> None:
        self.link.open("watch")
        self._known = True

    def enable_torque(self, motors: Any = None, num_retry: int = 0) -> None:
        self.link.open("send")

    def disable_torque(self, motors: Any = None, num_retry: int = 0) -> None:
        if self.link.mode == "send":
            self.link.open("watch")

    def sync_read(
        self, data_name: str, motors: Any = None, normalize: bool = True
    ) -> dict[str, float]:
        if data_name != "Present_Position":
            raise NotImplementedError(f"The arm's runtime reports positions only, not {data_name}.")
        q = self.link.positions()
        names = [motors] if isinstance(motors, str) else list(motors or self.link.joints)
        return {name: q[name] for name in names if name in q}

    def sync_write(
        self, data_name: str, values: Mapping[str, float], normalize: bool = True
    ) -> None:
        if data_name != "Goal_Position":
            raise NotImplementedError(
                f"The arm's runtime takes goal positions only, not {data_name}."
            )
        if self.link.mode == "send":
            self.link.target(values)

    def disconnect(self, disable_torque: bool = True) -> None:
        self.link.close()


__all__ = [
    "DEFAULT_ADDRESS",
    "RuntimeBus",
    "RuntimeLink",
    "RuntimeRefused",
    "finite",
    "parse_address",
    "with_port",
]
