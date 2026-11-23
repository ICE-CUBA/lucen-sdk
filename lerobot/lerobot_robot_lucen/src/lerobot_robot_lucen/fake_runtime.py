"""A stand-in for the Lucen arm's runtime, speaking its protocol, for tests and rehearsal.

It keeps the rules the real runtime keeps (the Lucen SDK's `bridge.py`): one client
at a time; a hello names the
joints, and unknown ones are listed back; send mode enables the motors where
they stand and answers ``q0``; every target is clamped to the hello's limits
and approached at a speed cap; a client quiet for 1.5 s (3 s before its first
target), or one that disconnects, leaves the arm damped and disabled; watch
mode moves nothing. The arm itself is a kinematic follower here, no motors.

    python -m lerobot_robot_lucen.fake_runtime --port 9880
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import threading
import time
from collections.abc import Iterable, Mapping
from typing import Any, Final

#: The real runtime's own numbers (bridge.py): the client timeout and the boot grace.
CLIENT_TIMEOUT_S: Final[float] = 1.5
BOOT_GRACE_S: Final[float] = 3.0
#: Its mirror speed cap (slew.py SOFT_SPD), radians a second.
SPEED_RAD_S: Final[float] = 1.0
TICK_S: Final[float] = 0.02
PUSH_S: Final[float] = 0.033


class FakeArmRuntime:
    """A runtime on a free port, in threads of its own; `events` says what it did."""

    def __init__(
        self,
        joints: Iterable[str],
        *,
        start: Mapping[str, float] | None = None,
        port: int = 0,
        speed: float = SPEED_RAD_S,
        client_timeout_s: float = CLIENT_TIMEOUT_S,
        boot_grace_s: float = BOOT_GRACE_S,
        first_report_s: Mapping[str, float] | None = None,
    ) -> None:
        """`first_report_s`: joints whose first position arrives that many seconds after
        `start()`, as a runtime started a moment ago reports some motors later than others."""
        self.known = list(joints)
        self.first_report_s = dict(first_report_s or {})
        self._started_at = math.inf
        self.q = {j: float((start or {}).get(j, 0.0)) for j in self.known}
        self.speed = speed
        self.client_timeout_s = client_timeout_s
        self.boot_grace_s = boot_grace_s
        self.mode = "idle"
        self.active: set[str] = set()
        self.limits: dict[str, tuple[float, float]] = {}
        self.target: dict[str, float] = {}
        self.events: list[dict[str, Any]] = []
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("127.0.0.1", port))
        self._server.listen(1)
        self._server.settimeout(0.2)
        self.port = int(self._server.getsockname()[1])
        self._threads = [
            threading.Thread(target=self._serve, name="fake-runtime-serve", daemon=True),
            threading.Thread(target=self._tick, name="fake-runtime-tick", daemon=True),
        ]

    @property
    def address(self) -> str:
        return f"127.0.0.1:{self.port}"

    def start(self) -> FakeArmRuntime:
        self._started_at = time.monotonic()
        for thread in self._threads:
            thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=2)
        self._server.close()

    def move_by_hand(self, positions: Mapping[str, float]) -> None:
        """Watch mode: a person moves the limp arm."""
        with self.lock:
            self.q.update({k: float(v) for k, v in positions.items() if k in self.q})

    def _event(self, kind: str, **detail: Any) -> None:
        self.events.append({"kind": kind, "t": time.monotonic(), **detail})

    def _safe_idle(self, why: str) -> None:
        with self.lock:
            if self.mode != "idle":
                self._event("safe_idle", why=why, was=self.mode)
            self.mode = "idle"
            self.target.clear()

    def _tick(self) -> None:
        while not self._stop.wait(TICK_S):
            with self.lock:
                if self.mode != "send":
                    continue
                for joint in self.active:
                    goal = self.target.get(joint, self.q[joint])
                    lo, hi = self.limits.get(joint, (-math.inf, math.inf))
                    goal = min(max(goal, lo), hi)
                    step = self.speed * TICK_S
                    self.q[joint] += max(-step, min(step, goal - self.q[joint]))

    def _hello(self, hello: Mapping[str, Any], send: Any) -> None:
        asked = list(hello.get("joints") or self.known)
        matched = [j for j in asked if j in self.q]
        unknown = [j for j in asked if j not in self.q]
        with self.lock:
            self.active = set(matched)
            self.limits = {
                k: (float(v[0]), float(v[1])) for k, v in (hello.get("limits") or {}).items()
            }
            self.target.clear()
            self.mode = str(hello.get("mode") or "watch")
            mode = self.mode
        self._event("hello", mode=mode, matched=matched, unknown=unknown)
        send({"matched": matched, "unknown": unknown})
        if mode == "send":
            with self.lock:
                q0 = {j: self.q[j] for j in self.active}
            send({"q0": q0})

    def _serve(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _addr = self._server.accept()
            except (TimeoutError, OSError):
                continue
            self._event("connect")
            self._session(conn)
            conn.close()
            self._safe_idle("client gone")

    def _session(self, conn: socket.socket) -> None:
        conn.settimeout(PUSH_S)
        buffer = b""
        last_rx = time.monotonic()
        got_target = False

        def send(message: dict[str, Any]) -> None:
            conn.sendall((json.dumps(message) + "\n").encode())

        while not self._stop.is_set():
            try:
                chunk = conn.recv(65536)
                if not chunk:
                    return
                buffer += chunk
                last_rx = time.monotonic()
            except TimeoutError:
                pass
            except OSError:
                return
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                if not line.strip():
                    continue
                message = json.loads(line)
                if "hello" in message:
                    got_target = False
                    self._hello(message["hello"], send)
                elif "target" in message:
                    with self.lock:
                        for joint, value in message["target"].items():
                            if joint in self.active:
                                self.target[joint] = float(value)
                    got_target = got_target or bool(message["target"])
            allow = self.client_timeout_s if got_target else self.boot_grace_s
            if self.mode == "send" and time.monotonic() - last_rx > allow:
                self._event("timeout", after_s=round(time.monotonic() - last_rx, 2))
                return
            up_s = time.monotonic() - self._started_at
            with self.lock:
                snapshot = (
                    {j: self.q[j] for j in self.active if up_s >= self.first_report_s.get(j, 0.0)}
                    if self.mode != "idle"
                    else {}
                )
            if snapshot:
                try:
                    send({"q": snapshot})
                except OSError:
                    return


def main(argv: list[str] | None = None) -> int:  # pragma: no cover - a person runs it
    from lerobot_robot_lucen.features import LUCEN_ARM_JOINTS

    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0] if __doc__ else None)
    parser.add_argument("--port", type=int, default=9880)
    args = parser.parse_args(argv)
    runtime = FakeArmRuntime(LUCEN_ARM_JOINTS, port=args.port).start()
    print(f"fake arm runtime on {runtime.address}; Ctrl-C stops it", flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        runtime.stop()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = ["FakeArmRuntime"]
