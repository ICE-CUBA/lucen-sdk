"""One session with a runtime that serves several arms of one process (no LeRobot needed here).

A Lucen Arm pair is two identical 6+1 arms, a follower and a leader, on one MC02
board, behind one bridge (the Lucen SDK's `bridge.py --config Arm/Arm_pair.yaml`, port
9881). Its joints carry the arm's prefix (`follower_joint1` … `leader_gripper_joint`),
and the bridge serves one client at a time. LeRobot builds the robot and the
teleoperator as two objects of one process, so they cannot each open a session:
the second would wait behind the first. Here they share one.

The rules are those of the arm driver that has driven both arms of a pair for a hosted
policy (`_Link` in `tools/dream_driver.py` of the robot software):

- everything that will use the session claims its joints first; the first `open()`
  says hello once, in send mode, for all of them, so every claimed motor is enabled
  where it stands and nothing else is;
- a sender thread sends the whole target 50 times a second, the bridge's own control
  rate, whether or not anyone wrote a new one, so an arm is never left without a
  target (the bridge damps and disables after 1.5 s of silence);
- each target is held inside the limits, and a target nothing else bounds (a policy's)
  within a leash of the measured position (0.35 rad, 0.25 on a gripper): an arm that is
  blocked or holding something pushes with a bounded torque instead of winding up. The
  leader's LeadFollow bounds its own (a speed cap, a collision pre-check, a gripper lead)
  and adds the gravity sag it learns (up to 0.6 rad), which a leash would cut, and its
  feed-forward hint would come out wrong. So the leader's joints are claimed without the
  leash, and the follower targets LeadFollow computes are vouched for (`vouch`) and go out
  as they are. Every other target on the follower keeps the leash: a policy driving it
  while a leader is connected is leashed like any other;
- an arm is parked before it is let go (`park`): this arm has no brake, so a released
  arm falls. The route is the driver's: shoulder, elbow and wrist to the upright rest
  pose with the base and the wrist roll unchanged, then the base and the wrist roll to
  zero, then fold to the stowed pose. It was swept in the model for self-collision and
  the table; between two arms it is not checked;
- the session ends when the last user releases it, and the bridge then damps and
  disables every motor.

If the process dies, the bridge damps and disables after 1.5 s and both arms fall from
where they are. The 48 V stop stays within reach.
"""

from __future__ import annotations

import atexit
import collections
import contextlib
import json
import math
import socket
import threading
import time
from collections.abc import Iterable, Mapping
from typing import Any, ClassVar, Final

from lerobot_robot_lucen.bridge import ManagedBridge
from lerobot_robot_lucen.runtime import RuntimeRefused, finite, parse_address, with_port

#: The arm driver's numbers (see the module docstring).
SEND_HZ: Final[float] = 50.0
LEASH_RAD: Final[float] = 0.35
GRIPPER_LEASH_RAD: Final[float] = 0.25
PARK_SPEED_RAD_S: Final[float] = 0.45
PARK_TOLERANCE_RAD: Final[float] = 0.15
STALE_S: Final[float] = 1.0
OPEN_TIMEOUT_S: Final[float] = 8.0
#: How many of a joint's latest vouched values still count when a target arrives (`vouch`):
#: five of LeadFollow's 50 Hz ticks, 0.1 s from the snapshot LeRobot takes to its send.
VOUCH_DEPTH: Final[int] = 5
#: The upright rest pose the park route passes through (a yaml's `routine.rest` overrides
#: it): the arm vertical, the least gravity torque, off every stop.
REST_POSE: Final[dict[str, float]] = {"joint2": 1.44, "joint3": 2.40, "joint4": 0.54}
#: The six arm joints of one 6+1 arm, base to wrist, and its gripper.
ARM_JOINTS: Final[tuple[str, ...]] = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER: Final[str] = "gripper_joint"


def _minjerk(u: float) -> float:
    return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)


def _key(address: str) -> str:
    host, port = parse_address(address)
    return f"{host}:{port}"


def _park_all_at_exit() -> None:
    """The process is ending with an arm still held (its owner's `disconnect` was skipped:
    LeRobot disconnects only what reports itself connected, and a robot with a dead camera
    does not). Park what is still held, then close: an arm let go unparked falls."""
    for shared in list(SharedRuntime._registry.values()):
        with contextlib.suppress(Exception):
            shared.park_held()
        with contextlib.suppress(Exception):
            shared.close("the process ended")


class SharedRuntime:
    """The session with one runtime, shared by every arm of this process that names it."""

    _registry: ClassVar[dict[str, SharedRuntime]] = {}
    _guard: ClassVar[threading.Lock] = threading.Lock()

    @classmethod
    def for_address(cls, address: str) -> SharedRuntime:
        key = _key(address)
        with cls._guard:
            shared = cls._registry.get(key)
            if shared is None or shared.closed:
                shared = cls._registry[key] = cls(key)
            return shared

    def __init__(self, address: str, *, open_timeout_s: float = OPEN_TIMEOUT_S) -> None:
        self.address = address
        self.open_timeout_s = open_timeout_s
        self.joints: list[str] = []
        self.limits: dict[str, tuple[float, float]] = {}
        self.grippers: set[str] = set()
        #: Joints some user claimed with the leash: they keep it, whoever else claims them.
        self.leashed: set[str] = set()
        #: Joints only users that bound them themselves claimed (see `claim`): sent unleashed.
        self.unleashed: set[str] = set()
        #: joint -> the latest values a self-bounding user handed out for it (see `vouch`).
        self._vouched: dict[str, collections.deque[float]] = {}
        #: Joints whose present target was one of those values when it was set.
        self._as_vouched: set[str] = set()
        self.users: set[str] = set()
        #: user -> how to park its arm (prefix, home, rest, speed), once it is connected.
        self.parking: dict[str, tuple[str, dict[str, float], dict[str, float] | None, float]] = {}
        self.opened = False
        self.closed = False
        self.why = ""
        self._lock = threading.Lock()
        self._q: dict[str, float] = {}
        self._target: dict[str, float] = {}
        self._stamp = 0.0
        self._sock: socket.socket | None = None
        #: The bridge this session starts when nothing listens, and stops when it ends.
        self.bridge: ManagedBridge | None = None
        self._buffer = b""
        self._threads: list[threading.Thread] = []

    # -- before the hello --------------------------------------------------------

    def claim(
        self,
        user: str,
        joints: Iterable[str],
        limits: Mapping[str, tuple[float, float] | list[float]],
        grippers: Iterable[str] = (),
        *,
        leash: bool = True,
    ) -> None:
        """`user` will use `joints` (runtime names). Every claim comes before the first open.

        `leash=False`: the user keeps its own bounds on these joints (LeadFollow: a speed cap,
        a collision pre-check, a gripper lead), so the session sends its targets as they are,
        as the SDK's own lead-follow tool does. The leash is for targets nothing else
        bounds: a policy's. A leashed claim wins: a joint any user claims with the leash
        keeps it, in whatever order the claims come (a self-bounding user that hands out
        targets for such a joint vouches for them instead, see `vouch`).
        """
        joints = list(joints)
        with self._lock:
            if self.opened:
                missing = [j for j in joints if j not in self.joints]
                if missing:
                    raise RuntimeRefused(
                        f"The session with {self.address} is already open without "
                        f"{', '.join(missing)}. Build every arm before connecting any."
                    )
            for joint in joints:
                if joint not in self.joints:
                    self.joints.append(joint)
                lo, hi = limits[joint]
                self.limits[joint] = (float(lo), float(hi))
            self.grippers.update(grippers)
            if leash:
                self.leashed.update(joints)
                self.unleashed.difference_update(joints)
            else:
                self.unleashed.update(j for j in joints if j not in self.leashed)
            self.users.add(user)

    def vouch(self, values: Mapping[str, float]) -> None:
        """A self-bounding user hands out these exact targets for leashed joints it bounds but
        does not own (LeadFollow's follower targets, which LeRobot sends through the follower
        robot): such a target goes out as it is when it arrives while still among the joint's
        last `VOUCH_DEPTH` vouched values. Any other target on the joint keeps the leash."""
        self._check(values)
        with self._lock:
            for joint, value in values.items():
                recent = self._vouched.get(joint)
                if recent is None:
                    recent = self._vouched[joint] = collections.deque(maxlen=VOUCH_DEPTH)
                recent.append(float(value))

    def forget(self, joints: Iterable[str]) -> None:
        """Nothing vouched for `joints` counts any more, the present target included (the user
        that vouched stopped or let go)."""
        with self._lock:
            for joint in joints:
                self._vouched.pop(joint, None)
                self._as_vouched.discard(joint)

    # -- the session ---------------------------------------------------------------

    def use_bridge(self, bridge: ManagedBridge) -> None:
        """Start `bridge` at the first open when nothing listens yet (one bridge per session)."""
        if self.bridge is not None and self.bridge is not bridge:
            raise RuntimeRefused(f"Two bridges were named for {self.address}.")
        if bridge.address != self.address:
            raise RuntimeRefused(
                f"The bridge's yaml listens on {bridge.address}, the arm names {self.address}."
            )
        self.bridge = bridge

    def open(self) -> dict[str, float]:
        """Say hello once for every claimed joint; returns where they stand. Idempotent."""
        with self._lock:
            if self.opened and not self.closed:
                return dict(self._q)
            if self.closed:
                raise RuntimeRefused(f"The session with {self.address} ended: {self.why}")
            joints, limits = list(self.joints), dict(self.limits)
        host, port = parse_address(self.address)
        if self.bridge is not None:
            self.bridge.ensure()
        try:
            sock = socket.create_connection((host, port), timeout=6.0)
        except OSError as exc:
            raise RuntimeRefused(
                f"No arm runtime answers at {host}:{port}. Start the bridge on the robot "
                "(python bridge.py --config Arm/Arm_pair.yaml in the Lucen SDK), or name its yaml "
                "with arm_config so the plugin starts it, then "
                "try again."
            ) from exc
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(None)
        self._sock = sock
        hello = {
            "mode": "send",
            "joints": joints,
            "limits": {j: list(v) for j, v in limits.items()},
        }
        self._send({"hello": hello})
        matched: list[str] | None = None
        q0: dict[str, float] | None = None
        deadline = time.monotonic() + self.open_timeout_s
        try:
            while time.monotonic() < deadline and (matched is None or q0 is None):
                for message in self._read(0.2):
                    if message.get("error"):
                        raise RuntimeRefused(f"The bridge refused: {message['error']}")
                    if "matched" in message:
                        matched = list(message["matched"])
                        unknown = list(message.get("unknown") or [])
                        if unknown:
                            raise RuntimeRefused(
                                f"The bridge does not know {', '.join(unknown)}. Its "
                                "configuration and these joint names must agree (prefix?)."
                            )
                    elif "q0" in message:
                        q0 = finite(message["q0"] or {})
        except (RuntimeRefused, OSError, ValueError) as exc:
            self.close(f"refused at the hello: {exc}")
            raise exc if isinstance(exc, RuntimeRefused) else RuntimeRefused(str(exc)) from exc
        missing = [j for j in joints if j not in (q0 or {})]
        if matched is None or q0 is None or missing:
            self.close("the motors were not all enabled")
            raise RuntimeRefused(
                f"The bridge at {self.address} did not enable "
                f"{', '.join(missing) if missing else 'the motors'} within "
                f"{self.open_timeout_s:.0f} s. Look at the bridge's terminal: a motor that does "
                "not answer, a chain without 48 V, or a CAN plug."
            )
        with self._lock:
            self._q = dict(q0)
            self._target = dict(q0)
            self._stamp = time.monotonic()
            self.opened = True
        self._threads = [
            threading.Thread(target=self._reader, name="lucen-shared-read", daemon=True),
            threading.Thread(target=self._sender, name="lucen-shared-send", daemon=True),
        ]
        for thread in self._threads:
            thread.start()
        return dict(q0)

    def _send(self, message: Mapping[str, Any]) -> None:
        sock = self._sock
        if sock is None:
            raise OSError("the session is closed")
        # NaN and infinity are never written: the bridge drops them, but say so here first.
        sock.sendall((json.dumps(message, allow_nan=False) + "\n").encode())

    def _read(self, timeout: float) -> list[dict[str, Any]]:
        import select

        sock = self._sock
        if sock is None:
            raise OSError("the session is closed")
        ready, _, _ = select.select([sock], [], [], timeout)
        if not ready:
            return []
        chunk = sock.recv(65536)
        if not chunk:
            raise OSError("the bridge closed the connection")
        self._buffer += chunk
        out = []
        while b"\n" in self._buffer:
            line, self._buffer = self._buffer.split(b"\n", 1)
            if line.strip():
                out.append(json.loads(line))
        return out

    def _reader(self) -> None:
        try:
            while not self.closed:
                for message in self._read(0.5):
                    if "q" in message:
                        with self._lock:
                            self._q.update(finite(message["q"]))
                            self._stamp = time.monotonic()
        except (OSError, ValueError) as exc:
            if not self.closed:  # closing the socket ends this thread too; say nothing then
                self.close(f"lost the bridge ({exc})")

    def outgoing(self) -> dict[str, float]:
        """The target this tick sends: each joint's within the limits, and within the leash of
        its measured position unless only users that bound it themselves claimed it, or the
        target was vouched for when it arrived (LeadFollow's own, see `vouch`)."""
        with self._lock:
            out: dict[str, float] = {}
            for joint in self.joints:
                q = self._q[joint]
                goal = self._target.get(joint, q)
                if joint not in self.unleashed and joint not in self._as_vouched:
                    lead = GRIPPER_LEASH_RAD if joint in self.grippers else LEASH_RAD
                    goal = min(max(goal, q - lead), q + lead)
                lo, hi = self.limits[joint]
                out[joint] = min(max(goal, lo), hi)
            return out

    def _sender(self) -> None:
        due = time.monotonic()
        while not self.closed:
            try:
                self._send({"target": self.outgoing()})
            except (OSError, ValueError) as exc:
                if not self.closed:
                    self.close(f"lost the bridge ({exc})")
                return
            due += 1.0 / SEND_HZ
            time.sleep(max(0.0, due - time.monotonic()))
            if due < time.monotonic() - 0.5:  # the machine stalled: re-align, do not catch up
                due = time.monotonic()

    def alive(self) -> bool:
        return self.opened and not self.closed and time.monotonic() - self._stamp < STALE_S

    def positions(self, joints: Iterable[str] | None = None) -> dict[str, float]:
        """The latest measured positions (radians); refuses ones the bridge stopped sending."""
        if self.closed:
            raise RuntimeRefused(f"The session with {self.address} ended: {self.why}")
        if not self.opened:
            raise RuntimeRefused("Connect first: the session with the bridge is not open.")
        with self._lock:
            age = time.monotonic() - self._stamp
            q = dict(self._q)
        if age > STALE_S:
            raise RuntimeRefused(f"The bridge at {self.address} sent no position for {age:.1f} s.")
        names = list(joints) if joints is not None else list(q)
        return {j: q[j] for j in names if j in q}

    def set_target(self, goal: Mapping[str, float]) -> None:
        """Ask for `goal` (radians, runtime names). The sender leashes, clamps and sends it.

        A value equal to one of the joint's latest vouched values (`vouch`) is LeadFollow's
        own and is sent unleashed for as long as it stays the target; anything else is not."""
        self._check(goal)
        with self._lock:
            for joint, value in goal.items():
                target = self._target[joint] = float(value)
                if target in self._vouched.get(joint, ()):
                    self._as_vouched.add(joint)
                else:
                    self._as_vouched.discard(joint)

    def _check(self, goal: Mapping[str, float]) -> None:
        unknown = [j for j in goal if j not in self.limits]
        if unknown:
            raise RuntimeRefused(f"No joint named {', '.join(unknown)} on this session.")
        bad = [j for j, v in goal.items() if not (isinstance(v, int | float) and math.isfinite(v))]
        if bad:
            raise RuntimeRefused(f"Refusing a target that is not a number for {', '.join(bad)}.")

    def targets(self, joints: Iterable[str]) -> dict[str, float]:
        with self._lock:
            return {j: self._target[j] for j in joints if j in self._target}

    # -- parking -------------------------------------------------------------------

    def run_to(self, goal: Mapping[str, float], speed: float = PARK_SPEED_RAD_S) -> None:
        """Move `goal`'s joints from their present targets together (minimum jerk); blocks."""
        start = self.targets(goal)
        span = max((abs(goal[j] - start[j]) for j in goal if j in start), default=0.0)
        if span < 0.02:
            return
        duration = max(1.0, 1.875 * span / speed)
        t0 = time.monotonic()
        while self.alive():
            u = min(1.0, (time.monotonic() - t0) / duration)
            self.set_target({j: start[j] + (goal[j] - start[j]) * _minjerk(u) for j in start})
            if u >= 1.0:
                return
            time.sleep(1.0 / SEND_HZ)

    def park(
        self,
        prefix: str,
        home: Mapping[str, float] | None = None,
        rest: Mapping[str, float] | None = None,
        speed: float = PARK_SPEED_RAD_S,
    ) -> bool:
        """Fold one arm (`prefix`) to its stowed pose by the driver's route; True once there.

        Does nothing to an arm already within 0.15 rad of it.
        """
        if not self.alive():
            return False
        home = dict(home or {})
        rest = dict(rest or REST_POSE)
        arm = [prefix + j for j in ARM_JOINTS if prefix + j in self.limits]
        grip = [prefix + GRIPPER] if prefix + GRIPPER in self.limits else []
        goal_home = {j: float(home.get(j, 0.0)) for j in arm + grip}

        def there() -> bool:
            q = self.positions(arm)
            return max(abs(q[j] - goal_home[j]) for j in arm) < PARK_TOLERANCE_RAD

        # Take over where the arm is, not at an old target.
        self.set_target(self.positions(arm + grip))
        if there():
            return True
        self.run_to({prefix + j: v for j, v in rest.items() if prefix + j in self.limits}, speed)
        self.run_to({j: goal_home[j] for j in arm if j[len(prefix) :] not in rest}, speed)
        self.run_to(goal_home, speed)
        deadline = time.monotonic() + 4.0
        while self.alive() and time.monotonic() < deadline:  # the position loop is soft: wait
            if there():
                return True
            time.sleep(0.05)
        return self.alive() and there()

    # -- the end ---------------------------------------------------------------------

    def hold(
        self,
        user: str,
        prefix: str,
        home: Mapping[str, float] | None = None,
        rest: Mapping[str, float] | None = None,
        speed: float = PARK_SPEED_RAD_S,
    ) -> None:
        """`user` holds an arm that must be parked before the session ends, whoever ends it."""
        with self._lock:
            self.parking[user] = (prefix, dict(home or {}), dict(rest) if rest else None, speed)

    def park_held(self) -> None:
        """Park every arm a user still holds, one after the other."""
        with self._lock:
            held = list(self.parking.items())
        for user, (prefix, home, rest, speed) in held:
            self.park(prefix, home, rest, speed)
            with self._lock:
                self.parking.pop(user, None)

    def release(self, user: str) -> None:
        """`user` is done; the last one ends the session (the bridge damps and disables)."""
        with self._lock:
            self.users.discard(user)
            self.parking.pop(user, None)
            last = not self.users
        if last:
            self.close("every arm let go")

    def close(self, why: str = "", *, stop_bridge: bool = True) -> None:
        """End the session (the bridge damps and disables), then stop a bridge started here,
        unless `stop_bridge` is False (zeroing reads the arm limp through the same bridge)."""
        if self.closed:
            return
        self.closed, self.why = True, why
        sock, self._sock = self._sock, None
        if sock is not None:
            with contextlib.suppress(OSError):
                sock.close()
        with SharedRuntime._guard:
            if SharedRuntime._registry.get(self.address) is self:
                del SharedRuntime._registry[self.address]
        if stop_bridge and self.bridge is not None:
            self.bridge.stop()


atexit.register(_park_all_at_exit)


def session_for(
    port: str,
    default_port: int,
    arm_config: str = "",
    *,
    python: str = "",
    profile: str = "mirror",
    grav_scale: float | None = None,
) -> SharedRuntime:
    """The shared session an arm names: the bridge of `arm_config` (started here when nothing
    listens), or a bridge someone runs at `port`."""
    if arm_config:
        bridge = ManagedBridge.for_config(
            arm_config, python=python, profile=profile, grav_scale=grav_scale
        )
        shared = SharedRuntime.for_address(bridge.address)
        shared.use_bridge(bridge)
        return shared
    return SharedRuntime.for_address(with_port(port, default_port))


class PrefixedBus:
    """One arm of a shared session, as the part of LeRobot's motor bus the plugin reads.

    Names are the hub card's (`joint1` … `gripper_joint`); the runtime's carry the arm's
    prefix (`follower_joint1`). `connect` opens the shared session (idempotent: the arm
    whose object connects first opens it for every arm that claimed joints), and
    `disconnect` parks this arm, then lets go of the session.
    """

    def __init__(
        self,
        shared: SharedRuntime,
        user: str,
        prefix: str,
        joints: Iterable[str],
        limits: Mapping[str, tuple[float, float] | list[float]],
        *,
        home: Mapping[str, float] | None = None,
        rest: Mapping[str, float] | None = None,
        park_on_disconnect: bool = True,
        park_speed: float = PARK_SPEED_RAD_S,
    ) -> None:
        from types import SimpleNamespace

        self.shared = shared
        self.user = user
        self.prefix = prefix
        self.joints = list(joints)
        self.home = {prefix + j: float(v) for j, v in (home or {}).items()}
        self.rest = dict(rest or REST_POSE)
        self.park_on_disconnect = park_on_disconnect
        self.park_speed = park_speed
        self.motors = {joint: SimpleNamespace(norm_mode="radians") for joint in self.joints}
        self.port_handler = None
        self._connected = False
        shared.claim(
            user,
            [prefix + j for j in self.joints],
            {prefix + j: limits[j] for j in self.joints},
            grippers=[prefix + j for j in self.joints if j == GRIPPER],
        )

    @property
    def is_connected(self) -> bool:
        return self._connected and self.shared.alive()

    @property
    def is_calibrated(self) -> bool:
        """The bridge owns calibration (each joint's sign and offset, in its yaml)."""
        return self._connected

    def connect(self, handshake: bool = True) -> None:
        self.shared.open()
        self._connected = True
        if self.park_on_disconnect:
            self.shared.hold(self.user, self.prefix, self.home, self.rest, self.park_speed)

    def enable_torque(self, motors: Any = None, num_retry: int = 0) -> None:
        """The shared session enables every claimed motor at its hello; nothing to do."""

    def disable_torque(self, motors: Any = None, num_retry: int = 0) -> None:
        """One arm cannot be disabled alone on a shared session; `disconnect` parks it."""

    def sync_read(
        self, data_name: str, motors: Any = None, normalize: bool = True
    ) -> dict[str, float]:
        if data_name != "Present_Position":
            raise NotImplementedError(f"The arm's runtime reports positions only, not {data_name}.")
        names = [motors] if isinstance(motors, str) else list(motors or self.joints)
        q = self.shared.positions([self.prefix + n for n in names])
        return {n: q[self.prefix + n] for n in names if self.prefix + n in q}

    def sync_write(
        self, data_name: str, values: Mapping[str, float], normalize: bool = True
    ) -> None:
        if data_name != "Goal_Position":
            raise NotImplementedError(
                f"The arm's runtime takes goal positions only, not {data_name}."
            )
        unknown = [n for n in values if n not in self.motors]
        if unknown:
            raise RuntimeRefused(f"No joint named {', '.join(unknown)} on this arm.")
        self.shared.set_target({self.prefix + n: v for n, v in values.items()})

    def park(self) -> bool:
        return self.shared.park(self.prefix, self.home, self.rest, self.park_speed)

    def disconnect(self, disable_torque: bool = True) -> None:
        try:
            if self._connected and self.park_on_disconnect and self.shared.alive():
                self.park()
        finally:
            self._connected = False
            self.shared.release(self.user)


__all__ = [
    "ARM_JOINTS",
    "GRIPPER",
    "GRIPPER_LEASH_RAD",
    "LEASH_RAD",
    "PARK_SPEED_RAD_S",
    "REST_POSE",
    "SEND_HZ",
    "VOUCH_DEPTH",
    "PrefixedBus",
    "SharedRuntime",
    "session_for",
]
