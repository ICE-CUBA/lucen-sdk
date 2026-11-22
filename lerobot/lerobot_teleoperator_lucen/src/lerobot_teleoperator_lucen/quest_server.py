# ruff: noqa: E501 - the WebXR page below is HTML and JavaScript held in a string
"""Where a Meta Quest's hand pose arrives: a loopback endpoint, one paired headset, a secret.

A pose here moves a real arm, so the endpoint is strict:

- **Loopback by default.** It binds 127.0.0.1. The headset reaches it over USB
  with `adb reverse tcp:PORT tcp:PORT`, and then its browser sees
  `http://localhost:PORT`, which the browser treats as a secure origin, so WebXR
  works without a certificate. Any other address is refused unless
  `allow_remote` is set, and then it is said loudly, every start.
- **A secret per session.** Generated at start, printed once for the person,
  carried in the page's URL fragment (never sent as part of the URL), and
  required on every pose message.
- **One headset.** The first client that presents the secret is paired; every
  other client is refused for the rest of the session, secret or not.
- **Numbers only.** A pose is 16 finite numbers, the trigger a finite number in
  [0, 1]; anything else is refused and changes nothing. A pose older than
  `stale_s` reads as no pose, so a headset that stops sending lets the arm hold.

The page it serves is a minimal WebXR client: the right controller's grip pose,
the grip button as the deadman, the trigger. Its exact button mapping is
confirmed with the headset on the bench.
"""

from __future__ import annotations

import hmac
import ipaddress
import json
import logging
import math
import secrets
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Final

import numpy as np

logger = logging.getLogger(__name__)

MAX_BODY: Final[int] = 4096
#: Seconds a client may take to send its request before the connection is dropped.
READ_TIMEOUT_S: Final[float] = 2.0
STALE_S: Final[float] = 0.3


class QuestServerRefused(ValueError):
    """The endpoint will not start as asked; the message says why."""


def is_loopback(host: str) -> bool:
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def parse_pose(body: dict[str, Any]) -> tuple[np.ndarray, dict[str, Any]] | None:
    """`(4 by 4 pose, buttons)` from a message, or None when anything is not a finite number."""
    pose = body.get("pose")
    if not isinstance(pose, list) or len(pose) != 16:
        return None
    values: list[float] = []
    for v in pose:
        if isinstance(v, bool) or not isinstance(v, int | float) or not math.isfinite(v):
            return None
        values.append(float(v))
    raw = body.get("buttons")
    buttons: dict[str, Any] = {}
    if isinstance(raw, dict):
        if "move" in raw:
            buttons["move"] = raw["move"] is True
        if "scale" in raw:
            pull = raw["scale"]
            if isinstance(pull, bool) or not isinstance(pull, int | float):
                return None
            if not math.isfinite(pull) or not 0.0 <= float(pull) <= 1.0:
                return None
            buttons["scale"] = float(pull)
    # WebXR's matrices are column-major.
    return np.array(values, dtype=float).reshape(4, 4).T, buttons


class QuestPoseServer:
    """The pose endpoint, on its own thread: `latest()` is what the teleoperator reads."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8777,
        *,
        allow_remote: bool = False,
        stale_s: float = STALE_S,
        clock: Any = time.monotonic,
    ) -> None:
        if not is_loopback(host):
            if not allow_remote:
                raise QuestServerRefused(
                    f"The Quest endpoint binds {host!r}, which other machines can reach: anyone "
                    "there could send poses that move the arm. Use 127.0.0.1 with `adb reverse`, "
                    "or set allow_remote to accept the risk."
                )
            warning = (
                f"WARNING: the Quest endpoint listens on {host}:{port}, reachable from other "
                "machines. Only the paired headset's poses are taken, but the network sees them."
            )
            logger.warning(warning)
            print(warning, file=sys.stderr, flush=True)
        self.secret = secrets.token_urlsafe(16)
        self.stale_s = stale_s
        self.clock = clock
        self.paired: str | None = None
        self.refused = 0
        self._pose: np.ndarray | None = None
        self._buttons: dict[str, Any] = {}
        self._at = 0.0
        self._lock = threading.Lock()
        self._server = ThreadingHTTPServer((host, port), self._handler())
        self._server.daemon_threads = True
        self.host = host
        self.port = int(self._server.server_address[1])
        self._thread = threading.Thread(
            target=self._server.serve_forever, name="quest-pose", daemon=True
        )

    @property
    def page_url(self) -> str:
        return f"http://localhost:{self.port}/#secret={self.secret}"

    def start(self) -> QuestPoseServer:
        self._thread.start()
        return self

    def stop(self) -> None:
        self._server.shutdown()
        self._server.server_close()

    def latest(self) -> tuple[np.ndarray | None, dict[str, Any]]:
        with self._lock:
            if self._pose is None or self.clock() - self._at > self.stale_s:
                return None, {}
            return self._pose.copy(), dict(self._buttons)

    def offer(self, client: str, body: dict[str, Any]) -> int:
        """One pose message from `client`: the HTTP status it earns (200, 400, 403)."""
        given = body.get("secret")
        if not isinstance(given, str) or not hmac.compare_digest(given, self.secret):
            self.refused += 1
            return 403
        with self._lock:
            if self.paired is None:
                self.paired = client
            elif self.paired != client:
                self.refused += 1
                return 403
        parsed = parse_pose(body)
        if parsed is None:
            return 400
        with self._lock:
            self._pose, self._buttons = parsed
            self._at = self.clock()
        return 200

    def _handler(self) -> type[BaseHTTPRequestHandler]:
        server = self

        class Handler(BaseHTTPRequestHandler):
            #: A client that sends slowly is dropped, never held for: the socket times out.
            timeout = READ_TIMEOUT_S

            def log_message(self, format: str, *args: Any) -> None:
                return None

            def _reply(self, status: int, body: bytes, ctype: str) -> None:
                self.send_response(status)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("Referrer-Policy", "no-referrer")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:
                if self.path.split("#")[0] != "/":
                    self._reply(404, b"", "text/plain")
                    return
                self._reply(200, PAGE.encode(), "text/html; charset=utf-8")

            def do_POST(self) -> None:
                if self.path != "/pose":
                    self._reply(404, b"", "text/plain")
                    return
                try:
                    length = int(self.headers.get("Content-Length") or 0)
                except ValueError:
                    self._reply(400, b"", "text/plain")
                    return
                if length < 0:
                    # `rfile.read(-1)` reads to the end, past any cap: refused outright.
                    self._reply(400, b"", "text/plain")
                    return
                if length > MAX_BODY:
                    self._reply(413, b"", "text/plain")
                    return
                try:
                    raw = self.rfile.read(length)
                except TimeoutError:
                    self.close_connection = True
                    return
                if len(raw) != length:
                    self._reply(400, b"", "text/plain")
                    return
                try:
                    body = json.loads(raw or b"{}")
                except ValueError:
                    self._reply(400, b"", "text/plain")
                    return
                if not isinstance(body, dict):
                    self._reply(400, b"", "text/plain")
                    return
                client = f"{self.client_address[0]}|{body.get('client', '')}"[:128]
                status = server.offer(client, body)
                self._reply(status, b"", "text/plain")

        return Handler


# The page is HTML and JavaScript in a string; its lines are the browser's, not Python's.
PAGE: Final[str] = """<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Lucen arm · Quest</title></head>
<body style="font:16px system-ui;margin:24px"><p id="say">Press Start, then hold the grip to move the arm.</p>
<button id="go" style="font-size:20px;padding:12px 20px">Start</button>
<script>
const secret = new URLSearchParams(location.hash.slice(1)).get("secret") || "";
history.replaceState(null, "", "/");
const client = crypto.getRandomValues(new Uint32Array(2)).join("-");
document.getElementById("go").onclick = async () => {
  const session = await navigator.xr.requestSession("immersive-vr");
  const ref = await session.requestReferenceSpace("local-floor");
  session.updateRenderState({ baseLayer: new XRWebGLLayer(session, document.createElement("canvas").getContext("webgl", { xrCompatible: true })) });
  session.requestAnimationFrame(function frame(t, xr) {
    session.requestAnimationFrame(frame);
    for (const source of session.inputSources) {
      if (source.handedness !== "right" || !source.gripSpace) continue;
      const pose = xr.getPose(source.gripSpace, ref);
      if (!pose) continue;
      const pad = source.gamepad;
      fetch("/pose", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({
        secret, client, pose: Array.from(pose.transform.matrix),
        buttons: { move: Boolean(pad && pad.buttons[1] && pad.buttons[1].pressed), scale: pad && pad.buttons[0] ? pad.buttons[0].value : 0 },
      }) });
    }
  });
};
</script></body></html>
"""


__all__ = ["QuestPoseServer", "QuestServerRefused", "is_loopback", "parse_pose"]
