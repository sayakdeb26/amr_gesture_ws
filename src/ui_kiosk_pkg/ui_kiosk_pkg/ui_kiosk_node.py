#!/usr/bin/env python3
# Minimal web kiosk + ROS bridge for confirm workflow.

import json
import mimetypes
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from builtin_interfaces.msg import Time

from amr_interfaces.msg import ConfirmRequest, ConfirmReply


# --- Shared state between ROS and HTTP handler ---
class KioskState:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest: Optional[ConfirmRequest] = None

    def set_latest(self, req: ConfirmRequest):
        with self.lock:
            self.latest = req

    def get_latest(self) -> Optional[ConfirmRequest]:
        with self.lock:
            return self.latest


STATE = KioskState()


# --- HTTP server handler ---
class KioskHandler(BaseHTTPRequestHandler):
    server_version = "KioskHTTP/1.0"

    def _send(self, code=200, ctype="text/html; charset=utf-8"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            latest = STATE.get_latest()
            label = latest.candidate_label if latest else "—"
            conf = f"{latest.confidence:.2f}" if latest else "—"
            hint = latest.hint if latest else ""
            has_preview = bool(latest and latest.clip_relpath and os.path.exists(latest.clip_relpath))
            video_tag = (
                '<video controls width="640" src="/preview"></video>'
                if has_preview else "<p><i>(No preview clip available)</i></p>"
            )
            html = f"""<!doctype html>
<html><head><meta charset="utf-8"><title>Gesture Confirm</title></head>
<body style="font-family: system-ui, sans-serif; max-width: 760px; margin: 24px auto;">
  <h2>Candidate: <code>{label}</code> (conf={conf})</h2>
  <p>Hint: {hint}</p>
  {video_tag}
  <hr/>
  <form id="f" onsubmit="return false;">
    <label>Final label: <input id="final_label" value="{label}"/></label>
    <br/><br/>
    <button onclick="send(true)">Approve</button>
    <button onclick="send(false)">Reject</button>
  </form>
  <script>
    async function send(ok){{
      const final_label = document.getElementById('final_label').value || '{label}';
      const res = await fetch('/api/decision', {{
        method: 'POST',
        headers: {{'Content-Type': 'application/json'}},
        body: JSON.stringify({{approved: ok, final_label}})
      }});
      alert('Sent decision: ' + ok + ' / ' + final_label);
    }}
  </script>
</body></html>"""
            self._send(200)
            self.wfile.write(html.encode("utf-8"))
            return

        if self.path == "/healthz":
            self._send(200, "text/plain; charset=utf-8")
            self.wfile.write(b"ok")
            return

        if self.path == "/status.json":
            latest = STATE.get_latest()
            data = None
            if latest:
                data = {
                    "candidate_label": latest.candidate_label,
                    "confidence": float(latest.confidence),
                    "hint": latest.hint,
                    "clip_relpath": latest.clip_relpath,
                }
            self._send(200, "application/json")
            self.wfile.write(json.dumps({"latest": data}).encode("utf-8"))
            return

        if self.path == "/preview":
            latest = STATE.get_latest()
            if latest and latest.clip_relpath and os.path.exists(latest.clip_relpath):
                ctype, _ = mimetypes.guess_type(latest.clip_relpath)
                self._send(200, ctype or "application/octet-stream")
                with open(latest.clip_relpath, "rb") as f:
                    self.wfile.write(f.read())
                return
            self._send(404, "text/plain; charset=utf-8")
            self.wfile.write(b"no preview")
            return

        self._send(404, "text/plain; charset=utf-8")
        self.wfile.write(b"not found")

    def do_POST(self):
        if self.path != "/api/decision":
            self._send(404, "text/plain; charset=utf-8")
            self.wfile.write(b"not found")
            return

        ln = int(self.headers.get("Content-Length", "0") or 0)
        body = self.rfile.read(ln)
        try:
            payload = json.loads(body.decode("utf-8"))
            approved = bool(payload.get("approved"))
            final_label = str(payload.get("final_label") or "").strip()
        except Exception:
            self._send(400, "text/plain; charset=utf-8")
            self.wfile.write(b"bad json")
            return

        # Publish reply via ROS
        self.server.node.publish_reply(approved, final_label)  # type: ignore[attr-defined]

        self._send(200, "application/json")
        self.wfile.write(b'{"ok":true}')


class KioskHTTPServer(ThreadingHTTPServer):
    # We stash the ROS node so the handler can publish.
    def __init__(self, addr, handler, node):
        super().__init__(addr, handler)
        self.node = node


# --- ROS Node ---
class UIKioskNode(Node):
    def __init__(self):
        super().__init__("ui_kiosk_node")
        qos = QoSProfile(depth=10)

        self.declare_parameter("web_port", 8080)
        port = int(self.get_parameter("web_port").value)

        # Sub: requests from perception
        self.sub_req = self.create_subscription(
            ConfirmRequest, "/vlm/confirm_request", self.on_request, qos
        )
        # Pub: reply to perception
        self.pub_reply = self.create_publisher(ConfirmReply, "/ui/confirm_reply", qos)

        # Start HTTP server (thread)
        self.http = KioskHTTPServer(("0.0.0.0", port), KioskHandler, self)
        self.http_thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.http_thread.start()

        self.get_logger().info(f"ui_kiosk_node up. Web UI on http://127.0.0.1:{port}")

    def on_request(self, msg: ConfirmRequest):
        STATE.set_latest(msg)
        self.get_logger().info(
            f'ConfirmRequest: label={msg.candidate_label} conf={msg.confidence:.2f} '
            f'preview={"ok" if msg.clip_relpath else "none"}'
        )

    def publish_reply(self, approved: bool, final_label: str):
        rep = ConfirmReply()
        rep.stamp = self.get_clock().now().to_msg()  # type: ignore[assignment]
        rep.approved = approved
        rep.final_label = final_label or ""
        self.pub_reply.publish(rep)
        self.get_logger().info(
            f"Confirm reply: approved={approved}, final_label={rep.final_label}"
        )


def main():
    rclpy.init()
    node = UIKioskNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

