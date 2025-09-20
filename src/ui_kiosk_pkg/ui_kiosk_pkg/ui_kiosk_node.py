#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# UI Kiosk (ROS 2) — review & approve VLM label suggestions (with auto-approve toggle & countdown)
#
# Topics:
#   Sub: /vlm/confirm_request  (amr_interfaces/msg/ConfirmRequest)
#   Pub: /ui/confirm_reply     (amr_interfaces/msg/ConfirmReply)
#
# HTTP:
#   GET  /            → main page
#   GET  /state       → JSON {pending, label, conf, hint, src, last, seq, auto_approve, remain_s}
#   POST /vote        → {action=approve|reject, label=...}
#   POST /toggle_auto → flips auto_approve

import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from amr_interfaces.msg import ConfirmRequest, ConfirmReply


HTML = """<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<title>Gesture Label Kiosk</title>
<style>
  body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu;max-width:900px;margin:24px auto;padding:0 16px}
  header{display:flex;align-items:center;gap:16px;flex-wrap:wrap}
  h1{margin:0}
  .status{opacity:.8}
  .chip{padding:6px 10px;border-radius:999px;font-size:13px;border:1px solid #e2e8f0;background:#f8fafc}
  .on{background:#dcfce7;border-color:#86efac}
  .off{background:#fee2e2;border-color:#fecaca}
  .card{border:1px solid #e2e8f0;border-radius:12px;padding:16px;margin-top:16px;box-shadow:0 1px 2px rgba(0,0,0,.04)}
  .row{display:grid;grid-template-columns:1fr auto auto auto;gap:16px;align-items:end}
  label{display:block;margin:0 0 6px;font-size:12px;opacity:.8}
  input[type=text]{font-size:16px;padding:8px 10px;border:1px solid #cbd5e1;border-radius:8px;width:100%}
  .muted{opacity:.7}
  .buttons{display:flex;gap:12px;margin-top:12px}
  button{font-size:16px;padding:10px 14px;border:0;border-radius:10px;cursor:pointer}
  .approve{background:#16a34a;color:white}
  .reject{background:#ef4444;color:white}
  .disabled{filter:grayscale(60%);opacity:.6;pointer-events:none}
  form.inline{display:inline}
  footer{margin-top:28px;font-size:12px;opacity:.6}
  code{background:#f1f5f9;padding:2px 6px;border-radius:6px}
</style>
</head>
<body>
<header>
  <h1>Gesture Label Kiosk</h1>
  <div class="status">status: <span id="status">connecting…</span></div>
  <form class="inline" method="post" action="/toggle_auto"><button id="autoBtn" class="chip off" title="Toggle auto-approve">Auto-approve: OFF</button></form>
</header>

<div class="card">
  <form id="vote" method="post" action="/vote" class="row">
    <div>
      <label for="label">Suggested label</label>
      <input id="label" name="label" type="text" value="">
    </div>
    <div>
      <label>Confidence</label>
      <div id="conf" class="muted">–</div>
    </div>
    <div>
      <label>Hint</label>
      <div id="hint" class="muted">–</div>
    </div>
    <div>
      <label>Source</label>
      <div id="src" class="muted">–</div>
    </div>
    <input type="hidden" id="action" name="action" value="">
  </form>

  <div class="buttons">
    <button class="approve" onclick="submitVote('approve')" id="btnApprove">Approve</button>
    <button class="reject"  onclick="submitVote('reject')"  id="btnReject">Reject</button>
  </div>
</div>

<footer>
  Page polls <code>/state</code> every 500 ms. The label field won’t be overwritten while you’re typing; it refreshes only when a new request arrives or after you vote.
</footer>

<script>
  let userEditing = false;
  let lastSeq = -1;

  const labelEl = document.getElementById('label');
  labelEl.addEventListener('input', () => { userEditing = true; });
  labelEl.addEventListener('focus', () => { userEditing = true; });
  labelEl.addEventListener('blur',  () => { userEditing = false; });

  function submitVote(kind){
    document.getElementById('action').value = kind;
    document.getElementById('vote').submit();
  }

  function setButtonsEnabled(enabled){
    for (const id of ['btnApprove','btnReject']){
      const el = document.getElementById(id);
      el.disabled = !enabled;
      if (enabled) el.classList.remove('disabled'); else el.classList.add('disabled');
    }
    labelEl.disabled = !enabled;
  }

  async function refresh(){
    try{
      const r = await fetch('/state', {cache:'no-store'});
      const s = await r.json();

      // status + auto-approve chip + countdown seconds
      const st = document.getElementById('status');
      st.textContent = s.pending ? ('pending… (' + (s.remain_s ?? 0) + 's)') : (s.last || 'idle');

      const autoBtn = document.getElementById('autoBtn');
      if (s.auto_approve){ autoBtn.classList.add('on'); autoBtn.classList.remove('off'); autoBtn.textContent='Auto-approve: ON'; }
      else               { autoBtn.classList.add('off'); autoBtn.classList.remove('on'); autoBtn.textContent='Auto-approve: OFF'; }

      // Enable/disable controls based on pending
      setButtonsEnabled(!!s.pending);

      // Only refresh editable fields when a NEW request arrives (seq changed)
      if (s.seq !== lastSeq){
        lastSeq = s.seq;
        userEditing = false; // reset dirty flag on new item
        labelEl.value = s.label || '';
        document.getElementById('conf').textContent = s.conf ?? '–';
        document.getElementById('hint').textContent = s.hint ?? '–';
        document.getElementById('src').textContent  = s.src  ?? '–';
      }else{
        // Update non-editables
        document.getElementById('conf').textContent = s.conf ?? '–';
        document.getElementById('hint').textContent = s.hint ?? '–';
        document.getElementById('src').textContent  = s.src  ?? '–';
        if (!userEditing){
          labelEl.value = s.label || '';
        }
      }
    }catch(e){ /* ignore transient errors */ }
    setTimeout(refresh, 500);
  }
  refresh();
</script>
</body>
</html>
"""

def now_stamp(node: Node):
    return node.get_clock().now().to_msg()

def now_sec(node: Node) -> float:
    t = node.get_clock().now().nanoseconds
    return t / 1e9


class UIKioskNode(Node):
    def __init__(self):
        super().__init__('ui_kiosk_node')

        # Params
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8008)
        self.declare_parameter('auto_approve', False)
        # local UX deadline (should match the bridge confirm_timeout_s)
        self.declare_parameter('deadline_s', 20.0)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.auto_approve = bool(self.get_parameter('auto_approve').value)
        self.deadline_s = float(self.get_parameter('deadline_s').value)

        qos = QoSProfile(depth=10)
        self.sub = self.create_subscription(
            ConfirmRequest, '/vlm/confirm_request', self.on_request, qos
        )
        self.pub = self.create_publisher(ConfirmReply, '/ui/confirm_reply', qos)

        # State
        self.pending: Optional[ConfirmRequest] = None
        self.seq = 0
        self.last_event = ""
        self.deadline_at: float = 0.0  # wall time (sec) when current item expires

        # HTTP server
        self._http = ThreadingHTTPServer((self.host, self.port), UIHTTP)
        self._http.RequestHandlerClass.kiosk = self
        self._srv_thread = threading.Thread(target=self._http.serve_forever, daemon=True)
        self._srv_thread.start()

        self.get_logger().info(
            f'ui_kiosk_node up at http://{self.host}:{self.port}/ '
            f'(auto_approve={self.auto_approve}, deadline_s={self.deadline_s:.1f})'
        )

    # ROS callbacks
    def on_request(self, msg: ConfirmRequest):
        self.pending = msg
        self.seq += 1
        self.deadline_at = now_sec(self) + self.deadline_s
        label = msg.candidate_label or ''
        conf = float(getattr(msg, 'candidate_conf', 0.0))
        hint = msg.hint or ''
        self.get_logger().info(f'ConfirmRequest: label={label} conf={conf:.2f} hint={hint}')
        if self.auto_approve:
            self.apply_vote('approve', label)


    # add this helper **inside** UIKioskNode (above apply_vote is fine)
    def _normalize_label(self, s: str) -> str:
        """
        Normalize operator-entered labels to a consistent format.
        Example: "No stop" -> "NO_STOP"
                 "  wave-stop  " -> "WAVE-STOP"  (keeps existing hyphens)
        """
        s = s.strip()
        # Replace any run of whitespace with single underscore
        s = "_".join(s.split())
        return s.upper()


    def apply_vote(self, action: str, label: str) -> bool:
        if self.pending is None:
            self.get_logger().warn('apply_vote called with no pending request.')
            return False

        rep = ConfirmReply()
        rep.stamp = now_stamp(self)

        if action == 'approve':
            # If operator left the field empty, fall back to the suggested label.
            raw_label = label or (self.pending.candidate_label or '')
            final = self._normalize_label(raw_label)

            rep.approved = True
            rep.final_label = final
            self.last_event = f'Approved: {final}'
            self.get_logger().info(self.last_event)
        else:
            rep.approved = False
            rep.final_label = ''
            self.last_event = 'Rejected'
            self.get_logger().info(self.last_event)

        # Publish reply and clear the pending item
        self.pub.publish(rep)
        self.pending = None
        self.seq += 1
        return True
    

    def toggle_auto(self):
        self.auto_approve = not self.auto_approve
        self.get_logger().info(f'auto_approve => {self.auto_approve}')
        try:
            self.set_parameters([rclpy.parameter.Parameter(
                'auto_approve', rclpy.parameter.Parameter.Type.BOOL, self.auto_approve
            )])
        except Exception:
            pass

    def remaining_seconds(self) -> int:
        if not self.pending:
            return 0
        rem = int(max(0.0, self.deadline_at - now_sec(self)))
        return rem

    def destroy_node(self):
        try:
            self._http.shutdown()
            self._http.server_close()
        except Exception:
            pass
        return super().destroy_node()


class UIHTTP(BaseHTTPRequestHandler):
    kiosk: UIKioskNode = None  # injected

    def log_message(self, *_):
        pass

    def _reply(self, code, body, ctype="text/html; charset=utf-8"):
    	self.send_response(code)
    	self.send_header("Content-Type", ctype)
    	self.end_headers()
    try:
        self.wfile.write(body.encode("utf-8") if isinstance(body, str) else body)
    except BrokenPipeError:
        pass  # client went away; ignore


    def _redirect(self, where="/"):
        self.send_response(303)
        self.send_header("Location", where)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", "0")
        self.end_headers()

    # Routes
    def do_GET(self):
        if self.path.startswith("/state"):
            k = self.kiosk
            p = k.pending
            out = {
                "pending": p is not None,
                "label": (p.candidate_label if p else ""),
                "conf": (float(getattr(p, "candidate_conf", 0.0)) if p else None),
                "hint": (p.hint if p else None),
                "src":  (getattr(p, "source", "") if p else None),
                "last": k.last_event or "",
                "seq":  k.seq,
                "auto_approve": k.auto_approve,
                "remain_s": k.remaining_seconds(),
            }
            return self._reply(200, json.dumps(out), "application/json")
        return self._reply(200, HTML)

    def do_POST(self):
        ln = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(ln).decode("utf-8")
        form = urllib.parse.parse_qs(raw)

        if self.path.startswith("/vote"):
            action = (form.get("action", [""])[0] or "").lower()
            label = form.get("label", [""])[0]
            self.kiosk.apply_vote(action, label)
            return self._redirect("/")

        if self.path.startswith("/toggle_auto"):
            self.kiosk.toggle_auto()
            return self._redirect("/")

        return self._reply(404, "not found", "text/plain; charset=utf-8")


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

