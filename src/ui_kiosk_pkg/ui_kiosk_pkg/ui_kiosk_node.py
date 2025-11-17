#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
import os, sys, json, time, shutil, threading, mimetypes, urllib.parse, socketserver
from http.server import HTTPServer, BaseHTTPRequestHandler
from datetime import datetime, timedelta
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vlm_interfaces.msg import ConfirmRequest, ConfirmReply
import cv2
import base64

def utc_ts():
    return int(time.time()*1000)

def safe_basename(p:str)->str:
    return os.path.basename(p).replace('\\','/')

class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.active = False
        self.session_id = ""
        self.candidate_label = ""
        self.candidate_conf = 0.0
        self.hint = ""                  # optional (leave blank if none)
        self.clip_src = ""              # browser path like /media/<file>.mp4
        self.clip_abspath = ""          # on-disk file in media_dir
        self.deadline_at_ms = 0
        self.auto_approve = False
        self.history = []               # list of dicts (max history_size)
        self.time_recv_ms = 0

class UiKiosk(Node):
    def __init__(self):
        super().__init__('ui_kiosk_node')
        p = self.declare_parameter
        self.http_host           = p('http_host', '0.0.0.0').value
        self.http_port           = int(p('http_port', 8008).value)
        self.media_dir           = p('media_dir', '/home/sayak/amr_kiosk_media').value
        self.history_size        = int(p('history_size', 5).value)
        self.video_w             = int(p('video_width', 960).value)
        self.video_h             = int(p('video_height', 540).value)
        self.janitor_enabled     = bool(p('janitor_enabled', True).value)
        self.janitor_interval_s  = int(p('janitor_interval_s', 3600).value)
        self.keep_days_approved  = int(p('keep_days_approved', 1).value)
        self.decision_timeout_s  = int(p('decision_timeout_s', 20).value)
        self.auto_approve        = bool(p('auto_approve_default', False).value)
        self.dev_mode            = bool(p('dev_mode', False).value)

        os.makedirs(self.media_dir, exist_ok=True)
        os.makedirs(os.path.join(self.media_dir, "approved"), exist_ok=True)
        os.makedirs(os.path.join(self.media_dir, "approved","keep"), exist_ok=True)
        os.makedirs(os.path.join(self.media_dir, "tmp"), exist_ok=True)

        self.state = State()
        self.state.auto_approve = self.auto_approve

        # ROS wiring
        self.sub_req = self.create_subscription(ConfirmRequest, "/vlm/confirm_request", self.on_confirm_request, 10)
        self.pub_reply = self.create_publisher(ConfirmReply, "/ui/confirm_reply", 10)
        self.pub_log = self.create_publisher(String, "/kiosk/decision_log", 10)

        # HTTP server in a thread
        self.httpd = None
        self.http_thread = threading.Thread(target=self._start_http, daemon=True)
        self.http_thread.start()

        # Timer to auto-timeout
        self.timer = self.create_timer(0.5, self._tick)

        # Janitor
        if self.janitor_enabled:
            self.janitor_thread = threading.Thread(target=self._janitor_loop, daemon=True)
            self.janitor_thread.start()

        self.get_logger().info(f"Kiosk ready at http://{self.http_host}:{self.http_port} (media_dir={self.media_dir})")

    # -------- HTTP helpers --------
    def _start_http(self):
        node_ref = self
        class Handler(BaseHTTPRequestHandler):
            def _send(self, code=200, headers=None, body=b""):
                self.send_response(code)
                if headers:
                    for k,v in headers.items():
                        self.send_header(k, v)
                self.end_headers()
                if body:
                    self.wfile.write(body)

            def log_message(self, fmt, *args):  # quieter
                node_ref.get_logger().debug("HTTP: " + fmt % args)

            def do_GET(self):
                if self.path == "/" or self.path.startswith("/index.html"):
                    body = node_ref._html_index().encode("utf-8")
                    return self._send(200, {"Content-Type":"text/html; charset=utf-8"}, body)
                elif self.path.startswith("/static/"):
                    return node_ref._serve_static(self)
                elif self.path == "/state":
                    return node_ref._serve_state(self)
                elif self.path.startswith("/media/"):
                    return node_ref._serve_media(self)
                else:
                    return self._send(404, {"Content-Type":"text/plain"}, b"not found")

            def do_POST(self):
                if self.path.startswith("/confirm"):
                    length = int(self.headers.get('Content-Length','0'))
                    raw = self.rfile.read(length) if length>0 else b""
                    try:
                        data = json.loads(raw.decode("utf-8") if raw else "{}")
                    except Exception:
                        data = {}
                    return node_ref._handle_confirm(self, data)
                elif self.path.startswith("/toggle_auto"):
                    with node_ref.state.lock:
                        node_ref.state.auto_approve = not node_ref.state.auto_approve
                    return self._send(200, {"Content-Type":"application/json"}, json.dumps({"auto_approve":node_ref.state.auto_approve}).encode("utf-8"))
                else:
                    return self._send(404, {"Content-Type":"text/plain"}, b"not found")

        class ReuseTCPServer(socketserver.TCPServer):
            allow_reuse_address = True

        try:
            self.httpd = ReuseTCPServer((self.http_host, self.http_port), Handler)
            self.httpd.serve_forever()
        except Exception as e:
            self.get_logger().error(f"HTTP server failed: {e}")

    def _html_index(self)->str:
        # If dev_mode and external file exists, serve it
        if self.dev_mode:
            try:
                here = Path(__file__).parent / "www" / "index.html"
                if here.exists():
                    return here.read_text(encoding="utf-8")
            except Exception:
                pass
        # Embedded minimal shell (you can still live-edit external files via VS Code; switch dev_mode=true to use them)
        return """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>KOLAMeRO UI Kiosk</title>
<style>
  :root {{
    --bgA: #0b0f1a; --bgB: #1a1440; --accent:#7c5cff; --ok:#32d583; --bad:#f97066; --muted:#9aa4b2;
  }}
  html,body {{ margin:0; height:100%; font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto; color:#e5e7eb; }}
  body {{ {{
    background: radial-gradient(1000px 600px at 20% 10%, rgba(124,92,255,0.25), transparent 60%),
                linear-gradient(135deg, var(--bgA), var(--bgB));
  }} }}
  .page {{ display:flex; height:100%; }}
  .main {{ flex:1; display:flex; flex-direction:column; align-items:center; padding:24px; gap:16px; }}
  .logos {{ position:fixed; top:12px; left:12px; right:12px; display:flex; justify-content:space-between; pointer-events:none; }}
  .logo {{ opacity:.9; filter: drop-shadow(0 2px 6px rgba(0,0,0,.4)); }}
  .logo img {{ height:40px; object-fit:contain; }}
  .logo.br {{ position:fixed; right:12px; bottom:12px; }}
  .video-box {{
    width:{self.video_w}px; height:{self.video_h}px; border:1px solid rgba(255,255,255,.2);
    border-radius:16px; display:grid; place-items:center; background:rgba(0,0,0,.35);
    box-shadow:0 10px 30px rgba(0,0,0,.35);
  }}
  video {{ width:100%; height:100%; border-radius:16px; object-fit:cover; }}
  .idle-mark {{ color:#94a3b8; font-size:18px; }}
  .panel {{ width:{self.video_w}px; display:flex; flex-direction:column; gap:10px; }}
  .row {{ display:flex; align-items:center; gap:12px; }}
  input[type=text] {{
    flex:1; background:#0f172a; color:#e5e7eb; border:1px solid rgba(255,255,255,.2);
    border-radius:10px; padding:10px 12px; font-size:16px;
  }}
  .pill {{ padding:6px 10px; border-radius:999px; background:rgba(255,255,255,.08); color:#e5e7eb; font-size:14px; }}
  .hint {{ color:#cbd5e1; font-size:14px; }}
  .bar {{ width:100%; height:10px; background:rgba(255,255,255,.08); border-radius:999px; overflow:hidden; }}
  .fill {{ height:100%; width:0%; background:linear-gradient(90deg, #4f46e5, #06b6d4); transition:width .2s; }}
  .actions {{ display:flex; gap:10px; }}
  button {{
    border:0; border-radius:10px; padding:10px 14px; font-weight:600; cursor:pointer;
    background:#1f2937; color:#e5e7eb;
  }}
  button.approve {{ background: #164e63; }}
  button.reject  {{ background: #4c0519; }}
  .toast {{ position:fixed; top:16px; left:50%; transform:translateX(-50%); background:#111827; color:#e5e7eb;
           padding:10px 14px; border-radius:10px; box-shadow:0 8px 30px rgba(0,0,0,.35); display:none; }}
  .aside {{ width:300px; border-left:1px solid rgba(255,255,255,.12); padding:16px; display:flex; flex-direction:column; gap:10px; }}
  .card {{ background:rgba(255,255,255,.06); padding:10px; border-radius:10px; font-size:14px; }}
  .status {{ font-size:13px; color:#9aa4b2; }}
</style>
</head>
<body>
<div class="page">
  <div class="main">
    <div class="logos">
      <div class="logo tl"><img src="/static/logo_kolamero.jpg" alt="KOLAMeRO"></div>
      <div class="logo tr"><img src="/static/logo_faps.png" alt="FAPS"></div>
    </div>
    
    <div class="video-box" id="vbox">
      <img id="clip" style="display:none; max-width:100%; max-height:100%; border-radius:16px; object-fit:contain"/>
      <div class="idle-mark" id="idle">Waiting for a clip…</div>
    </div>
    <div class="panel">
      <div class="row">
        <input id="label" type="text" placeholder="Label…"/>
        <div class="pill" id="conf">conf –</div>
      </div>
      <div class="row"><div class="hint" id="hint"></div></div>
      <div class="bar"><div class="fill" id="fill"></div></div>
      <div class="row actions">
        <button class="approve" id="approve">Approve</button> 
        <button class="reject"  id="reject">Reject</button>
        <div class="status"><input type="checkbox" id="auto"/> Auto-approve (debug)</div>
      </div>
      <div class="status" id="status"></div>
    </div>
  </div>
  <div class="aside">
    <div class="logo br"><img src="/static/logo_fau.png" alt="FAU"></div>
    <div class="card"><strong>Recent</strong><div id="hist"></div></div>
  </div>
</div>
<div class="toast" id="toast"></div>
<script>
let st = null, last_session = null;
const el = (id)=>document.getElementById(id);
function toast(msg){ const t=el('toast'); t.textContent=msg; t.style.display='block'; setTimeout(()=>t.style.display='none', 1200); }
function setVideo(src){
  const v=el('clip'), idle=el('idle');
  if(src){ v.src=src; v.style.display='block'; idle.style.display='none'; }
  else { v.removeAttribute('src'); v.style.display='none'; idle.style.display='grid'; }
}
async function pull(){
  try{
    const r = await fetch('/state'); if(!r.ok) throw 0;
    const s = await r.json(); st = s;
    el('label').value = s.label || '';
    el('conf').textContent = `conf ${ (s.confidence??0).toFixed(2) }`;
    el('hint').textContent = s.hint || '';
    el('auto').checked = !!s.auto_approve;
    setVideo(s.media_src || null);
    if(s.time_left_ms!==undefined && s.timeout_ms){
      const pct = Math.max(0, Math.min(100, 100*(s.time_left_ms/s.timeout_ms)));
      el('fill').style.width = pct+'%';
      el('status').textContent = s.media_src ? `time left: ${(s.time_left_ms/1000).toFixed(1)}s` : '';
    }
    if(last_session !== s.session_id){ last_session = s.session_id; el('label').focus(); }
    renderHist(s.history || []);
  }catch(e){
    // ignore transient errors
  } finally { setTimeout(pull, 1000); }
}
function renderHist(h){
  const box=el('hist'); box.innerHTML='';
  h.forEach(it=>{
    const d=document.createElement('div');
    d.className='status';
    d.textContent = `${it.outcome} · ${it.final_label||it.candidate_label||''} · ${new Date(it.ts).toLocaleTimeString()}`;
    box.appendChild(d);
  });
}
async function send(approved){
  if(!st || !st.session_id) return;
  const body = { session_id: st.session_id, approved: approved, final_label: el('label').value || '' };
  const r= await fetch('/confirm', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(body)});
  if(r.ok){ toast(approved?'Approved':'Rejected'); }
}
el('approve').onclick = ()=>send(true);
el('reject').onclick  = ()=>send(false);
pull();
</script>
</body></html>
"""
    # -------- HTTP route impls --------
    def _serve_state(self, handler:BaseHTTPRequestHandler):
        with self.state.lock:
            now = utc_ts()
            time_left = max(0, self.state.deadline_at_ms - now) if self.state.active else 0
            payload = {
                "session_id": self.state.session_id if self.state.active else "",
                "label": self.state.candidate_label if self.state.active else "",
                "confidence": float(self.state.candidate_conf) if self.state.active else 0.0,
                "hint": self.state.hint if self.state.active else "",
                "media_src": self.state.clip_src if self.state.active else "",
                "auto_approve": self.state.auto_approve,
                "timeout_ms": self.decision_timeout_s*1000,
                "time_left_ms": time_left,
                "history": self.state.history[-self.history_size:]
            }
        body = json.dumps(payload).encode("utf-8")
        handler._send(200, {"Content-Type":"application/json"}, body)

    def _serve_static(self, handler:BaseHTTPRequestHandler):
        # serve VSCode assets if present
        base = Path(__file__).parent / "www"
        req = handler.path[len("/static/"):]
        target = base / "static" / req
        if not target.exists() or not target.is_file():
            return handler._send(404, {"Content-Type":"text/plain"}, b"static not found")
        mime,_ = mimetypes.guess_type(str(target))
        try:
            data = target.read_bytes()
            return handler._send(200, {"Content-Type": mime or "application/octet-stream"}, data)
        except Exception as e:
            self.get_logger().warn(f"static error: {e}")
            return handler._send(500, {"Content-Type":"text/plain"}, b"static error")

    def _serve_media(self, handler:BaseHTTPRequestHandler):
        rel = handler.path[len("/media/"):]
        target = os.path.join(self.media_dir, rel)
        if not os.path.isfile(target):
            return handler._send(404, {"Content-Type":"text/plain"}, b"media not found")
        mime,_ = mimetypes.guess_type(target)
        try:
            with open(target, "rb") as f:
                data = f.read()
            return handler._send(200, {"Content-Type": mime or "video/mp4"}, data)
        except Exception as e:
            self.get_logger().warn(f"media error: {e}")
            return handler._send(500, {"Content-Type":"text/plain"}, b"media error")

    def _handle_confirm(self, handler:BaseHTTPRequestHandler, data:dict):
        approved = bool(data.get("approved", False))
        final_label = str(data.get("final_label","")).strip()
        session_id = str(data.get("session_id","")).strip()

        with self.state.lock:
            if not self.state.active or session_id != self.state.session_id:
                return handler._send(409, {"Content-Type":"application/json"}, json.dumps({"ok":False, "reason":"no-active-or-mismatch"}).encode("utf-8"))
            start_ms = self.state.time_recv_ms
            clip_abs = self.state.clip_abspath
            candidate_label = self.state.candidate_label
            candidate_conf = float(self.state.candidate_conf)

        # Publish ConfirmReply
        msg = ConfirmReply()
        msg.session_id = session_id
        msg.approved = approved
        msg.final_label = final_label if final_label else candidate_label
        self.pub_reply.publish(msg)

        # Decision log
        outcome = "approved" if approved else "rejected"
        latency_ms = max(0, utc_ts() - start_ms)
        log = {
            "ts": utc_ts(),
            "session_id": session_id,
            "outcome": outcome,
            "candidate_label": candidate_label,
            "final_label": msg.final_label,
            "candidate_conf": candidate_conf,
            "latency_ms": latency_ms,
            "clip_name": safe_basename(clip_abs)
        }
        self.pub_log.publish(String(data=json.dumps(log)))

        # Retention
        try:
            if approved:
                # keep for keep_days_approved (janitor will purge later)
                # move into approved/ (not keep/) – training job can move to keep/ if desired
                dst = os.path.join(self.media_dir, "approved", safe_basename(clip_abs))
                if clip_abs and os.path.exists(clip_abs):
                    if os.path.abspath(clip_abs) != os.path.abspath(dst):
                        shutil.move(clip_abs, dst)
            else:
                # delete immediately
                if clip_abs and os.path.exists(clip_abs):
                    os.remove(clip_abs)
        except Exception as e:
            self.get_logger().warn(f"retention op failed: {e}")

        # Clear state
        with self.state.lock:
            self._push_history(log)
            self._clear_active()

        return handler._send(200, {"Content-Type":"application/json"}, json.dumps({"ok":True}).encode("utf-8"))

    def _push_history(self, item):
        self.state.history.append(item)
        if len(self.state.history) > self.history_size:
            self.state.history = self.state.history[-self.history_size:]

    def _clear_active(self):
        self.state.active = False
        self.state.session_id = ""
        self.state.candidate_label = ""
        self.state.candidate_conf = 0.0
        self.state.hint = ""
        self.state.clip_src = ""
        self.state.clip_abspath = ""
        self.state.deadline_at_ms = 0
        self.state.time_recv_ms = 0

    # -------- ROS callbacks --------

    def on_confirm_request(self, req: ConfirmRequest):
        
        # Map fields from the VLM bridge
        self._current_request = {
            "session_id": msg.session_id,
            "window_id": msg.window_id,
            "label": msg.label,
            "confidence": msg.confidence,
            "clip_url": self._relativize_clip(msg.clip_path),
            "preview_frame_b64": msg.preview_frame_b64 or "",
        }

        # Validate clip_path
        if not src_path or not os.path.isfile(src_path):
            self.get_logger().warn(f"ConfirmRequest missing/invalid clip_path: {src_path}")
            return

        try:
            # 1) Copy the MP4 into media_dir (for retention / training)
            base = safe_basename(src_path)
            dst_mp4 = os.path.join(self.media_dir, base)
            if os.path.abspath(src_path) != os.path.abspath(dst_mp4):
                shutil.copy2(src_path, dst_mp4)

            # 2) Extract middle frame as JPEG preview
            preview_name = base.rsplit(".", 1)[0] + "_preview.jpg"
            preview_path = os.path.join(self.media_dir, preview_name)

            try:
                cap = cv2.VideoCapture(dst_mp4)
                total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                mid = total // 2 if total > 0 else 0
                cap.set(cv2.CAP_PROP_POS_FRAMES, mid)
                ok, frame = cap.read()
                cap.release()
                if ok:
                    cv2.imwrite(preview_path, frame)
                else:
                    preview_path = ""  # fallback to MP4 thumbnail if needed
            except Exception as e:
                self.get_logger().warn(f"preview extract failed: {e}")
                preview_path = ""

            # 3) Update kiosk state (point UI at the preview frame)
            with self.state.lock:
                self.state.active = True
                self.state.session_id = session_id
                self.state.candidate_label = label
                self.state.candidate_conf = conf
                self.state.hint = ""
                self.state.clip_abspath = dst_mp4
                # If preview exists, serve that; otherwise, still show the MP4
                if preview_path and os.path.exists(preview_path):
                    self.state.clip_src = "/media/" + safe_basename(preview_path)
                else:
                    self.state.clip_src = "/media/" + safe_basename(dst_mp4)
                self.state.time_recv_ms = utc_ts()
                self.state.deadline_at_ms = self.state.time_recv_ms + self.decision_timeout_s * 1000

            self.get_logger().info(
                f"ConfirmRequest {session_id} → {base} (conf={conf:.2f})"
            )

        except Exception as e:
            self.get_logger().error(f"on_confirm_request error: {e}")


    # -------- housekeeping --------
    def _tick(self):
        # auto-approve on timeout (if auto_approve ON)
        with self.state.lock:
            if not self.state.active: return
            now = utc_ts()
            if now >= self.state.deadline_at_ms:
                if self.state.auto_approve:
                    # synthesize approve with current label
                    msg = ConfirmReply()
                    msg.session_id = self.state.session_id
                    msg.approved = True
                    msg.final_label = self.state.candidate_label
                    self.pub_reply.publish(msg)
                    self._push_history({"ts":utc_ts(), "session_id":self.state.session_id,
                        "outcome":"auto-approved", "candidate_label":self.state.candidate_label,
                        "final_label":self.state.candidate_label, "candidate_conf":self.state.candidate_conf,
                        "latency_ms": self.decision_timeout_s*1000, "clip_name": safe_basename(self.state.clip_abspath)})
                    # keep for 1 day; move to approved/
                    try:
                        dst = os.path.join(self.media_dir, "approved", safe_basename(self.state.clip_abspath))
                        if self.state.clip_abspath and os.path.exists(self.state.clip_abspath):
                            if os.path.abspath(self.state.clip_abspath) != os.path.abspath(dst):
                                shutil.move(self.state.clip_abspath, dst)
                    except Exception as e:
                        self.get_logger().warn(f"auto-approve retention move failed: {e}")
                    self._clear_active()
                else:
                    # timeout → delete immediately
                    try:
                        if self.state.clip_abspath and os.path.exists(self.state.clip_abspath):
                            os.remove(self.state.clip_abspath)
                    except Exception as e:
                        self.get_logger().warn(f"timeout delete failed: {e}")
                    self._push_history({"ts":utc_ts(), "session_id":self.state.session_id,
                        "outcome":"timed-out", "candidate_label":self.state.candidate_label,
                        "final_label":"", "candidate_conf":self.state.candidate_conf,
                        "latency_ms": self.decision_timeout_s*1000, "clip_name": safe_basename(self.state.clip_abspath)})
                    self._clear_active()

    def _janitor_loop(self):
        while rclpy.ok():
            try:
                # delete approved older than keep_days_approved
                cutoff = time.time() - self.keep_days_approved*86400
                for root, dirs, files in os.walk(self.media_dir):
                    for f in files:
                        path = os.path.join(root, f)
                        if "approved/keep" in path:  # preserved by training
                            continue
                        try:
                            st = os.stat(path)
                            if st.st_mtime < cutoff:
                                os.remove(path)
                        except Exception:
                            pass
            except Exception as e:
                self.get_logger().warn(f"janitor: {e}")
            time.sleep(self.janitor_interval_s)

def main():
    rclpy.init()
    node = UiKiosk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
