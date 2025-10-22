#!/usr/bin/env python3
import json, time, signal, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from amr_interfaces.msg import UnknownGesture, Intent, ConfirmRequest, ConfirmReply

class Mon(Node):
    def __init__(self):
        super().__init__('pipeline_monitor')
        self.create_subscription(Intent, '/intents_local', self.on_local, 10)
        self.create_subscription(Intent, '/intents_approved', self.on_approved, 10)
        self.create_subscription(UnknownGesture, '/lstm/unknown', self.on_unknown, 10)
        self.create_subscription(String, '/recorder/clip_ready', self.on_clip, 10)
        self.create_subscription(ConfirmRequest, '/vlm/confirm_request', self.on_req, 10)
        self.create_subscription(ConfirmReply, '/ui/confirm_reply', self.on_reply, 10)

        self.cnts = dict(local=0, approved=0, unknown=0, clip=0, req=0, reply=0)
        self.last = dict(local=None, approved=None, unknown=None, clip=None, req=None, reply=None)
        self._stop = threading.Event()
        threading.Thread(target=self._ticker, daemon=True).start()
        self.get_logger().info('monitor online')

    def _ticker(self):
        while not self._stop.is_set():
            time.sleep(5)
            l = self.last
            msg = (
                f"[5s] local={self.cnts['local']}, approved={self.cnts['approved']}, "
                f"unknown={self.cnts['unknown']}, clips={self.cnts['clip']}, "
                f"requests={self.cnts['req']}, replies={self.cnts['reply']}"
            )
            self.get_logger().info(msg)
            # brief last-seen snapshot
            if l['local']:
                self.get_logger().info(f" last local: {l['local']}")
            if l['approved']:
                self.get_logger().info(f" last approved: {l['approved']}")
            if l['req']:
                self.get_logger().info(f" last request: {l['req']}")
            if l['clip']:
                self.get_logger().info(f" last clip: {l['clip']}")

    def on_local(self, m: Intent):
        lab = getattr(m, "label", "")
        conf = getattr(m, "confidence", 0.0)
        self.cnts['local'] += 1
        self.last['local'] = f"{lab} ({conf:.2f})"
        self.get_logger().info(f"LSTM local → {lab} ({conf:.2f})")

    def on_approved(self, m: Intent):
        lab = getattr(m, "label", "")
        conf = getattr(m, "confidence", 0.0)
        self.cnts['approved'] += 1
        self.last['approved'] = f"{lab} ({conf:.2f})"
        self.get_logger().info(f"APPROVED → {lab} ({conf:.2f})")

    def on_unknown(self, m: UnknownGesture):
        conf = getattr(m, "confidence", 0.0)
        hint = getattr(m, "hint", "")
        self.cnts['unknown'] += 1
        self.last['unknown'] = f"{hint} ({conf:.2f})"
        self.get_logger().info(f"UNKNOWN → {hint} ({conf:.2f})")

    def on_clip(self, s: String):
        try:
            obj = json.loads(s.data)
            path = obj.get("clip_path", "")
            fps  = obj.get("fps", "?")
            self.last['clip'] = f"{path} ({fps}fps)"
        except Exception:
            self.last['clip'] = "<unparseable>"
        self.cnts['clip'] += 1
        self.get_logger().info(f"CLIP → {self.last['clip']}")

    def on_req(self, r: ConfirmRequest):
        lab = getattr(r, "candidate_label", "") or getattr(r, "label", "")
        conf = getattr(r, "candidate_conf", None)
        if conf is None:
            conf = getattr(r, "confidence", 0.0)
        self.cnts['req'] += 1
        self.last['req'] = f"{lab} ({conf:.2f})"
        self.get_logger().info(f"REQUEST → {lab} ({conf:.2f})")

    def on_reply(self, r: ConfirmReply):
        final = getattr(r, "final_label", "") or getattr(r, "label", "")
        approved = getattr(r, "approved", False)
        self.cnts['reply'] += 1
        self.last['reply'] = f"{approved} {final}"
        self.get_logger().info(f"REPLY → approved={approved} label={final}")

    def clean_shutdown(self):
        self._stop.set()

def main():
    rclpy.init()
    n = Mon()
    def _sigint(_s,_f):
        n.get_logger().info("monitor: shutting down...")
        n.clean_shutdown()
        n.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    signal.signal(signal.SIGINT, _sigint)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        _sigint(None, None)

if __name__ == '__main__':
    main()
