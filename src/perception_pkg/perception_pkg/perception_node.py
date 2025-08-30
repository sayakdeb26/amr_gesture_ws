#!/usr/bin/env python3
import os
import json
import time
import requests

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from amr_interfaces.msg import Intent, UnknownGesture


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Params
        self.vlm_url = os.getenv('VLM_URL', 'http://127.0.0.1:8000')
        # τ for confident LSTM intents (can be adjusted at runtime if you later expose it as a ROS param)
        self.tau = float(os.getenv('LSTM_TAU', '0.60'))

        # Pubs
        self.pub_intents = self.create_publisher(Intent, '/intents_raw', 10)

        # Subs
        self.sub_lstm_intent = self.create_subscription(
            Intent, '/intent_lstm', self.on_lstm_intent, 10
        )
        self.sub_lstm_unknown = self.create_subscription(
            UnknownGesture, '/lstm/unknown', self.on_lstm_unknown, 10
        )

        self.get_logger().info(f'perception_node up. VLM_URL={self.vlm_url}, τ={self.tau:.2f}')

    # ---------- LSTM confident branch ----------
    def on_lstm_intent(self, msg: Intent):
        """Forward confident LSTM intents directly to /intents_raw."""
        if msg.confidence >= self.tau:
            out = Intent()
            out.stamp = msg.stamp if msg.stamp.sec or msg.stamp.nanosec else Time()
            out.label = msg.label
            out.confidence = msg.confidence
            out.latency_ms = msg.latency_ms
            out.source = 'lstm'  # mark origin
            self.pub_intents.publish(out)
            self.get_logger().info(
                f'PASS-THROUGH from LSTM: {out.label} (conf={out.confidence:.2f}) → /intents_raw'
            )
        else:
            
            self.get_logger().warn(
                f'Received /intent_lstm with low conf={msg.confidence:.2f} < τ={self.tau:.2f}; ignoring here.'
            )

    # ---------- LSTM low-confidence branch (VLM fallback) ----------
    def on_lstm_unknown(self, msg: UnknownGesture):
        """Low-confidence from LSTM → call VLM → publish Intent."""
        self.get_logger().info(
            f'UnknownGesture received (conf={msg.confidence:.2f}, hint="{msg.hint}") … calling VLM'
        )
        try:
            payload = {
                "frames": [],          
                "context": {"hint": msg.hint or ""}
            }
            t0 = time.time()
            r = requests.post(f'{self.vlm_url}/infer', json=payload, timeout=5)
            r.raise_for_status()
            dt_ms = int((time.time() - t0) * 1000)
            data = r.json() if r.headers.get('content-type', '').startswith('application/json') else json.loads(r.text)

            label = data.get('label', 'UNKNOWN')
            conf = float(data.get('conf', 0.0))
            self.get_logger().info(f'VLM result: {label} conf={conf:.2f} lat={dt_ms}ms')

            out = Intent()
            out.stamp = Time()  
            out.label = label
            out.confidence = conf
            out.latency_ms = dt_ms
            out.source = 'vlm-http'
            self.pub_intents.publish(out)
            self.get_logger().info(
                f'Published /intents_raw (AUTO-APPROVED): {label} ({conf:.2f})'
            )
        except Exception as e:
            self.get_logger().warn(f'VLM call failed: {e}')

def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

