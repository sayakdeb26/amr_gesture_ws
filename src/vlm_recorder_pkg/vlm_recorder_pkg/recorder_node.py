# Package: vlm_recorder_pkg
# File: vlm_recorder_pkg/recorder_node.py
# ROS 2 (Humble) Python node that buffers camera frames and writes a 4s@10fps
# MP4 clip when an /lstm/unknown event arrives. It then publishes a simple JSON
# message with the clip path + window_id on /recorder/clip_ready.
#
# Drop-in defaults match your Phase-1 spec:
#   - 4.0 s buffer, 10 FPS, 320x240, H.264 (CRF 28)
#   - Writes to: ~/amr_gesture_ws/data/training/samples/
#
# Usage:
#   ros2 run vlm_recorder_pkg recorder_node
# or via launch file in this package.

import os
import cv2
import json
import time
import shlex
import queue
import errno
import signal
import shutil
import subprocess
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String

# If your custom interface exists, we can optionally import it; otherwise we stay generic.
try:
    from amr_interfaces.msg import UnknownGesture  # noqa: F401
    HAS_AMR_UNKNOWN = True
except Exception:
    HAS_AMR_UNKNOWN = False


class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')

        # --- Parameters (with your Phase-1 defaults) ---
        self.declare_parameter('image_topic', '/image_raw_30hz')
        self.declare_parameter('target_fps', 10)
        self.declare_parameter('buffer_seconds', 4.0)
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('data_root', os.path.expanduser('~/amr_gesture_ws/data/training'))
        self.declare_parameter('samples_subdir', 'samples')
        self.declare_parameter('crf', 28)  # H.264 quality (lower = better quality)
        self.declare_parameter('ffmpeg_path', shutil.which('ffmpeg') or '/usr/bin/ffmpeg')
        self.declare_parameter('pix_fmt', 'bgr24')  # OpenCV frames are BGR
        self.declare_parameter('preset', 'veryfast')
        self.declare_parameter('publish_json_topic', '/recorder/clip_ready')
        self.declare_parameter('filename_pattern', '{unix_ts}_{label}_v{version}.mp4')
        self.declare_parameter('default_label', 'unknown')

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.target_fps = int(self.get_parameter('target_fps').get_parameter_value().integer_value)
        self.buffer_seconds = float(self.get_parameter('buffer_seconds').get_parameter_value().double_value)
        self.width = int(self.get_parameter('width').get_parameter_value().integer_value)
        self.height = int(self.get_parameter('height').get_parameter_value().integer_value)
        self.data_root = self.get_parameter('data_root').get_parameter_value().string_value
        self.samples_subdir = self.get_parameter('samples_subdir').get_parameter_value().string_value
        self.crf = int(self.get_parameter('crf').get_parameter_value().integer_value)
        self.ffmpeg_path = self.get_parameter('ffmpeg_path').get_parameter_value().string_value
        self.pix_fmt = self.get_parameter('pix_fmt').get_parameter_value().string_value
        self.preset = self.get_parameter('preset').get_parameter_value().string_value
        self.publish_json_topic = self.get_parameter('publish_json_topic').get_parameter_value().string_value
        self.filename_pattern = self.get_parameter('filename_pattern').get_parameter_value().string_value
        self.default_label = self.get_parameter('default_label').get_parameter_value().string_value

        # Ensure directories exist
        self.samples_dir = os.path.join(self.data_root, self.samples_subdir)
        os.makedirs(self.samples_dir, exist_ok=True)

        # Ring buffer sized for buffer_seconds @ target_fps
        self.buffer_len = max(1, int(self.buffer_seconds * self.target_fps))
        self.frame_buffer = deque(maxlen=self.buffer_len)

        # Downsampling timer for target_fps
        self._last_emit_ts = 0.0
        self._emit_period = 1.0 / max(1, self.target_fps)

        # Publisher for clip_ready JSON
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_clip_ready = self.create_publisher(String, self.publish_json_topic, qos_latched)

        # Subscriptions
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub_image = self.create_subscription(Image, self.image_topic, self.on_image, qos_sensor)

        if HAS_AMR_UNKNOWN:
            self.sub_unknown = self.create_subscription(
                UnknownGesture, '/lstm/unknown', self.on_unknown, 10
            )
            self.get_logger().info('Subscribed to /lstm/unknown (amr_interfaces/UnknownGesture).')
        else:
            # Fallback: allow triggering via JSON on /lstm/unknown_json (std_msgs/String)
            self.sub_unknown_json = self.create_subscription(
                String, '/lstm/unknown', self.on_unknown_json, 10
            )
            self.get_logger().warn('amr_interfaces/UnknownGesture not found. Listening to /lstm/unknown as std_msgs/String (JSON).')

        self.get_logger().info(
            f'Recorder ready | topic={self.image_topic} fps={self.target_fps} buffer={self.buffer_seconds}s ' \
            f'({self.buffer_len} frames) size={self.width}x{self.height} → {self.samples_dir}'
        )

    # --- Image handling ---
    def on_image(self, msg: Image):
        now = self.get_clock().now().seconds_nanoseconds()[0] + \
              self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        if now - self._last_emit_ts < self._emit_period:
            return
        self._last_emit_ts = now

        # Convert ROS Image (assume encoding is bgr8 or rgb8)
        if msg.encoding.lower() == 'bgr8':
            frame = self._rosimg_to_bgr(msg)
        elif msg.encoding.lower() == 'rgb8':
            frame = self._rosimg_to_bgr(msg)[:, :, ::-1]
        else:
            # Try to handle other encodings by letting cv_bridge decode when available
            try:
                from cv_bridge import CvBridge
                bridge = CvBridge()
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                frame = cv_img
            except Exception as e:
                self.get_logger().warn(f'Unsupported encoding {msg.encoding}, error: {e}')
                return

        # Resize to target WxH if needed
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)

        # Push into buffer
        self.frame_buffer.append((now, frame))

    @staticmethod
    def _rosimg_to_bgr(msg: Image):
        import numpy as np
        dtype = np.uint8
        step = msg.step
        data = np.frombuffer(msg.data, dtype=dtype)
        img = data.reshape((msg.height, step // 3, 3))[:, :msg.width, :]
        return img

    # --- Unknown gesture handling ---
    def on_unknown(self, msg):
        # amr_interfaces/UnknownGesture expected to have: conf, window_id, stamp
        window_id = getattr(msg, 'window_id', -1)
        label_hint = self.default_label
        self._dump_and_publish(window_id=window_id, label_hint=label_hint)

    def on_unknown_json(self, s: String):
        try:
            payload = json.loads(s.data)
        except Exception:
            payload = {}
        window_id = payload.get('window_id', -1)
        label_hint = payload.get('label_hint', self.default_label)
        self._dump_and_publish(window_id=window_id, label_hint=label_hint)

    # --- Core: dump current buffer to MP4 using ffmpeg ---
    def _dump_and_publish(self, window_id: int, label_hint: str):
        if len(self.frame_buffer) < max(1, int(self.buffer_seconds * self.target_fps * 0.5)):
            self.get_logger().warn('Not enough frames buffered to dump a clip yet.')
            return

        unix_ts = int(time.time())
        version = 1
        fname = self.filename_pattern.format(unix_ts=unix_ts, label=label_hint, version=version)
        out_path = os.path.join(self.samples_dir, fname)

        # Ensure unique filename if exists
        while os.path.exists(out_path):
            version += 1
            fname = self.filename_pattern.format(unix_ts=unix_ts, label=label_hint, version=version)
            out_path = os.path.join(self.samples_dir, fname)

        ok = self._write_mp4_ffmpeg(out_path)
        if not ok:
            self.get_logger().error('Failed to write MP4 via ffmpeg.')
            return

        # Publish JSON payload for downstream consumers (bridge/UI)
        payload = {
            'ts_iso': datetime.utcnow().isoformat() + 'Z',
            'window_id': int(window_id),
            'clip_path': out_path,
            'fps': self.target_fps,
            'width': self.width,
            'height': self.height,
            'label_hint': label_hint,
            'source': 'recorder_node'
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_clip_ready.publish(msg)
        self.get_logger().info(f'Clip ready → {out_path} (window_id={window_id})')

    def _write_mp4_ffmpeg(self, out_path: str) -> bool:
        # Build ffmpeg command to read raw BGR frames from stdin
        cmd = [
            self.ffmpeg_path,
            '-y',
            '-f', 'rawvideo',
            '-pix_fmt', self.pix_fmt,
            '-s', f'{self.width}x{self.height}',
            '-r', str(self.target_fps),
            '-i', '-',  # stdin
            '-an',
            '-c:v', 'libx264',
            '-preset', self.preset,
            '-crf', str(self.crf),
            '-movflags', '+faststart',
            out_path,
        ]

        try:
            proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            self.get_logger().error(f'ffmpeg not found at {self.ffmpeg_path}. Install ffmpeg or set parameter ffmpeg_path.')
            return False
        except Exception as e:
            self.get_logger().error(f'Failed to spawn ffmpeg: {e}')
            return False

        # Stream frames in chronological order
        try:
            for _, frame in list(self.frame_buffer):
                # Ensure contiguous BGR bytes
                proc.stdin.write(frame.tobytes())
        except Exception as e:
            self.get_logger().error(f'Error writing frames to ffmpeg stdin: {e}')
            try:
                proc.stdin.close()
            except Exception:
                pass
            proc.kill()
            return False

        try:
            proc.stdin.close()
        except Exception:
            pass
        ret = proc.wait(timeout=10)
        if ret != 0:
            self.get_logger().error(f'ffmpeg exited with code {ret}')
            return False
        return True


def main():
    rclpy.init()
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
