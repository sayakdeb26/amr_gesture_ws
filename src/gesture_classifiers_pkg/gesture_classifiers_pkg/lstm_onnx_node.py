#!/usr/bin/env python3
import math
import time
import numpy as np
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time

from amr_interfaces.msg import KeypointsWindow, Intent, UnknownGesture


def now_stamp(node: Node) -> Time:
    return node.get_clock().now().to_msg()


class Normalizer:
    """Simple normalizer that loads mean/std or applies identity."""
    def __init__(self, mean=None, std=None):
        self.mean = np.array(mean, dtype=np.float32) if mean is not None else None
        self.std = np.array(std, dtype=np.float32) if std is not None else None

    def apply(self, arr: np.ndarray) -> np.ndarray:
        if self.mean is None or self.std is None:
            return arr
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        if self.mean.ndim == 0:
            return (arr - float(self.mean)) / (float(self.std) + 1e-6)
        # per-joint normalization
        return (arr - self.mean) / (self.std + 1e-6)


class LSTMOnnxNode(Node):
    def _prep_window(self, msg: KeypointsWindow):
        """
        Returns (X, F, J_eff)
          X: np.ndarray shape [F, J_eff]
          F: frames
          J_eff: features per frame (after any XYZ→XY flatten)
        Accepts:
          - data len == F * J_meta          (already 2D features per frame)
          - data len == F * J_meta * 3      (XYZ → keep XY only)
          - or infers J if needed
        """
        import numpy as np

        F = int(msg.frames)
        J_meta = int(msg.joints_per_frame)
        data = np.array(msg.data, dtype=np.float32)
        N = int(data.size)

        # Case A: matches declared J -> [F, J_meta]
        if F > 0 and N == F * J_meta:
            X = data.reshape(F, J_meta)
            return X, F, J_meta

        # Case B: looks like XYZ for J_meta joints -> [F, J_meta, 3] -> drop Z -> [F, J_meta*2]
        if F > 0 and N == F * J_meta * 3:
            X3 = data.reshape(F, J_meta, 3)
            X2 = X3[:, :, :2].reshape(F, J_meta * 2)  # keep XY
            return X2, F, J_meta * 2

        # Case C: infer J from data/F and handle XYZ→XY if divisible by 3
        if F > 0 and N % F == 0:
            J_infer = N // F
            if J_infer % 3 == 0:
                Jj = J_infer // 3
                X3 = data.reshape(F, Jj, 3)
                X2 = X3[:, :, :2].reshape(F, Jj * 2)
                return X2, F, Jj * 2
            else:
                X = data.reshape(F, J_infer)
                return X, F, J_infer

        raise ValueError(f"data len {N} not compatible with frames={F}, J_meta={J_meta}")
    def __init__(self):
        super().__init__("lstm_onnx_node")

        self.declare_parameter("weights_path", "")
        self.declare_parameter("normalizer_path", "")
        self.declare_parameter("labels_path", "")
        self.declare_parameter("conf_threshold", 0.80)
        self.declare_parameter("min_frames", 30)
        self.declare_parameter("model_frames", 30)
        self.declare_parameter("intra_threads", 3)
        self.declare_parameter("allow_stub", True)

        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.min_frames = int(self.get_parameter("min_frames").value)
        self.model_frames = int(self.get_parameter("model_frames").value)
        self.allow_stub = bool(self.get_parameter("allow_stub").value)

        self.pub_intent = self.create_publisher(Intent, "/intents_raw", 10)
        self.pub_unknown = self.create_publisher(UnknownGesture, "/lstm/unknown", 10)
        self.sub_kp = self.create_subscription(
            KeypointsWindow, "/lstm/keypoints_window", self.on_keypoints, 10
        )

        self.labels = ["UNKNOWN"]
        self.model = type("Model", (), {})()
        self.model.sess = None
        self.model.input_name = "input"
        self.model.output_name = "logits"

        self._load_labels()
        self.norm = self._load_normalizer()
        self._load_onnx()

        self.get_logger().info(
            f"lstm_onnx_node up. mode={'onnx' if self.model.sess else 'stub'}, "
            f"conf_threshold={self.conf_threshold:.2f}, "
            f"min_frames={self.min_frames}, model_frames={self.model_frames}, "
            f"labels={len(self.labels)}"
        )

    def _load_labels(self):
        path = self.get_parameter("labels_path").value
        try:
            with open(path) as f:
                self.labels = [line.strip() for line in f if line.strip()]
        except Exception:
            self.get_logger().warn(f"Labels file not found at {path}; using ['UNKNOWN'].")

    def _load_normalizer(self):
        path = self.get_parameter("normalizer_path").value
        try:
            import json
            with open(path) as f:
                norm_data = json.load(f)
            mean = norm_data.get("mean", 0.0)
            std = norm_data.get("std", 1.0)
            return Normalizer(mean=mean, std=std)
        except Exception:
            self.get_logger().warn(f"Normalizer not found at {path}; using identity.")
            return Normalizer()

    def _load_onnx(self):
        path = self.get_parameter("weights_path").value
        try:
            sess_opts = ort.SessionOptions()
            sess_opts.intra_op_num_threads = int(
                self.get_parameter("intra_threads").value
            )
            self.model.sess = ort.InferenceSession(path, sess_options=sess_opts)
            self.model.input_name = self.model.sess.get_inputs()[0].name
            self.model.output_name = self.model.sess.get_outputs()[0].name
            self.get_logger().info(
                f"ONNX loaded. input={self.model.input_name} shape={self.model.sess.get_inputs()[0].shape}, "
                f"output={self.model.output_name} shape={self.model.sess.get_outputs()[0].shape}"
            )
        except Exception as e:
            self.get_logger().warn(f"ONNX model not found at {path}, running in stub mode. ({e})")
            self.model.sess = None
    def _prep_window(self, msg: KeypointsWindow):
        """
        Returns (X, F, J_eff)
          X: np.ndarray shape [F, J_eff]
          F: frames
          J_eff: features per frame (after any XYZ→XY flatten)
        Accepts:
          - data len == F * J_meta          (already 2D features per frame)
          - data len == F * J_meta * 3      (XYZ → keep XY only)
          - or infers J if needed
        """
        import numpy as np

        F = int(msg.frames)
        J_meta = int(msg.joints_per_frame)
        data = np.array(msg.data, dtype=np.float32)
        N = int(data.size)

        # Case A: matches declared J -> [F, J_meta]
        if F > 0 and N == F * J_meta:
            X = data.reshape(F, J_meta)
            return X, F, J_meta

        # Case B: looks like XYZ for J_meta joints -> [F, J_meta, 3] -> drop Z -> [F, J_meta*2]
        if F > 0 and N == F * J_meta * 3:
            X3 = data.reshape(F, J_meta, 3)
            X2 = X3[:, :, :2].reshape(F, J_meta * 2)  # keep XY
            return X2, F, J_meta * 2

        # Case C: infer J from data/F and handle XYZ→XY if divisible by 3
        if F > 0 and N % F == 0:
            J_infer = N // F
            if J_infer % 3 == 0:
                Jj = J_infer // 3
                X3 = data.reshape(F, Jj, 3)
                X2 = X3[:, :, :2].reshape(F, Jj * 2)
                return X2, F, Jj * 2
            else:
                X = data.reshape(F, J_infer)
                return X, F, J_infer

        raise ValueError(f"data len {N} not compatible with frames={F}, J_meta={J_meta}")

    def _publish_intent(self, label, conf, latency_ms):
        it = Intent()
        it.stamp = now_stamp(self)
        it.label = label
        it.confidence = float(conf)
        it.latency_ms = int(latency_ms)
        it.source = "lstm_onnx"
        self.pub_intent.publish(it)

    def _publish_unknown(self, hint):
        unk = UnknownGesture()
        unk.stamp = now_stamp(self)
        unk.confidence = 0.25
        unk.window_frames = self.min_frames
        unk.source = "lstm_onnx"
        unk.hint = hint
        self.pub_unknown.publish(unk)

    def _heuristic_conf(self, flat, F, J):
        arr = np.array(flat, dtype=np.float32).reshape(F, J)
        mean = arr.mean(axis=0)
        var = ((arr - mean) ** 2).mean()
        return 1.0 / (1.0 + math.exp(-10.0 * (var - 0.02)))

    def on_keypoints(self, msg: KeypointsWindow):
        # Guard on frame count
        if msg.frames < self.min_frames:
            self.get_logger().debug(
                f"frames={msg.frames} < min_frames={self.min_frames} → unknown"
            )
            self._publish_unknown(hint="low_frames")
            return

        try:
            X, F, J = self._prep_window(msg)
        except Exception as e:
            self.get_logger().warn(f"_prep_window failed: {e}; → unknown")
            self._publish_unknown(hint="prep_error")
            return

        if X.ndim != 2:
            try:
                X = X.reshape(-1, J)
                F = X.shape[0]
            except Exception as e:
                self.get_logger().warn(f"reshape failed: {e}; → unknown")
                self._publish_unknown(hint="shape_error")
                return

        try:
            Xn = self.norm.apply(X)
        except Exception as e:
            self.get_logger().warn(f"normalizer.apply failed: {e}; → unknown")
            self._publish_unknown(hint="norm_error")
            return

        if self.model.sess is None:
            if self.allow_stub:
                conf = self._heuristic_conf(
                    Xn.flatten().tolist(), F=min(F, self.model_frames), J=J
                )
                if conf >= self.conf_threshold:
                    self._publish_intent("WAVE_STOP", conf, 0)
                    self.get_logger().info(
                        f"[STUB] High-conf: WAVE_STOP ({conf:.2f}) J={J} F={F}"
                    )
                else:
                    self.get_logger().info(
                        f"[STUB] Low-conf ({conf:.2f} < {self.conf_threshold:.2f}); → VLM"
                    )
                    self._publish_unknown(hint="wave")
            else:
                self._publish_unknown(hint="no_model")
            return

        try:
            t0 = time.time()
            inp = Xn.reshape(1, -1).astype("float32")  # [1, T*J]
            feeds = {self.model.input_name: inp}
            logits = self.model.sess.run([self.model.output_name], feeds)[0]
            lat_ms = int((time.time() - t0) * 1000.0)

            e = np.exp(logits - logits.max(axis=1, keepdims=True))
            probs = e / e.sum(axis=1, keepdims=True)
            p = float(probs[0].max())
            k = int(probs[0].argmax())
            label = self.labels[k] if 0 <= k < len(self.labels) else "UNKNOWN"

            if p >= self.conf_threshold and label != "UNKNOWN":
                self._publish_intent(label, p, lat_ms)
                self.get_logger().info(
                    f"ONNX intent: {label} ({p:.2f}) J={J} F={F} lat={lat_ms}ms"
                )
            else:
                self.get_logger().info(
                    f"ONNX low-conf ({p:.2f} < {self.conf_threshold:.2f}); → VLM"
                )
                self._publish_unknown(hint="wave")
        except Exception as e:
            self.get_logger().error(f"ONNX inference failed: {e}")
            self._publish_unknown(hint="onnx_error")


def main():
    rclpy.init()
    node = LSTMOnnxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

