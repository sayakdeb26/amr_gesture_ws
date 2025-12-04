from typing import Optional, Tuple

import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node

from vlm_interfaces.msg import Intent, UnknownGesture, KeypointsWindow


class LstmOnnxNode(Node):
    def __init__(self) -> None:
        super().__init__("lstm_onnx_node")

        # ---- parameters
        self.declare_parameter("weights_path", "")
        self.declare_parameter("normalizer_path", "")
        self.declare_parameter("labels_path", "")
        self.declare_parameter("conf_threshold", 0.80)
        self.declare_parameter("min_frames", 12)     # allow early decision
        self.declare_parameter("model_frames", 30)   # training window length
        self.declare_parameter("intra_threads", 3)
        self.declare_parameter("use_cuda", True)
        self.declare_parameter("allow_stub", False)
        self.declare_parameter("background_label", "NO_GESTURE")
        self.declare_parameter("treat_background_as_unknown", True)

        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.model_frames = int(self.get_parameter("model_frames").value)
        self.min_frames = int(self.get_parameter("min_frames").value)
        self.allow_stub = bool(self.get_parameter("allow_stub").value)

        self.bg_label = self.get_parameter("background_label").value or "NO_GESTURE"
        self.bg_is_unknown = bool(self.get_parameter("treat_background_as_unknown").value)

        # ---- labels / normalizer
        self.labels = self._load_labels(self.get_parameter("labels_path").value)
        self.mu, self.sigma = self._load_normalizer(self.get_parameter("normalizer_path").value)

        # ---- ONNX session (decide expected layout)
        self.session: Optional[ort.InferenceSession] = None
        self.input_rank: Optional[int] = None   # 2 or 3
        self.frames_needed: int = self.model_frames
        self.feat_dim: Optional[int] = None     # expected F (e.g., 126)

        weights_path = self.get_parameter("weights_path").value
        try:
            if not weights_path:
                raise RuntimeError("weights_path param is empty")

            so = ort.SessionOptions()
            so.intra_op_num_threads = int(self.get_parameter("intra_threads").value)
            use_cuda = bool(self.get_parameter("use_cuda").value)
            providers = ["CUDAExecutionProvider","CPUExecutionProvider"] if use_cuda else ["CPUExecutionProvider"]
            self.session = ort.InferenceSession(weights_path, sess_options=so, providers=providers)
            self.get_logger().info(f"ONNX providers: {self.session.get_providers()}")

            inp = self.session.get_inputs()[0]
            ishape = tuple(int(d) if isinstance(d, (int, np.integer)) else None for d in inp.shape)
            self.input_rank = len(ishape)

            if self.input_rank == 3:
                # [B, T, F]
                T = ishape[1] if ishape[1] is not None else self.model_frames
                F = ishape[2] if ishape[2] is not None else None
                self.frames_needed = int(T)
                self.feat_dim = int(F) if F is not None else None
                self.get_logger().info(f"ONNX loaded. input=x shape=['batch', {T}, {F}]")
            elif self.input_rank == 2:
                # [B, T*F]
                self.frames_needed = self.model_frames
                self.feat_dim = None
                self.get_logger().info("ONNX loaded. input=x shape=['batch', T*F]")
            else:
                raise RuntimeError(f"Unsupported ONNX input rank: {self.input_rank}, shape={ishape}")

        except Exception as e:
            if self.allow_stub:
                self.get_logger().warn(f"ONNX load failed ({e}); using STUB fallback")
                self.session = None
            else:
                raise

        # ---- I/O
        self.sub_kp = self.create_subscription(KeypointsWindow, "/lstm/keypoints_window", self.on_kpwin, 10)
        self.pub_intent_raw = self.create_publisher(Intent, "/intents_raw", 10)
        self.pub_intent = self.create_publisher(Intent, "/intents", 10)
        self.pub_unknown = self.create_publisher(UnknownGesture, "/lstm/unknown", 10)

    # ------------------------ helpers ------------------------

    def _publish_unknown(self, hint: str, conf: float = 0.0, window_frames: int = 0) -> None:
        m = UnknownGesture()
        # Fill only fields that exist in your .msg
        if hasattr(m, "confidence"):
            m.confidence = float(conf)
        if hasattr(m, "window_frames"):
            m.window_frames = int(window_frames)
        if hasattr(m, "source"):
            m.source = "lstm_onnx"
        if hasattr(m, "hint"):
            m.hint = str(hint)
        try:
            if hasattr(m, "stamp"):
                from rclpy.clock import Clock
                m.stamp = Clock().now().to_msg()
        except Exception:
            pass
        self.pub_unknown.publish(m)

    def _set_intent_fields(self, msg: Intent, label: str, confidence: float) -> None:
        # Be defensive about interface fields
        if hasattr(msg, "label"):
            msg.label = label
        if hasattr(msg, "confidence"):
            msg.confidence = float(confidence)
        # Optional fields (populate if present)
        try:
            if hasattr(msg, "stamp"):
                from rclpy.clock import Clock
                msg.stamp = Clock().now().to_msg()
        except Exception:
            pass
        if hasattr(msg, "source"):
            setattr(msg, "source", "lstm_onnx")

    def _load_labels(self, path: str):
        if not path:
            return []
        try:
            with open(path, "r") as f:
                return [ln.strip() for ln in f if ln.strip()]
        except Exception as e:
            self.get_logger().warn(f"Could not load labels from {path}: {e}")
            return []

    def _load_normalizer(self, path: str):
        try:
            import json
            with open(path, "r") as f:
                j = json.load(f)
            mu = np.array(j.get("mu", []), dtype=np.float32) if "mu" in j else None
            sigma = np.array(j.get("sigma", []), dtype=np.float32) if "sigma" in j else None
            return mu, sigma
        except Exception as e:
            self.get_logger().warn(f"No/invalid normalizer at {path}: {e}")
            return None, None

    def _normalize(self, x: np.ndarray) -> np.ndarray:
        # x is either (1, T, F) or (1, T*F) just before flattening
        if self.mu is None or self.sigma is None:
            return x
        mu, sg = self.mu, np.where(self.sigma == 0, 1.0, self.sigma)

        if x.ndim == 3:
            T, F = x.shape[1], x.shape[2]
            if mu.ndim == 1 and mu.size == F:
                return (x - mu[None, None, :]) / sg[None, None, :]
            if mu.ndim == 1 and mu.size == T * F:
                mu2 = mu.reshape(T, F)
                sg2 = sg.reshape(T, F)
                return (x - mu2[None, :, :]) / sg2[None, :, :]
        elif x.ndim == 2:
            N = x.shape[1]
            if mu.ndim == 1 and mu.size == N:
                return (x - mu[None, :]) / sg[None, :]
        return x

    @staticmethod
    def _softmax(z: np.ndarray) -> np.ndarray:
        z = z - np.max(z)
        e = np.exp(z)
        return e / np.sum(e)

    # ------------------------ callback ------------------------

    def on_kpwin(self, msg: KeypointsWindow) -> None:
        try:
            T = int(msg.frames)
            J = int(msg.joints_per_frame)
            arr = np.asarray(msg.data, dtype=np.float32).ravel()

            if T <= 0 or J <= 0:
                self.get_logger().warn(f"Bad KeypointsWindow header: T={T}, J={J}")
                self._publish_unknown("bad_header")
                return

            # Compute coords_per_joint (2: x,y or 3: x,y,z)
            if arr.size % T != 0:
                self.get_logger().warn(
                    f"Bad KeypointsWindow size: {arr.size} not divisible by T={T}"
                )
                self._publish_unknown("bad_size", window_frames=T)
                return

            per_frame_raw = arr.size // T
            if per_frame_raw % J != 0:
                self.get_logger().warn(
                    f"Bad KeypointsWindow size: {arr.size} cannot factor into T={T}, J={J} (per_frame_raw={per_frame_raw})"
                )
                self._publish_unknown("bad_factor", window_frames=T)
                return

            C = per_frame_raw // J
            if C not in (2, 3):
                self.get_logger().warn(f"Unsupported coords per joint: {C} (expected 2 or 3)")
                self._publish_unknown("bad_coords", window_frames=T)
                return

            F = J * C
            x = arr.reshape(1, T, F)  # (1, T, J*C)

            # If model expects 3 coords (F=126) but stream is x,y only (F=84), pad z=0
            if self.input_rank == 3:
                expected_F = self.feat_dim if self.feat_dim is not None else (J * 3)
                if F == J * 2 and expected_F == J * 3:
                    x = x.reshape(1, T, J, 2)
                    z = np.zeros((1, T, J, 1), dtype=np.float32)
                    x = np.concatenate([x, z], axis=-1).reshape(1, T, J * 3)
                    F = J * 3
                elif F != expected_F and expected_F is not None:
                    self.get_logger().warn(
                        f"Feature mismatch: got F={F}, expected F={expected_F} (T={T}, J={J})"
                    )
                    self._publish_unknown("feat_mismatch", window_frames=T)
                    return

            # Conform T to model expectation (left-pad with zeros or trim on the left)
            if self.frames_needed and T != self.frames_needed:
                if T < self.frames_needed:
                    pad = np.zeros((1, self.frames_needed - T, F), dtype=np.float32)
                    x = np.concatenate([pad, x], axis=1)
                    T = self.frames_needed
                else:
                    x = x[:, -self.frames_needed:, :]
                    T = self.frames_needed

            # Too few frames for a decision?
            if T < self.min_frames:
                self._publish_unknown("too_few_frames", window_frames=T)
                return

            # Normalize (handles (1,T,F) or flattened)
            x = self._normalize(x)

            if self.session is None:
                self._publish_unknown("onnx_stub", window_frames=T)
                return

            # Flatten if model wants [B, T*F]
            if self.input_rank == 2:
                x = x.reshape(1, T * x.shape[2])

#----------------
            out = self.session.run(None, {self.session.get_inputs()[0].name: x})[0]  # (1, C)
            probs = self._softmax(out[0])
            idx = int(np.argmax(probs))
            conf = float(probs[idx])
            label = self.labels[idx] if 0 <= idx < len(self.labels) else f"class_{idx}"

            # --- routing logic ---
            # 1) Pure background → NO_GESTURE → drop completely
            if label == "NO_GESTURE":
                # optional debug:
                # self.get_logger().debug(f"Dropping NO_GESTURE (conf={conf:.3f})")
                return

            # 2) Explicit "IGNORE" class → send ONLY to VLM via /lstm/unknown
            if label == "IGNORE":
                self._publish_unknown("ignore", conf=conf, window_frames=T)
                return

            # 3) Other labels:
            #    - if confidence too low → drop (do NOT send to VLM)
            #    - if high enough → publish /intents_raw + /intents
            if conf < self.conf_threshold:
                # optional debug:
                # self.get_logger().debug(f"Low-conf {label} (conf={conf:.3f}), dropping.")
                return

            msg_out = Intent()
            self._set_intent_fields(msg_out, label, conf)
            self.pub_intent_raw.publish(msg_out)
            self.pub_intent.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f"Error in on_kpwin: {e}")

def main() -> None:
    rclpy.init()
    node = LstmOnnxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
