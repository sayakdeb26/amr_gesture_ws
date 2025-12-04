#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from amr_interfaces.msg import UnknownGesture, Intent, ConfirmReply # Added Intent and ConfirmReply
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime as ort
import mediapipe as mp
from mediapipe.tasks.python import vision
from mediapipe.tasks.python import BaseOptions
import collections
import os
import time
import json

def softmax(x):
    """Compute softmax values for each sets of scores in x."""
    e_x = np.exp(x - np.max(x))
    return e_x / e_x.sum(axis=0)

class LSTMNode(Node):
    def __init__(self):
        super().__init__('lstm_node')
        
        self.declare_parameter('min_frames', 30)
        self.declare_parameter('conf_threshold', 0.80)
        self.declare_parameter('model_path', '')
        self.declare_parameter('mirror', True) # Flip input horizontally
        
        self.min_frames = self.get_parameter('min_frames').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.model_path = self.get_parameter('model_path').value
        self.mirror = self.get_parameter('mirror').value

        # State
        self.bridge = CvBridge()
        self.feature_buffer = collections.deque(maxlen=self.min_frames)
        self.keypoint_buffer_debug = collections.deque(maxlen=self.min_frames) # For visualization
        self.window_counter = 0
        self.session_counter = 0
        
        # Activation State
        self.is_active = False # Start in IDLE mode
        self.last_pred_label = "IDLE"
        self.last_pred_conf = 0.0
        self.fps = 0.0
        self.last_time = time.time()
        self.frame_count = 0
        
        # Cooldown State
        self.paused = False
        self.pause_until_time = 0.0
        self.waiting_for_confirmation = False
        self.current_unknown_session = ""
        self.cooldown_known = 2.5  # seconds after known gesture
        self.cooldown_unknown = 90.0  # timeout for unknown (fallback)

        # Load ONNX Model
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model not found: {self.model_path}")
            raise FileNotFoundError(self.model_path)
            
        try:
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            self.ort_session = ort.InferenceSession(self.model_path, providers=providers)
            self.input_name = self.ort_session.get_inputs()[0].name
            self.get_logger().info(f"Loaded LSTM model from {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX model: {e}")
            raise e

        # Labels (12 classes)
        self.model_labels = [
            "IGNORE", "NO_GESTURE", "STOP_SIGN", "PUSHING_HAND_AWAY", 
            "SWIPE_DOWN", "SWIPE_LEFT", "SWIPE_RIGHT", "SWIPE_UP", 
            "THUMB_DOWN", "THUMB_UP", "ZOOM_IN", "ZOOM_OUT"
        ]
        
        # Mapping to system labels
        self.label_map = {
            "SWIPE_LEFT": "SWIPE_LEFT",
            "SWIPE_RIGHT": "SWIPE_RIGHT",
            "SWIPE_UP": "SWIPE_UP",
            "SWIPE_DOWN": "SWIPE_DOWN",
            "STOP_SIGN": "STOP",
            "THUMB_UP": "THUMB_UP",
            "THUMB_DOWN": "THUMB_DOWN",
            "ZOOM_IN": "ZOOM_IN",
            "ZOOM_OUT": "ZOOM_OUT",
            "PUSHING_HAND_AWAY": "ZOOM_OUT", # Alias
            "NO_GESTURE": "NO_GESTURE"    # Explicitly mapped (don't trigger VLM)
            # "IGNORE" is NOT mapped -> becomes "UNKNOWN" -> triggers VLM
        }

        # Initialize MediaPipe
        base_options = BaseOptions(model_asset_path='/home/sayak/amr_gesture_ws/models/mediapipe/hand_landmarker.task')
        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=2,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.landmarker = vision.HandLandmarker.create_from_options(options)
        self.get_logger().info("MediaPipe HandLandmarker initialized.")

        # Subs/Pubs
        self.sub_img = self.create_subscription(Image, '/image_raw', self.frame_callback, 10)
        self.sub_reply = self.create_subscription(ConfirmReply, '/ui/confirm_reply', self.on_confirm_reply, 10)
        self.pub_intent = self.create_publisher(Intent, '/intents_raw', 10) # Changed to Intent
        self.pub_unknown = self.create_publisher(UnknownGesture, '/lstm/unknown', 10)
        self.pub_debug = self.create_publisher(Image, '/lstm/debug_feed', 10)

    def extract_features(self, img_rgb):
        # MediaPipe inference
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=img_rgb)
        result = self.landmarker.detect(mp_img)
        
        # Extract 84 features (2 hands * 21 landmarks * 2 coords)
        # If < 2 hands, pad with 0
        feats = []
        
        # For visualization, store the raw landmarks
        raw_landmarks = []
        if result.hand_landmarks:
            raw_landmarks = result.hand_landmarks

        # Logic to flatten landmarks to 84-dim vector
        # We need to be consistent with training data.
        # Assuming training data is: Hand1 (x,y,x,y...) then Hand2 (x,y...)
        # Normalized by wrist? The user snippet does normalization.
        # Let's try to match the user snippet's normalization logic.
        
        # User snippet logic:
        # 1. Collect all landmarks (x,y)
        # 2. If 1 hand, pad with 0s to 42 floats.
        # 3. Normalize relative to wrist (landmark 0).
        
        # Collect raw points first
        all_points = []
        if result.hand_landmarks:
            for hand_lms in result.hand_landmarks:
                for lm in hand_lms:
                    all_points.extend([lm.x, lm.y])
        
        # Pad to 84 (2 hands * 21 * 2)
        # Wait, user snippet says: "If only one hand, pad second hand with zeros".
        # And "keypoints.extend([0.0] * 42)" if len(result.hand_landmarks) == 1.
        # What if 0 hands?
        if not all_points:
            return np.zeros(84, dtype=np.float32), raw_landmarks
            
        if len(all_points) < 84:
            all_points.extend([0.0] * (84 - len(all_points)))
        
        # Truncate if > 84 (more than 2 hands?)
        all_points = all_points[:84]
        
        # Normalize
        # User snippet:
        # wrist_x = frame[0], wrist_y = frame[1]
        # for j in range(0, 84, 2): if frame[j]!=0... normalized = frame - wrist
        
        feats = np.array(all_points, dtype=np.float32)
        wrist_x = feats[0]
        wrist_y = feats[1]
        
        for j in range(0, 84, 2):
            # Check if it's a valid point (not padding)
            # Simple check: if it's 0.0 and 0.0 it might be padding (or actually at 0,0)
            # But let's just apply normalization to everything that isn't the padded zeros.
            # Actually, if we pad with 0, subtracting wrist will make them non-zero (-wrist).
            # We should only normalize the detected points.
            if feats[j] != 0 or feats[j+1] != 0:
                feats[j] -= wrist_x
                feats[j+1] -= wrist_y
                
        return feats, raw_landmarks

    def frame_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.mirror:
                cv_img = cv2.flip(cv_img, 1)
            
            rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            
            # Extract features
            feats, raw_landmarks = self.extract_features(rgb_img)
            self.feature_buffer.append(feats)
            self.keypoint_buffer_debug.append(raw_landmarks) # Store for drawing if needed, but we draw current frame
            
            # FPS calc
            self.frame_count += 1
            now = time.time()
            if now - self.last_time >= 1.0:
                self.fps = self.frame_count / (now - self.last_time)
                self.frame_count = 0
                self.last_time = now

            # Prediction
            label = "BUFFERING"
            conf = 0.0
            
            if len(self.feature_buffer) == self.min_frames:
                label, conf = self.process_window()
                self.last_pred_label = label
                self.last_pred_conf = conf
            
            # Draw HUD
            annotated_img = self.draw_interface(cv_img, raw_landmarks, self.last_pred_label, self.last_pred_conf, self.fps, self.is_active)
            
            # Publish debug feed
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(annotated_img, "bgr8"))
            
        except Exception as e:
            self.get_logger().error(f"Frame processing error: {e}")

    def on_confirm_reply(self, msg: ConfirmReply):
        # Resume when operator confirms/rejects unknown gesture
        if self.waiting_for_confirmation and msg.session_id == self.current_unknown_session:
            self.waiting_for_confirmation = False
            self.paused = False
            self.get_logger().info(f"Resuming gesture recognition after confirmation")

    def process_window(self):
        # ALWAYS run inference first (to detect ZOOM gestures)
        input_data = np.array(self.feature_buffer, dtype=np.float32).reshape(1, self.min_frames, 84)
        
        try:
            outputs = self.ort_session.run(None, {self.input_name: input_data})
            logits = outputs[0][0] # (12,)
            probs = softmax(logits)
            
            pred_idx = np.argmax(probs)
            confidence = float(probs[pred_idx])
            model_label = self.model_labels[pred_idx]
            
            # Map to system label
            label = self.label_map.get(model_label, "UNKNOWN")
            
            # PRIORITY: Check for ZOOM_OUT/ZOOM_IN first (bypass cooldown)
            # These are control gestures and should ALWAYS work
            if label == "ZOOM_OUT" and confidence > self.conf_threshold:
                if not self.is_active:
                    self.is_active = True
                    self.get_logger().info("ACTIVATED Gesture Control (ZOOM_OUT detected)")
                    self.publish_intent(label, confidence)
                    # Clear any existing pause
                    self.paused = False
                    self.waiting_for_confirmation = False
                return label, confidence

            if label == "ZOOM_IN" and confidence > self.conf_threshold:
                if self.is_active:
                    # Don't interrupt VLM processing
                    if self.waiting_for_confirmation:
                        return "BLOCKED_VLM", 0.0
                    self.is_active = False
                    self.publish_intent(label, confidence)
                    # Clear any existing pause
                    self.paused = False
                    self.waiting_for_confirmation = False
                return label, confidence
            
            # NOW check if paused (for other gestures)
            now = time.time()
            if self.paused and now < self.pause_until_time:
                return "PAUSED", 0.0
            elif self.paused and now >= self.pause_until_time:
                # Timeout reached, auto-resume
                self.paused = False
                self.get_logger().info("Auto-resuming after cooldown")
            
            if self.waiting_for_confirmation:
            # Wait indefinitely for VLM + operator confirmation
                    return "WAITING_CONFIRM", 0.0
                    return "WAITING_CONFIRM", 0.0

            # If not active, ignore everything else
            if not self.is_active:
                return "IDLE", 0.0

            # If active, process gestures
            if confidence > self.conf_threshold:
                if label != "UNKNOWN":
                    self.get_logger().info(f"Detected: {label} ({confidence:.2f})")
                    self.publish_intent(label, confidence)
                    # Set cooldown for known gestures
                    self.paused = True
                    self.pause_until_time = time.time() + self.cooldown_known
                    self.get_logger().info(f"Pausing for {self.cooldown_known}s (gesture execution)")
                else:
                    # Unknown gesture handling
                    # Trigger for UNKNOWN (which includes IGNORE from model)
                    # But NOT for NO_GESTURE (no hand detected)
                    if model_label != "NO_GESTURE":
                        self.get_logger().info(f"Unknown gesture: {model_label} ({confidence:.2f})")
                        session_id = self.publish_unknown(model_label, confidence)
                        # Pause until confirmation received
                        self.waiting_for_confirmation = True
                        self.current_unknown_session = session_id
                        self.pause_until_time = time.time() + self.cooldown_unknown
                        self.get_logger().info(f"Pausing for VLM confirmation (session: {session_id})")
            
            return label, confidence
            
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            return "ERROR", 0.0

    def publish_intent(self, label, conf):
        msg = Intent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.session_id = f"lstm_{int(time.time())}" # Simple session ID for LSTM events
        msg.label = label
        msg.confidence = float(conf)
        msg.source = "lstm"
        self.pub_intent.publish(msg)

    def publish_unknown(self, label, conf):
        # Trigger VLM pipeline
        self.session_counter += 1
        session_id = f"sess_{int(time.time())}_{self.session_counter}"
        
        msg = UnknownGesture()
        msg.stamp = self.get_clock().now().to_msg()
        msg.session_id = session_id
        msg.window_id = self.window_counter
        msg.label = label
        msg.confidence = float(conf)
        msg.hint = ""
        msg.source = "lstm"
        
        self.pub_unknown.publish(msg)
        return session_id  # Return for tracking

    # --- Visualization Methods ---

    def draw_interface(self, image, landmarks_list, gesture_label, confidence, fps, is_active):
        h, w = image.shape[:2]
        overlay = image.copy()
        
        # Top bar background
        cv2.rectangle(overlay, (0, 0), (w, 140), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.8, image, 0.2, 0, image)

        # Draw landmarks
        if landmarks_list:
            self._draw_hand_landmarks(image, landmarks_list)

        # Status Text
        if is_active:
            if confidence > self.conf_threshold and gesture_label not in ["IDLE", "BUFFERING", "UNKNOWN"]:
                color = (0, 255, 0) # Green
                status = "GESTURE DETECTED"
            else:
                color = (0, 200, 255) # Yellow/Orange
                status = "ACTIVE - LISTENING"
        else:
            color = (100, 100, 255) # Red/Blueish
            status = "IDLE (ZOOM_OUT to Activate)"

        cv2.putText(image, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(image, f"{gesture_label}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(image, f"Confidence: {confidence:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.putText(image, f"FPS: {fps:.1f}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        return image

    def _draw_hand_landmarks(self, image, landmarks_list):
        h, w = image.shape[:2]
        
        # landmarks_list is a list of NormalizedLandmarkList (one per hand)
        # We need to iterate over hands
        for hand_landmarks in landmarks_list:
            points = []
            for lm in hand_landmarks:
                x = int(lm.x * w)
                y = int(lm.y * h)
                points.append((x, y))
                cv2.circle(image, (x, y), 3, (0, 255, 0), -1)

            if len(points) >= 21:
                connections = [
                    (0, 1), (1, 2), (2, 3), (3, 4),
                    (0, 5), (5, 6), (6, 7), (7, 8),
                    (0, 9), (9, 10), (10, 11), (11, 12),
                    (0, 13), (13, 14), (14, 15), (15, 16),
                    (0, 17), (17, 18), (18, 19), (19, 20),
                    (5, 9), (9, 13), (13, 17),
                ]
                for start, end in connections:
                    cv2.line(image, points[start], points[end], (0, 200, 200), 2)

def main():
    rclpy.init()
    node = LSTMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
