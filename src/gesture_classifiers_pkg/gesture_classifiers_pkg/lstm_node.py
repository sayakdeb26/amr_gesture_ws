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
        self.declare_parameter('model_path', '/home/sayak/amr_gesture_ws/models/lstm/jester20b_12cls/final_jester_model.onnx')
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
        self.hand_moving = False  # Track if hand is in motion
        
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
            "NO_GESTURE": "NO_GESTURE",    # Explicitly mapped (don't trigger VLM)
            "IGNORE": "IGNORE"             # Map to itself (don't trigger VLM)
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

        # Initialize MediaPipe Face Detection (lightweight filter for false hand detections)
        self.face_detection = mp.solutions.face_detection.FaceDetection(
            model_selection=0,  # 0 = short-range (< 2m), faster
            min_detection_confidence=0.5
        )
        self.rejected_hand_count = 0
        self.get_logger().info("Face detection filter enabled.")

        # Subs/Pubs
        self.sub_img = self.create_subscription(Image, '/image_raw', self.frame_callback, 10)
        self.sub_reply = self.create_subscription(ConfirmReply, '/ui/confirm_reply', self.on_confirm_reply, 10)
        self.pub_intent = self.create_publisher(Intent, '/intents_raw', 10) # Changed to Intent
        self.pub_unknown = self.create_publisher(UnknownGesture, '/lstm/unknown', 10)
        self.pub_debug = self.create_publisher(Image, '/lstm/debug_feed', 10)

    def _get_face_bbox(self, rgb_img):
        """Detect primary face and return its bounding box in normalized coords."""
        results = self.face_detection.process(rgb_img)
        if results.detections and len(results.detections) > 0:
            det = results.detections[0]
            bbox = det.location_data.relative_bounding_box
            return (bbox.xmin, bbox.ymin, bbox.xmin + bbox.width, bbox.ymin + bbox.height)
        return None

    def _get_hand_bbox(self, landmarks):
        """Calculate bounding box from hand landmarks in normalized coords."""
        xs = [lm.x for lm in landmarks]
        ys = [lm.y for lm in landmarks]
        return (min(xs), min(ys), max(xs), max(ys))

    def _bbox_overlap(self, b1, b2):
        """Calculate overlap percentage of b1 with b2."""
        ix1 = max(b1[0], b2[0])
        iy1 = max(b1[1], b2[1])
        ix2 = min(b1[2], b2[2])
        iy2 = min(b1[3], b2[3])
        inter_area = max(0, ix2 - ix1) * max(0, iy2 - iy1)
        b1_area = (b1[2] - b1[0]) * (b1[3] - b1[1])
        return inter_area / b1_area if b1_area > 0 else 0

    def _is_invalid_hand(self, hand_bbox, face_bbox, frame_width, frame_height):
        """
        Check if a hand detection is invalid based on:
        1. Face overlap > 20%
        2. Too small (width or height < 40 pixels)
        3. Aspect ratio < 0.2 (too compressed)
        4. Upper-center region (y < 25% AND x between 30-70%)
        """
        x_min, y_min, x_max, y_max = hand_bbox
        w_px = (x_max - x_min) * frame_width
        h_px = (y_max - y_min) * frame_height
        
        # Rule 1: Too small
        if w_px < 40 or h_px < 40:
            return True, "too_small"
        
        # Rule 2: Aspect ratio too compressed
        aspect = min(w_px, h_px) / max(w_px, h_px) if max(w_px, h_px) > 0 else 0
        if aspect < 0.2:
            return True, "bad_aspect"
        
        # Rule 3: Upper-center region (likely face)
        center_x = (x_min + x_max) / 2
        center_y = (y_min + y_max) / 2
        if center_y < 0.25 and 0.30 < center_x < 0.70:
            return True, "upper_center"
        
        # Rule 4: Face overlap > 20%
        if face_bbox is not None:
            overlap = self._bbox_overlap(hand_bbox, face_bbox)
            if overlap > 0.20:
                return True, "face_overlap"
        
        return False, ""

    def extract_features(self, img_rgb, frame_width, frame_height):
        # MediaPipe inference
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=img_rgb)
        result = self.landmarker.detect(mp_img)
        
        # Detect face for filtering (lightweight, ~0.5ms)
        face_bbox = self._get_face_bbox(img_rgb)
        
        # Filter out invalid hand detections
        valid_hands = []
        raw_landmarks = []
        
        if result.hand_landmarks:
            for hand_lms in result.hand_landmarks:
                hand_bbox = self._get_hand_bbox(hand_lms)
                is_invalid, reason = self._is_invalid_hand(hand_bbox, face_bbox, frame_width, frame_height)
                
                if is_invalid:
                    self.rejected_hand_count += 1
                    if self.rejected_hand_count % 100 == 1:
                        self.get_logger().warn(f"Rejected false hand detection ({reason}) - total: {self.rejected_hand_count}")
                else:
                    valid_hands.append(hand_lms)
            
            raw_landmarks = valid_hands

        # Collect raw points from valid hands only
        all_points = []
        if valid_hands:
            for hand_lms in valid_hands:
                for lm in hand_lms:
                    all_points.extend([lm.x, lm.y])
        
        # Pad to 84 (2 hands * 21 * 2)
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
            
            # Get frame dimensions for face detection filter
            H, W = cv_img.shape[:2]
            
            # Extract features (with face detection filtering)
            feats, raw_landmarks = self.extract_features(rgb_img, W, H)
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
            annotated_img = self.draw_interface(cv_img, raw_landmarks, self.last_pred_label, self.last_pred_conf, self.fps, self.is_active, self.hand_moving)
            
            # Show debug window (OpenCV)
            cv2.imshow("LSTM Debug Feed", annotated_img)
            key = cv2.waitKey(1) & 0xFF
            
            # Handle keyboard input
            if key == ord('q'):
                self.get_logger().info("Quit requested - shutting down")
                rclpy.shutdown()
            elif key == ord('r'):
                self.feature_buffer.clear()
                self.keypoint_buffer_debug.clear()
                self.prediction_history.clear()
                self.get_logger().info("Buffers reset")
            
            # Publish debug feed (for ROS visualization)
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
            
            # PRIORITY: Check for THUMB_UP/THUMB_DOWN first (bypass cooldown)
            # These are control gestures and should ALWAYS work
            if label == "THUMB_UP" and confidence > self.conf_threshold:
                if not self.is_active:
                    self.is_active = True
                    self.get_logger().info("ACTIVATED Gesture Control (THUMB_UP detected)")
                    self.publish_intent(label, confidence)
                    # Clear any existing pause
                    self.paused = False
                    self.waiting_for_confirmation = False
                return label, confidence

            if label == "THUMB_DOWN" and confidence > self.conf_threshold:
                if self.is_active:
                    # Don't interrupt VLM processing
                    if self.waiting_for_confirmation:
                        return "BLOCKED_VLM", 0.0
                    self.is_active = False
                    self.get_logger().info("DEACTIVATED Gesture Control (THUMB_DOWN detected)")
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
                # Movement gestures that trigger cooldown
                movement_gestures = ["SWIPE_LEFT", "SWIPE_RIGHT", "SWIPE_UP", "SWIPE_DOWN", "STOP"]
                
                if label in movement_gestures:
                    # Only movement gestures trigger cooldown
                    self.get_logger().info(f"Detected movement: {label} ({confidence:.2f})")
                    self.publish_intent(label, confidence)
                    self.paused = True
                    self.pause_until_time = time.time() + self.cooldown_known
                    self.get_logger().info(f"Pausing for {self.cooldown_known}s (gesture execution)")
                elif label not in ["NO_GESTURE", "IGNORE", "THUMB_UP", "THUMB_DOWN", "UNKNOWN"]:
                    # Other known gestures (like ZOOM_IN, ZOOM_OUT) - publish but no cooldown
                    self.get_logger().info(f"Detected: {label} ({confidence:.2f})")
                    self.publish_intent(label, confidence)
                elif label == "UNKNOWN":
                    # Unknown gesture handling - trigger VLM
                    if model_label not in ["NO_GESTURE", "IGNORE"]:
                        self.get_logger().info(f"Unknown gesture: {model_label} ({confidence:.2f})")
                        session_id = self.publish_unknown(model_label, confidence)
                        self.waiting_for_confirmation = True
                        self.current_unknown_session = session_id
                        self.pause_until_time = time.time() + self.cooldown_unknown
                        self.get_logger().info(f"Pausing for VLM confirmation (session: {session_id})")
                # NO_GESTURE, IGNORE, THUMB_UP, THUMB_DOWN just pass through without cooldown
            
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

    def draw_interface(self, image, landmarks_list, gesture_label, confidence, fps, is_active, hand_moving):
        h, w = image.shape[:2]
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (w, 140), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.8, image, 0.2, 0, image)

        # Draw hand landmarks with connections
        if landmarks_list:
            self._draw_hand_landmarks(image, landmarks_list)

        # Status Text based on state
        if is_active:
            if confidence > self.conf_threshold and gesture_label not in ["IDLE", "BUFFERING", "UNKNOWN", "NO_GESTURE"]:
                color = (0, 255, 0)  # Green
                status = "GESTURE DETECTED"
            elif hand_moving:
                color = (0, 200, 255)  # Yellow/Orange
                status = "HAND MOVING"
            else:
                color = (0, 200, 255)  # Yellow/Orange
                status = "ACTIVE - LISTENING"
        else:
            color = (100, 100, 255)  # Red/Blueish
            status = "IDLE (THUMB_UP to Activate)"

        cv2.putText(image, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(image, f"{gesture_label}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(image, f"Confidence: {confidence:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Stats line
        cv2.putText(image, f"FPS: {fps:.1f}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(image, f"Frames: {len(self.feature_buffer)}/{self.min_frames}", (100, 120), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        motion_text = "MOVING" if hand_moving else "STATIONARY"
        motion_color = (0, 200, 255) if hand_moving else (100, 100, 100)
        cv2.putText(image, f"Hand: {motion_text}", (250, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, motion_color, 1)

        # Controls hint at bottom
        cv2.putText(image, "Q: Quit  R: Reset  Space: Info", (10, h - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
        
        return image

    def _draw_hand_landmarks(self, image, landmarks_list):
        h, w = image.shape[:2]
        
        # landmarks_list is a list of NormalizedLandmarkList (one per hand)
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
