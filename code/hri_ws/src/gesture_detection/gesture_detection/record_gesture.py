import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
from collections import deque
import threading

SEQ_LEN = 30
NUM_FEATURES = 63  # 21 landmarks * 3 coords
SAVE_DIR = "data_new"
CLASSES_FILE = "gesture_recognition/gesture_recognition/resources/classes.json"

def load_class_list(filename=CLASSES_FILE):
    if os.path.exists(filename):
        with open(filename, "r") as f:
            return json.load(f)
    return []

def save_class_list(classes, filename=CLASSES_FILE):
    with open(filename, "w") as f:
        json.dump(sorted(classes), f)
    print(f"[INFO] Saved classes to {filename}: {sorted(classes)}")

def save_sequence(X_seq, class_label):
    os.makedirs(SAVE_DIR, exist_ok=True)
    save_path = os.path.join(SAVE_DIR, f"gesture_{class_label}.npz")

    if os.path.exists(save_path):
        data = np.load(save_path)
        X_existing, y_existing = data["X"], data["y"]
        X = np.concatenate([X_existing, X_seq[None, :, :]], axis=0)
        y = np.concatenate([y_existing, np.array([class_label], dtype=np.int64)], axis=0)
    else:
        X = X_seq[None, :, :]
        y = np.array([class_label], dtype=np.int64)

    np.savez(save_path, X=X, y=y)
    print(f"[INFO] Saved sequence to {save_path}. Total samples: {len(y)}")

    classes = load_class_list()
    if class_label not in classes:
        classes.append(class_label)
        save_class_list(classes)

# Landmarks connections for drawing lines (like MediaPipe HAND_CONNECTIONS)
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),       # Thumb
    (0, 5), (5, 6), (6, 7), (7, 8),       # Index
    (0, 9), (9, 10), (10, 11), (11, 12),  # Middle
    (0, 13), (13, 14), (14, 15), (15, 16),# Ring
    (0, 17), (17, 18), (18, 19), (19, 20) # Pinky
]

class GestureCollectorNode(Node):
    def __init__(self):
        super().__init__('gesture_collector')

        self.bridge = CvBridge()

        self.subscription_img = self.create_subscription(
            Image,
            '/zed/zed_node/left/image_rect_color',  # Change if your topic is different
            self.image_callback,
            10)

        self.subscription_kp = self.create_subscription(
            String,
            '/gesture/keypoints',
            self.keypoints_callback,
            10)

        self.frame_buffer = deque(maxlen=SEQ_LEN)
        self.latest_keypoints = None
        self.latest_image = None
        self.lock = threading.Lock()

        # Start thread for key press listening
        self.keypress_thread = threading.Thread(target=self.listen_for_keypress, daemon=True)
        self.keypress_thread.start()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")
            return

        with self.lock:
            self.latest_image = cv_image.copy()
            # Overlay keypoints if available
            if self.latest_keypoints is not None:
                self.draw_keypoints(self.latest_image, self.latest_keypoints)

        cv2.imshow("ZED Gesture Collection", self.latest_image)
        cv2.waitKey(1)

    def keypoints_callback(self, msg):
        try:
            parts = list(map(float, msg.data.strip().split(',')))
            if len(parts) != NUM_FEATURES:
                self.get_logger().warn(f"Received keypoints length {len(parts)} != {NUM_FEATURES}")
                return
            keypoints = np.array(parts, dtype=np.float32)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse keypoints: {e}")
            return

        with self.lock:
            self.latest_keypoints = keypoints
            self.frame_buffer.append(keypoints)

    def draw_keypoints(self, image, keypoints):
        # keypoints is 63 floats (21 points * 3 coords)
        h, w, _ = image.shape
        pts = []
        for i in range(21):
            x = int(keypoints[i*3] * w)
            y = int(keypoints[i*3 + 1] * h)
            pts.append((x, y))
            cv2.circle(image, (x, y), 5, (0, 255, 0), -1)

        # Draw connections
        for connection in HAND_CONNECTIONS:
            pt1 = pts[connection[0]]
            pt2 = pts[connection[1]]
            cv2.line(image, pt1, pt2, (0, 255, 255), 2)

    def listen_for_keypress(self):
        print("[INFO] Press number keys (0-9) in this terminal to save buffered gesture. Ctrl+C to exit.")
        while True:
            try:
                label = input()
                if not label.isdigit() or not (0 <= int(label) <= 9):
                    print("[WARN] Please enter a number between 0-9.")
                    continue
                label = int(label)

                with self.lock:
                    if len(self.frame_buffer) == SEQ_LEN:
                        seq_array = np.stack(self.frame_buffer, axis=0)
                        save_sequence(seq_array, label)
                    else:
                        print(f"[WARN] Not enough frames collected: {len(self.frame_buffer)}/{SEQ_LEN}")
            except Exception as e:
                print(f"[ERROR] Error in keypress listener: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GestureCollectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down node...")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
