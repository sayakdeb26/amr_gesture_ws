import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray, String
from zed_msgs.msg import ObjectsStamped

import cv2
from cv_bridge import CvBridge
import mediapipe as mp
import numpy as np
import json

class ArmHand3D(Node):
    def __init__(self):
        super().__init__('arm_hand_3d_processor')

        # Parameters
        self.declare_parameter('max_num_hands', 1)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)

        max_num_hands = self.get_parameter('max_num_hands').value
        min_detection_confidence = self.get_parameter('min_detection_confidence').value
        min_tracking_confidence = self.get_parameter('min_tracking_confidence').value

        # MediaPipe solutions
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )

        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )

        # CvBridge
        self.bridge = CvBridge()

        # Camera intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Image buffers
        self.rgb_image = None
        self.depth_image = None

        # ZED skeletons for comparison
        self.zed_arm_3d = []

        # Subscribers
        self.create_subscription(CameraInfo, '/zed/zed_node/left/camera_info', self.cam_info_callback, 1)
        self.create_subscription(Image, '/zed/zed_node/left/image_rect_color', self.image_callback, 10)
        self.create_subscription(Image, '/zed/zed_node/depth/depth_registered', self.depth_callback, 10)
        self.create_subscription(ObjectsStamped, '/zed/zed_node/body_trk/skeletons', self.skeleton_callback, 10)

        # Publishers
        self.pub_combined_3d = self.create_publisher(Float32MultiArray, '/gesture/keypoints/combined_3d', 10)
        self.pub_arm_error = self.create_publisher(Float32MultiArray, '/gesture/keypoints/arm_error', 10)

    def cam_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def image_callback(self, msg: Image):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_if_ready()

    def depth_callback(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.process_if_ready()

    def skeleton_callback(self, msg: ObjectsStamped):
        self.zed_arm_3d = []
        for obj in msg.objects:
            if hasattr(obj, 'skeleton_3d') and obj.skeleton_3d:
                for kp3d in obj.skeleton_3d.keypoints:
                    x, y, z = kp3d.kp
                    self.zed_arm_3d.extend([float(x), float(y), float(z)])
        self.process_if_ready()

    def process_if_ready(self):
        if self.rgb_image is None or self.depth_image is None or self.fx is None:
            return

        rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
        height, width, _ = rgb_image.shape
        display_image = self.rgb_image.copy()  # copy for visualization

        # --- MediaPipe Pose for Arm ---
        pose_results = self.pose.process(rgb_image)
        arm_3d = []
        wrist_z = 0.0  # Referenztiefe für Hand

        if pose_results.pose_landmarks:
            arm_indices = [self.mp_pose.PoseLandmark.LEFT_SHOULDER,
                        self.mp_pose.PoseLandmark.LEFT_ELBOW,
                        self.mp_pose.PoseLandmark.LEFT_WRIST]
            
            for i, idx in enumerate(arm_indices):
                lm = pose_results.pose_landmarks.landmark[idx]
                u = int(round(lm.x * width))
                v = int(round(lm.y * height))
                
                if 0 <= u < width and 0 <= v < height:
                    # ZED-Tiefe, falls verfügbar
                    if self.zed_arm_3d and len(self.zed_arm_3d) >= (i+1)*3:
                        Z_zed = self.zed_arm_3d[i*3 + 2]
                    else:
                        Z_zed = float('nan')
                    
                    # Wenn ZED NaN/Inf → MediaPipe verwenden
                    if not np.isfinite(Z_zed):
                        # Z_mp ist relativ zur virtuellen Referenz
                        Z = lm.z + 0.0  # hier 0.0 als Referenz, wird später angepasst
                    else:
                        Z = Z_zed
                    
                    # Update Referenz für Handgelenk
                    if idx == self.mp_pose.PoseLandmark.LEFT_WRIST:
                        wrist_z = Z
                    
                    X = (u - self.cx) * Z / self.fx
                    Y = (v - self.cy) * Z / self.fy
                    arm_3d.extend([X, Y, Z])
                    
                    # Draw pose keypoints
                    cv2.circle(display_image, (u, v), 5, (0, 255, 0), -1)
                else:
                    arm_3d.extend([0.0, 0.0, 0.0])


        # --- MediaPipe Hands for Hand ---
        hand_results = self.hands.process(rgb_image)
        hand_3d = []
        if hand_results.multi_hand_landmarks:
            for hand_landmarks in hand_results.multi_hand_landmarks:
                for lm in hand_landmarks.landmark:
                    u = int(round(lm.x * width))
                    v = int(round(lm.y * height))
                    Z_mp = lm.z
                    Z = wrist_z + Z_mp
                    X = (u - self.cx) * Z / self.fx
                    Y = (v - self.cy) * Z / self.fy
                    hand_3d.extend([X, Y, Z])
                    # Draw hand keypoints
                    cv2.circle(display_image, (u, v), 3, (0, 0, 255), -1)

        # --- Combine ---
        combined_3d = arm_3d + hand_3d
        msg_3d = Float32MultiArray()
        msg_3d.data = combined_3d
        self.pub_combined_3d.publish(msg_3d)

        # --- Compare Arm 3D to ZED skeleton ---
        arm_error = []
        if self.zed_arm_3d and len(self.zed_arm_3d) == len(arm_3d):
            for i in range(len(arm_3d)):
                arm_error.append(abs(arm_3d[i] - self.zed_arm_3d[i]))
            msg_error = Float32MultiArray()
            msg_error.data = arm_error
            self.pub_arm_error.publish(msg_error)

        # --- Show the image with keypoints ---
        cv2.imshow('Arm & Hand Keypoints', display_image)
        cv2.waitKey(1)  # 1 ms delay to allow window refresh

def main(args=None):
    rclpy.init(args=args)
    node = ArmHand3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
