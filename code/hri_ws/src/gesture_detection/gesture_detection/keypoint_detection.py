import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import mediapipe as mp
import json

class KeypointIdentifier(Node):
    def __init__(self):
        super().__init__('gesture_processor')

        # Parameter einlesen oder Standard setzen
        self.declare_parameter('use_static_image_mode', False)
        self.declare_parameter('max_num_hands', 1)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)

        use_static_image_mode = self.get_parameter('use_static_image_mode').get_parameter_value().bool_value
        max_num_hands = self.get_parameter('max_num_hands').get_parameter_value().integer_value
        min_detection_confidence = self.get_parameter('min_detection_confidence').get_parameter_value().double_value
        min_tracking_confidence = self.get_parameter('min_tracking_confidence').get_parameter_value().double_value

        # MediaPipe Hand-Tracking initialisieren
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=use_static_image_mode,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )

        # CvBridge für Bildkonvertierung
        self.bridge = CvBridge()

        # Subscriber auf /zed_kamera (sensor_msgs/Image)
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/left/image_rect_color',
            self.image_callback,
            10)

        # Publisher für Keypoints (JSON String)
        self.publisher = self.create_publisher(String, '/gesture/keypoints', 10)

        self.get_logger().info("KeypointIdentifier subscribed to /zed/zed_node/left/image_rect_color gestartet, publishing to /gesture/keypoints")

    def image_callback(self, msg):
        try:
            # ROS Image -> OpenCV BGR Bild
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # OpenCV BGR -> RGB für Mediapipe
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Hand Keypoints mit Mediapipe erkennen
            results = self.hands.process(rgb_image)

            keypoints_list = []
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    keypoints = []
                    for lm in hand_landmarks.landmark:
                        keypoints.append({'x': lm.x, 'y': lm.y, 'z': lm.z})
                    keypoints_list.append(keypoints)

            # Keypoints als JSON String publizieren
            keypoints_msg = String()
            keypoints_msg.data = json.dumps(keypoints_list)
            self.publisher.publish(keypoints_msg)

        except Exception as e:
            self.get_logger().error(f'Fehler beim Verarbeiten des Bildes: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = KeypointIdentifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import cv2
# import mediapipe as mp
# import json

# class KeypointIdentifier(Node):
#     def __init__(self):
#         super().__init__('gesture_processor')

#         # 🔧 Parameter einlesen oder Standard setzen
#         self.declare_parameter('use_static_image_mode', False)
#         self.declare_parameter('max_num_hands', 1)
#         self.declare_parameter('min_detection_confidence', 0.5)
#         self.declare_parameter('min_tracking_confidence', 0.5)

#         use_static_image_mode = self.get_parameter('use_static_image_mode').get_parameter_value().bool_value
#         max_num_hands = self.get_parameter('max_num_hands').get_parameter_value().integer_value
#         min_detection_confidence = self.get_parameter('min_detection_confidence').get_parameter_value().double_value
#         min_tracking_confidence = self.get_parameter('min_tracking_confidence').get_parameter_value().double_value

#         # 🔍 MediaPipe Hand-Tracking initialisieren
#         self.mp_hands = mp.solutions.hands
#         self.hands = self.mp_hands.Hands(
#             static_image_mode=use_static_image_mode,
#             max_num_hands=max_num_hands,
#             min_detection_confidence=min_detection_confidence,
#             min_tracking_confidence=min_tracking_confidence
#         )

#         # ROS Publisher
#         self.publisher = self.create_publisher(String, '/gesture_keypoints', 10)

#         self.get_logger().info("KeypointIdentifier mit Webcam gestartet")

#     def run(self):
#         cap = cv2.VideoCapture(0)

#         if not cap.isOpened():
#             self.get_logger().error("Webcam konnte nicht geöffnet werden.")
#             return

#         while rclpy.ok():
#             ret, frame = cap.read()
#             if not ret:
#                 self.get_logger().warn("Kein Bild von der Webcam erhalten.")
#                 continue

#             rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#             results = self.hands.process(rgb_image)

#             keypoints_list = []
#             if results.multi_hand_landmarks:
#                 for hand_landmarks in results.multi_hand_landmarks:
#                     keypoints = []
#                     for lm in hand_landmarks.landmark:
#                         keypoints.append({'x': lm.x, 'y': lm.y, 'z': lm.z})
#                     keypoints_list.append(keypoints)

#                     # Optional: Hand anzeigen (auskommentiert)
#                     # mp.solutions.drawing_utils.draw_landmarks(
#                     #     frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

#             # Keypoints publizieren
#             keypoints_msg = String()
#             keypoints_msg.data = json.dumps(keypoints_list)
#             self.publisher.publish(keypoints_msg)

#             # Fenster-Anzeige auskommentiert
#             # cv2.imshow("Webcam - Keypoints", frame)
#             # if cv2.waitKey(1) & 0xFF == 27:  # ESC zum Beenden
#             #     break

#         cap.release()
#         # cv2.destroyAllWindows()

# def main(args=None):
#     rclpy.init(args=args)
#     node = KeypointIdentifier()
#     try:
#         node.run()
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
