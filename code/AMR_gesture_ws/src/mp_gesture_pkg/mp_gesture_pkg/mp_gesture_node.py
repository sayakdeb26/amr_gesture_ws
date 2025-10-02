#!/usr/bin/env python3
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from amr_interfaces.msg import Intent

import mediapipe as mp
mp_hands = mp.solutions.hands

def vec(a, b):
    return (b[0]-a[0], b[1]-a[1])

def dist(a, b):
    return math.hypot(b[0]-a[0], b[1]-a[1])

def is_extended(tip, pip, wrist, scale):
    # finger extended if tip further from wrist than pip by a margin
    return (dist(tip, wrist) - dist(pip, wrist)) > (0.1 * scale)

def thumb_direction(thumb_tip, thumb_ip, wrist):
    # +x right, -x left in image coords (after RGB->BGR we keep same indices)
    v = vec(thumb_ip, thumb_tip)
    ang = math.degrees(math.atan2(v[1], v[0]))
    return ang  # rough orientation

def classify_gesture(lm2d):
    """
    lm2d: list of (x,y) pixel coords length 21 (MediaPipe Hands order)
    Returns (label, conf)
    """
    # indices
    WRIST = 0
    TH_TIP, TH_IP = 4, 3
    IDX_TIP, IDX_PIP = 8, 6
    MID_TIP, MID_PIP = 12, 10
    RNG_TIP, RNG_PIP = 16, 14
    LIT_TIP, LIT_PIP = 20, 18

    wrist = lm2d[WRIST]
    scale = dist(lm2d[TH_TIP], wrist) + dist(lm2d[IDX_TIP], wrist)  # crude size proxy

    f_idx = is_extended(lm2d[IDX_TIP], lm2d[IDX_PIP], wrist, scale)
    f_mid = is_extended(lm2d[MID_TIP], lm2d[MID_PIP], wrist, scale)
    f_rng = is_extended(lm2d[RNG_TIP], lm2d[RNG_PIP], wrist, scale)
    f_lit = is_extended(lm2d[LIT_TIP], lm2d[LIT_PIP], wrist, scale)

    # thumb: compare tip to IP vs wrist, or angle
    th_ext = dist(lm2d[TH_TIP], wrist) > dist(lm2d[TH_IP], wrist) + 0.06 * scale
    th_ang = thumb_direction(lm2d[TH_TIP], lm2d[TH_IP], wrist)

    extended_count = sum([f_idx, f_mid, f_rng, f_lit])
    folded_count   = 4 - extended_count

    # Simple patterns
    if th_ext and extended_count == 4:
        return ("OPEN_PALM", 0.9)
    if (not th_ext) and folded_count == 4:
        return ("FIST", 0.9)
    if f_idx and (not f_mid) and (not f_rng) and (not f_lit):
        return ("POINT_UP", 0.75)
    if th_ext and (not f_idx) and (not f_mid) and (not f_rng) and (not f_lit):
        # Use thumb angle to decide up vs down roughly (image y increases downward)
        if th_ang < -30:   # pointing right-ish
            return ("THUMBS_RIGHT", 0.7)
        if th_ang > 30:    # left-ish
            return ("THUMBS_LEFT", 0.7)
        # vertical-ish thumb: compare y
        if lm2d[TH_TIP][1] < lm2d[TH_IP][1]:
            return ("THUMBS_UP", 0.8)
        else:
            return ("THUMBS_DOWN", 0.8)
    if f_idx and f_mid and (not f_rng) and (not f_lit):
        return ("VICTORY", 0.75)

    # OK gesture heuristic (index tip near thumb tip)
    if dist(lm2d[IDX_TIP], lm2d[TH_TIP]) < 0.12 * scale and f_mid and f_rng and f_lit:
        return ("OK", 0.7)

    return ("UNKNOWN", 0.0)

class MPGNode(Node):
    def __init__(self):
        super().__init__('mp_gesture_node')
        self.declare_parameter('image_topic', '/image_raw_60hz')
        self.declare_parameter('min_conf', 0.6)
        self.declare_parameter('debug', False)  # if you want overlay later

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.min_conf = float(self.get_parameter('min_conf').value)
        self.debug = bool(self.get_parameter('debug').value)

        self.bridge = CvBridge()
        qos = rclpy.qos.QoSProfile(depth=10)
        self.pub_intent = self.create_publisher(Intent, '/intents_raw', qos)
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, qos)

        self.hands = mp_hands.Hands(static_image_mode=False,
                                    max_num_hands=2,
                                    min_detection_confidence=0.5,
                                    min_tracking_confidence=0.5)
        self.get_logger().info(f'mp_gesture_node up. image_topic={self.image_topic}, min_conf={self.min_conf:.2f}')

    def on_image(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)
        if not res.multi_hand_landmarks:
            return

        h, w = bgr.shape[:2]
        best_label, best_conf = None, 0.0

        for hand_landmarks in res.multi_hand_landmarks:
            lm2d = [(lm.x * w, lm.y * h) for lm in hand_landmarks.landmark]
            label, conf = classify_gesture(lm2d)
            if conf > best_conf:
                best_label, best_conf = label, conf

        if best_label and best_conf >= self.min_conf and best_label != "UNKNOWN":
            out = Intent()
            out.stamp = self.get_clock().now().to_msg()
            out.source = 'mp_rule'
            out.label = best_label
            out.confidence = float(best_conf)
            self.pub_intent.publish(out)

def main():
    rclpy.init()
    node = MPGNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
