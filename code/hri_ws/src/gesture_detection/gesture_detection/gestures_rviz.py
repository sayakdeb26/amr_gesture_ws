import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),        # Thumb
    (0, 5), (5, 6), (6, 7), (7, 8),        # Index
    (5, 9), (9, 10), (10, 11), (11, 12),   # Middle
    (9, 13), (13, 14), (14, 15), (15, 16), # Ring
    (13, 17), (17, 18), (18, 19), (19, 20),# Pinky
    (0, 17)                                # Palm base
]

def make_point(coords):
    p = Point()
    p.x, p.y, p.z = coords
    return p

def create_sphere_marker(ns, id, position, color, scale=0.01):
    m = Marker()
    m.header.frame_id = "zed_camera_center"
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.ns = ns
    m.id = id
    m.pose.position.x = position[0]
    m.pose.position.y = position[1]
    m.pose.position.z = position[2]
    m.scale.x = m.scale.y = m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = color
    return m

def create_line_marker(ns, id, points, color, scale=0.003):
    m = Marker()
    m.header.frame_id = "zed_camera_center"
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.ns = ns
    m.id = id
    m.scale.x = scale
    m.color.r, m.color.g, m.color.b, m.color.a = color

    for start, end in HAND_CONNECTIONS:
        # Linien vom Ursprungspunkt 0 weglassen
        if start == 0 or end == 0:
            continue
        if start < len(points) and end < len(points):
            m.points.append(make_point(points[start]))
            m.points.append(make_point(points[end]))
    return m


class MultiTopicKeypointVisualizer(Node):
    def __init__(self):
        super().__init__('multi_keypoint_visualizer')

        self.create_subscription(Float32MultiArray, '/gesture/keypoints/combined_3d', self.callback_combined_3d, 1)
        self.pub_combined_3d = self.create_publisher(MarkerArray, '/gesture/keypoints/rviz/combined_3d', 10)

        self.get_logger().info("MultiTopicKeypointVisualizer started and subscribing to all keypoint topics.")

    import math

    def callback_combined_3d(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) % 3 != 0:
            self.get_logger().warn("Combined 3D coords array length invalid")
            return

        points = []
        for i in range(0, len(data), 3):
            x, y, z = data[i], data[i+1], data[i+2]
            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue  # skip this point if any coordinate is NaN
            points.append((x, y, z))

        if not points:
            return  # nothing to publish

        marker_array = MarkerArray()

        # Delete previous markers
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_all.header.frame_id = "zed_camera_center"
        marker_array.markers.append(delete_all)

        # Add spheres
        for i, p in enumerate(points):
            marker_array.markers.append(create_sphere_marker('combined_3d_points', i, p, (1.0, 0.0, 1.0, 1.0), 0.03))

        # Add lines
        marker_array.markers.append(create_line_marker('combined_3d_lines', 1000, points, (0.5, 0.0, 0.5, 1.0), 0.01))

        # Add timestamp
        from rclpy.time import Time
        now = self.get_clock().now().to_msg()
        for m in marker_array.markers:
            m.header.stamp = now

        self.pub_combined_3d.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicKeypointVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
