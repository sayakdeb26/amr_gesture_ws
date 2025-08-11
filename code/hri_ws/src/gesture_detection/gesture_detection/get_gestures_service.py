import rclpy
from rclpy.node import Node
from gesture_detection_interface.srv import GetGestures
import json

class GestureServiceNode(Node):
    def __init__(self):
        super().__init__('gesture_service_node')

        self.srv = self.create_service(GetGestures, 'get_gestures', self.get_gestures_callback)

        self.meanings = {
            0: 'open palm',
            1: 'fist',
            2: 'thumbs up',
            3: 'pointing',
            4: 'waving',
        }

        self.get_logger().info('Gesture service ready.')

    def get_gestures_callback(self, request, response):
        try:
            with open('src/gesture_detection/resource/classes.json', 'r') as f:
                self.gesture_ids = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load classes.json: {e}")
            response.gestures = []
            return response

        response.gestures = [f"{gid}: {self.meanings.get(gid, 'tbd')}" for gid in self.gesture_ids]
        self.get_logger().info(f'Returning available gestures: {response.gestures}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GestureServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
