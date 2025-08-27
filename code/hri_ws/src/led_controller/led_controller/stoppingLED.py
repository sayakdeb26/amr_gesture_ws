import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, MultiArrayDimension, Int32MultiArray

class StoppingLED(Node):
    def __init__(self):
        super().__init__('stop_led_controller')
        self.declare_parameter('num_leds', 100)
        self.num_leds = self.get_parameter('num_leds').value

        self.subscription = self.create_subscription(
            Bool,
            '/stopping_state',
            self.stopping_callback,
            10
        )
        self.led_publisher = self.create_publisher(Int32MultiArray, '/stoppingLEDs', 10)

    def stopping_callback(self, msg: Bool):
        if msg.data:
            # Alle LEDs auf rot
            led_data = [[led_id, 255, 0, 0, 255] for led_id in range(self.num_leds)]
            self.get_logger().info('Stopping detected: Turning all LEDs red')
        else:
            # Alle LEDs aus
            led_data = [[led_id, 0, 0, 0, 0] for led_id in range(self.num_leds)]
            # self.get_logger().info('Not stopping: no stopping lights')

        self.publish_led_data(led_data)

    def publish_led_data(self, led_data):
        msg = Int32MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "leds"
        msg.layout.dim[0].size = len(led_data)
        msg.layout.dim[0].stride = 5  # id, r, g, b, brightness
        msg.data = [value for led in led_data for value in led]
        self.led_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StoppingLED()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
