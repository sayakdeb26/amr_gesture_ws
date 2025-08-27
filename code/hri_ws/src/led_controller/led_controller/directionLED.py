import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import MultiArrayDimension, Int32MultiArray  # Use MultiArray for structured data
from rclpy.timer import Timer

class DirectionLED(Node):
    def __init__(self):
        super().__init__('led_controller')
        self.declare_parameter('num_leds', 100)
        self.num_leds = self.get_parameter('num_leds').value
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.led_publisher = self.create_publisher(Int32MultiArray, '/directionLEDs', 10)
        
        # Timer, um LEDs nach 3 Sekunden Inaktivität auszuschalten
        self.timeout_timer = self.create_timer(3.0, self.turn_off_leds)
        self.last_received_time = self.get_clock().now()

    def cmd_vel_callback(self, msg: Twist):
        self.last_received_time = self.get_clock().now()
        self.set_leds_from_twist(msg)

    def set_leds_from_twist(self, msg: Twist):
        led_data = []
        angular_z = msg.angular.z
        central_led = int(self.map_value(self.constrain(angular_z * -10, -150, 150), -150, 150, 0, self.num_leds - 1))
        start_led = self.constrain(max(0, central_led - 1), 0, self.num_leds - 1)
        end_led = self.constrain(min(self.num_leds - 1, central_led + 1), 0, self.num_leds - 1)

        for led_id in range(self.num_leds):
            if start_led <= led_id <= end_led:
                led_data.append([led_id, 0, 0, 255, 255])  # Blau mit voller Helligkeit
            else:
                led_data.append([led_id, 0, 0, 0, 0])  # Ausschalten

        self.publish_led_data(led_data)
        self.get_logger().info(f'Setting LEDs {start_led}-{end_led} to blue')

    def turn_off_leds(self):
        if (self.get_clock().now() - self.last_received_time).nanoseconds > 3 * 1e9:
            led_data = [[led_id, 0, 0, 0, 0] for led_id in range(self.num_leds)]  # Alle LEDs aus
            self.publish_led_data(led_data)
            self.get_logger().info('No cmd_vel received for 3s, turning off LEDs')

    def publish_led_data(self, led_data):
        msg = Int32MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "leds"
        msg.layout.dim[0].size = len(led_data)
        msg.layout.dim[0].stride = 5  # 5 Elemente pro LED (id, r, g, b, brightness)
        msg.data = [element for sublist in led_data for element in sublist]
        self.led_publisher.publish(msg)
    
    def constrain(self, val, min_val, max_val):
        return max(min(val, max_val), min_val)

    def map_value(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def main(args=None):
    rclpy.init(args=args)
    node = DirectionLED()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
