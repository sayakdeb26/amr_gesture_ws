import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, MultiArrayDimension, Int32MultiArray
from zed_msgs.msg import ObjectsStamped

class PersonLocatorLED(Node):
    def __init__(self):
        super().__init__('personlocatorLED')
        
        self.declare_parameter('num_leds', 100)
        self.num_leds = self.get_parameter('num_leds').value
        
        self.subscription = self.create_subscription(
            Float32,   
            '/person_x_pose',
            self.x_pose_callback,
            10
        )

        self.zed_subscription = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/obj_det/objects',
            self.zed_callback,
            10
        )
        
        self.led_publisher = self.create_publisher(Int32MultiArray, '/personLEDs', 10)
        self.pose_publisher = self.create_publisher(Float32, '/person_x_pose', 10)

        self.timeout_timer = self.create_timer(3.0, self.timeout_callback)
        self.last_msg_time = self.get_clock().now()
    
    def zed_callback(self, msg: ObjectsStamped):
        for obj in msg.objects:
            if obj.label == "PERSON":
                # X-Wert aus der 3D-Position verwenden
                x_value = float(obj.position[0])  # position = [x, y, z]
                
                # Optional: Werte auf -1 bis 1 normalisieren (abhängig von deinem LED-Mapping)
                # Beispiel: Kamera erkennt x in -2m bis 2m
                # x_normalized = self.constrain(x_value / 2.0, -1.0, 1.0)

                pose_msg = Float32()
                pose_msg.data = x_value
                self.pose_publisher.publish(pose_msg)
                self.last_msg_time = self.get_clock().now()
                self.get_logger().info(f'Published person X pose: {x_value:.3f}')
                break  # Nur die erste Person


    def x_pose_callback(self, msg: Float32):
        self.last_msg_time = self.get_clock().now()
        x_pose = msg.data
        if x_pose is None:
            return
        
        # LED zentral positionieren
        central_led = int(self.map_value(self.constrain(x_pose, -1.0, 1.0), -1.0, 1.0, 0, self.num_leds - 1))
        start_led = max(0, central_led - 1)
        end_led = min(self.num_leds - 1, central_led + 1)
        
        led_data = []
        for led_id in range(self.num_leds):
            if start_led <= led_id <= end_led:
                led_data.append([led_id, 0, 125, 0, 255])  # grün
        self.publish_leds(led_data)
        self.get_logger().info(f'Setting LEDs {start_led}-{end_led} to green')
    
    def timeout_callback(self):
        if (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9 > 3.0:
            self.get_logger().info("No message received for 3s, turning off all LEDs")
            led_data = [[i, 0, 0, 0, 0] for i in range(self.num_leds)]
            self.publish_leds(led_data)
    
    def publish_leds(self, led_data):
        msg = Int32MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "leds"
        msg.layout.dim[0].size = len(led_data)
        msg.layout.dim[0].stride = 5
        msg.data = [element for sublist in led_data for element in sublist]
        self.led_publisher.publish(msg)
    
    @staticmethod
    def constrain(value, min_val, max_val):
        return max(min_val, min(value, max_val))
    
    @staticmethod
    def map_value(value, in_min, in_max, out_min, out_max):
        return out_min + (float(value - in_min) / float(in_max - in_min) * (out_max - out_min))

def main():
    rclpy.init()
    node = PersonLocatorLED()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
