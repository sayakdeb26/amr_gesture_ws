import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import threading

class DirectionLEDVisualizer(Node):
    def __init__(self):
        super().__init__('led_visualizer')

        # Number of LEDs
        self.declare_parameter('num_leds', 100)
        self.num_leds = self.get_parameter('num_leds').value

        # ROS2 Subscription to 'directionLEDs' topic
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'LED_strip',
            self.direction_leds_callback,
            10
        )

        # Initialize Matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(15, 3))
        self.ax.set_xlim(-1, self.num_leds)
        self.ax.set_ylim(-1, 5)
        self.ax.axis('off')

        # Store LED patches for updating
        self.led_circles = [None] * self.num_leds

        # Create initial LED visualization (default LEDs are black)
        for i in range(self.num_leds):
            circle = patches.Circle((i, 2.5), radius=0.3, color='black')
            self.ax.add_patch(circle)
            self.led_circles[i] = circle  # Store reference to update later

        plt.ion()
        plt.show()

        # Start ROS2 in a separate thread
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.ros_thread.start()

        # Start Matplotlib update loop
        self.run_matplotlib_loop()

    def direction_leds_callback(self, msg):
        """Update LEDs based on incoming ROS2 message."""
        led_data = msg.data

        # Iterate through the LED data (format: [id, r, g, b, brightness])
        for i in range(0, len(led_data), 5):
            led_id = led_data[i]
            r, g, b, _ = led_data[i+1:i+5]  # Ignore brightness

            # Normalize RGB to range [0,1]
            color = (r / 255, g / 255, b / 255)

            # Update corresponding LED circle
            if 0 <= led_id < self.num_leds:
                self.led_circles[led_id].set_facecolor(color)


        # Redraw figure
        self.fig.canvas.draw()

    def run_matplotlib_loop(self):
        """Runs Matplotlib in a loop to keep visualization updated."""
        while rclpy.ok():
            plt.pause(0.05)  # Keeps the window responsive

    def destroy_node(self):
        """Shutdown properly when exiting."""
        super().destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DirectionLEDVisualizer()

    try:
        node.run_matplotlib_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
