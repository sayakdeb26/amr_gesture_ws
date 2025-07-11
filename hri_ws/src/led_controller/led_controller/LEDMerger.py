import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, MultiArrayDimension

class LEDMerger(Node):
    def __init__(self):
        super().__init__('led_merger')
        
        # Definiere die Anzahl der LEDs (Standardwert 100)
        self.declare_parameter('num_leds', 100)
        self.num_leds = self.get_parameter('num_leds').value
        
        # Abonnieren der Themen für Person-LEDs und Richtungs-LEDs
        self.subscription_person = self.create_subscription(
            Int32MultiArray,
            'personLEDs',
            self.person_led_callback,
            10
        )
        
        self.subscription_direction = self.create_subscription(
            Int32MultiArray,
            'directionLEDs',
            self.direction_led_callback,
            10
        )
        
        self.subscription_stopping = self.create_subscription(
            Int32MultiArray,
            'stoppingLEDs',
            self.stopping_led_callback,
            10
        )

        # Publisher für die zusammengeführten LED-Daten
        self.led_publisher = self.create_publisher(Int32MultiArray, 'LED_strip', 10)
        
        # Initialisierung der LED-Daten für Person und Richtung
        self.person_leds = [0] * (self.num_leds * 5)
        self.direction_leds = [0] * (self.num_leds * 5)
        self.merged_leds = [0] * (self.num_leds * 5)  # Variable zum Speichern der zusammengeführten LEDs
        self.stopping_leds = [0] * (self.num_leds * 5)

    def person_led_callback(self, msg: Int32MultiArray):
        # Verarbeite die Person-LED-Daten
        stride = msg.layout.dim[0].stride  # Holen der Stride (5 in diesem Fall)
        for i in range(0, len(msg.data), stride):
            led_id = msg.data[i]
            r = msg.data[i+1]
            g = msg.data[i+2]
            b = msg.data[i+3]
            brightness = msg.data[i+4]
            self.person_leds[i:i+5] = [led_id, r, g, b, brightness]  # Update die LED-Daten für Person
        
        self.get_logger().info(f"Received Person LED Data")
        self.merge_leds()  # Merge nach dem Empfang der Person-LED-Daten
    
    def direction_led_callback(self, msg: Int32MultiArray):
        # Verarbeite die Richtungs-LED-Daten
        stride = msg.layout.dim[0].stride  # Holen der Stride (5 in diesem Fall)
        for i in range(0, len(msg.data), stride):
            led_id = msg.data[i]
            r = msg.data[i+1]
            g = msg.data[i+2]
            b = msg.data[i+3]
            brightness = msg.data[i+4]
            self.direction_leds[i:i+5] = [led_id, r, g, b, brightness]  # Update die LED-Daten für Richtung
        
        self.get_logger().info(f"Received Direction LED Data")
        self.merge_leds()  # Merge nach dem Empfang der Richtungs-LED-Daten

    def stopping_led_callback(self, msg: Int32MultiArray):
        # Verarbeite die Richtungs-LED-Daten
        self.get_logger().info("✅ Stopping-Callback wurde aufgerufen!")
        stride = msg.layout.dim[0].stride  # Holen der Stride (5 in diesem Fall)
        for i in range(0, len(msg.data), stride):
            led_id = msg.data[i]
            r = msg.data[i+1]
            g = msg.data[i+2]
            b = msg.data[i+3]
            brightness = msg.data[i+4]
            self.stopping_leds[i:i+5] = [led_id, r, g, b, brightness]  # Update die LED-Daten für Richtung
        
        self.get_logger().info(f"Received stopping LED Data")
        self.merge_leds()  # Merge nach dem Empfang der Richtungs-LED-Daten

    def merge_leds(self):
        # Merging-Logik: Daten von Person-LEDs und Richtungs-LEDs zusammenführen
        merged_leds_local = [0] * (self.num_leds * 5)
        for i in range(self.num_leds):
            merged_leds_local[i * 5] = i  # Setze die IDs von 0 bis 99
            merged_leds_local[i * 5 + 1] = 0  # R = 0
            merged_leds_local[i * 5 + 2] = 0  # G = 0
            merged_leds_local[i * 5 + 3] = 0  # B = 0
            merged_leds_local[i * 5 + 4] = 0  # Helligkeit = 0
        
        # Zusammenführen der Arrays: prio: 3 stopping, 2 person, 1 direction
        for i in range(0, len(merged_leds_local), 5):
            if self.stopping_leds[i+1] != 0 or self.stopping_leds[i+2] != 0 or self.stopping_leds[i+3] != 0:
                merged_leds_local[i:i+5] = self.stopping_leds[i:i+5] 
            # Wenn Person-LED-Daten vorhanden sind, benutze diese
            elif self.person_leds[i+1] != 0 or self.person_leds[i+2] != 0 or self.person_leds[i+3] != 0:
                merged_leds_local[i:i+5] = self.person_leds[i:i+5]  # Benutze Person-LED-Daten
            # Wenn keine Person-LED-Daten vorhanden sind, aber Richtungs-LED-Daten, benutze diese
            elif self.direction_leds[i+1] != 0 or self.direction_leds[i+2] != 0 or self.direction_leds[i+3] != 0:
                merged_leds_local[i:i+5] = self.direction_leds[i:i+5]  # Benutze Richtungs-LED-Daten
 

        # Speichern der zusammengeführten LED-Daten in der Instanzvariablen
        self.merged_leds = merged_leds_local  # Speichern in der Instanzvariablen

        # Erstelle eine Nachricht, um die zusammengeführten LED-Daten zu veröffentlichen
        led_msg = Int32MultiArray()
       # led_msg.layout.dim.append(MultiArrayDimension())
       # led_msg.layout.dim[0].label = "leds"
       # led_msg.layout.dim[0].size = self.num_leds
       # led_msg.layout.dim[0].stride = 5  # 5 Elemente pro LED (id, r, g, b, brightness)
        
        led_msg.layout.dim = []  # Leeres Array für dim
        led_msg.layout.data_offset = 0

        led_msg.data = self.merged_leds  # Setze die zusammengeführten Daten
        
        # Veröffentliche die zusammengeführten LED-Daten
        self.led_publisher.publish(led_msg)
        self.get_logger().info(f'Merged LED data published:{led_msg.data}')

def main():
    # Initialisiere den ROS2-Node und starte ihn
    rclpy.init()
    node = LEDMerger()
    rclpy.spin(node)  # Der Node bleibt aktiv und wartet auf Nachrichten
    node.destroy_node()
    rclpy.shutdown()  # Schließe ROS2 nach dem Beenden des Nodes

if __name__ == '__main__':
    main()
