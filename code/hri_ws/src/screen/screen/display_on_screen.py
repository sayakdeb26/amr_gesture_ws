import sys
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QColor, QPalette
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        
        # Haupttext oben
        self.subscription = self.create_subscription(
            String,
            '/screen_text',
            self.listener_callback,
            10)
        
        # Zweiter Text unten
        self.low_subscription = self.create_subscription(
            String,
            '/screen_text/low',
            self.low_callback,
            10)

        self.text = "Warte auf Nachricht..."
        self.color = QColor(220, 220, 220)  # Standardfarbe hellgrau
        self.low_text = ""  # Neue zweite Zeile

    def listener_callback(self, msg):
        if msg.data.lower().startswith("rot:"):
            self.color = QColor(255, 100, 100)
            self.text = msg.data[4:].strip()
        elif msg.data.lower().startswith("grün:") or msg.data.lower().startswith("gruen:"):
            self.color = QColor(100, 255, 100)
            self.text = msg.data[5:].strip()
        else:
            self.color = QColor(220, 220, 220)
            self.text = msg.data

    def low_callback(self, msg):
        self.low_text = msg.data  # Einfacher Text unten

class MainWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("ROS2 GUI mit zwei Zeilen")
        self.resize(600, 400)

        self.label_top = QLabel()
        self.label_top.setAlignment(Qt.AlignCenter)
        self.label_top.setStyleSheet("font-size: 36px; font-weight: bold;")
        self.label_top.setAutoFillBackground(True)

        self.label_bottom = QLabel()
        self.label_bottom.setAlignment(Qt.AlignCenter)
        self.label_bottom.setStyleSheet("font-size: 24px; color: gray;")

        layout = QVBoxLayout()
        layout.addWidget(self.label_top)
        layout.addWidget(self.label_bottom)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_labels)
        self.timer.start(100)

    def update_labels(self):
        # Oberes Label mit Farbe
        self.label_top.setText(self.node.text)
        pal = self.label_top.palette()
        pal.setColor(QPalette.Window, self.node.color)
        self.label_top.setPalette(pal)

        # Unteres Label ohne Farbänderung
        self.label_bottom.setText(self.node.low_text)

def main():
    rclpy.init()
    node = GuiNode()

    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    sys.exit(app.exec())

if __name__ == '__main__':
    main()
