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
        self.subscription = self.create_subscription(
            String,
            '/screen_text',
            self.listener_callback,
            10)
        self.text = "Warte auf Nachricht..."
        self.color = QColor(220, 220, 220)  # Standardfarbe hellgrau

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

class MainWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("ROS2 GUI mit farbigem Feld")
        self.resize(600, 400)

        self.label = QLabel()
        self.label.setAlignment(Qt.AlignCenter)  # horizontal + vertikal zentriert
        self.label.setStyleSheet("font-size: 36px; font-weight: bold;")
        self.label.setAutoFillBackground(True)  # Wichtig, damit Hintergrundfarbe wirkt

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_label)
        self.timer.start(100)

    def update_label(self):
        self.label.setText(self.node.text)
        pal = self.label.palette()
        pal.setColor(QPalette.Window, self.node.color)
        self.label.setPalette(pal)

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
