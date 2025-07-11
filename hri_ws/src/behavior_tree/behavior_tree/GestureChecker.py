import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GestureChecker(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name):
        py_trees.behaviour.Behaviour.__init__(self, name)
        Node.__init__(self, "gesture_checker_node")
        self.gesture_received = None

        # Subscriber für das /gesture-Topic
        self.subscription = self.create_subscription(
            String,
            "/gesture",
            self.gesture_callback,
            10
        )

    def gesture_callback(self, msg):
        """Speichert die empfangene Geste"""
        self.gesture_received = msg.data
        self.get_logger().info(f"Received gesture: {msg.data}")

    def update(self):
        """Überprüft, ob die 'stopping' Geste erkannt wurde"""
        if self.gesture_received == "stopping":
            self.get_logger().info("Stopping gesture detected! Returning FAILURE.")
            return py_trees.common.Status.FAILURE
        else:
            self.get_logger().info("No stopping gesture detected. Returning SUCCESS.")
            return py_trees.common.Status.SUCCESS

def create_behavior_tree():
    root = py_trees.composites.Selector("Root")
    notstopp_selector = py_trees.composites.Selector("Notstopp")

    gesture_checker = GestureChecker("GestureChecker")
    
    root.add_children([notstopp_selector])
    notstopp_selector.add_children([gesture_checker])
  
    return py_trees.trees.BehaviourTree(root)

def main():
    rclpy.init()
    tree = create_behavior_tree()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(tree.root.children[0].children[0])  # Füge GestureChecker-Node hinzu

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)  # ROS 2 Callback verarbeiten
            tree.tick()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        tree.root.children[0].children[0].destroy_node()  # Zerstöre GestureChecker-Node
        rclpy.shutdown()

if __name__ == "__main__":
    main()
