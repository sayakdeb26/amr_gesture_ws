import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import py_trees
import time

# ---------------------
# Blackboard
# ---------------------
class Blackboard:
    login = False
    aktuelle_geste = None

blackboard = Blackboard()

# ---------------------
# ROS2 Subscriber Node mit Publisher
# ---------------------
class GestureSubscriber(Node):
    def __init__(self, blackboard):
        super().__init__('gesture_subscriber')
        self.blackboard = blackboard
        self.subscription = self.create_subscription(
            String,
            '/gesture',
            self.listener_callback,
            10)

        self.screen_text_publisher = self.create_publisher(String, '/screen_text', 10)
        self.stopping_state_publisher = self.create_publisher(Bool, '/stopping_state', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Empfangene Geste: {msg.data}")
        self.blackboard.aktuelle_geste = msg.data

    def zeige(self, text):
        msg = String()
        msg.data = text
        self.screen_text_publisher.publish(msg)
        self.get_logger().info(f"[SCREEN PUBLISHED]: {text}")

    def publish_stopping_state(self, value: bool):
        msg = Bool()
        msg.data = value
        self.stopping_state_publisher.publish(msg)
        self.get_logger().info(f"[STOPPING STATE PUBLISHED]: {value}")

# ---------------------
# Dummyfunktionen
# ---------------------
def folge_mensch():
    print("Folge dem Menschen")

def fahre_zum_ziel():
    print("Fahre zum gezeigten Ziel")

def stoppe():
    print("Stoppe sofort")

# ---------------------
# Conditions
# ---------------------
class CheckLogin(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="CheckLogin")

    def update(self):
        return py_trees.common.Status.SUCCESS if blackboard.login else py_trees.common.Status.FAILURE

class CheckNotLogin(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="CheckNotLogin")

    def update(self):
        return py_trees.common.Status.SUCCESS if not blackboard.login else py_trees.common.Status.FAILURE

class GesteIst(py_trees.behaviour.Behaviour):
    def __init__(self, zielgeste):
        super().__init__(name=f"Geste == {zielgeste}")
        self.zielgeste = zielgeste

    def update(self):
        return py_trees.common.Status.SUCCESS if blackboard.aktuelle_geste == self.zielgeste else py_trees.common.Status.FAILURE

# ---------------------
# Actions
# ---------------------
class SetLogin(py_trees.behaviour.Behaviour):
    def __init__(self, wert):
        super().__init__(name=f"SetLogin({wert})")
        self.wert = wert

    def update(self):
        blackboard.login = self.wert
        return py_trees.common.Status.SUCCESS

class ZeigeText(py_trees.behaviour.Behaviour):
    def __init__(self, text, node):
        super().__init__(name=f"Zeige: {text}")
        self.text = text
        self.node = node

    def update(self):
        self.node.zeige(self.text)
        return py_trees.common.Status.SUCCESS

class ZeigeGeste(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super().__init__(name="Zeige aktuelle Geste")
        self.node = node

    def update(self):
        g = blackboard.aktuelle_geste if blackboard.aktuelle_geste else "keine"
        self.node.zeige(f"Geste erkannt: {g}")
        return py_trees.common.Status.SUCCESS

class FolgeAktion(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="FolgeAktion")

    def update(self):
        folge_mensch()
        return py_trees.common.Status.RUNNING

class FahreAktion(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="FahreAktion")

    def update(self):
        fahre_zum_ziel()
        return py_trees.common.Status.RUNNING

class StoppeAktion(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super().__init__(name="StoppeAktion")
        self.node = node
        self.start_time = None

    def initialise(self):
        stoppe()
        self.node.zeige("stoppt")
        self.node.publish_stopping_state(True)
        self.start_time = time.time()

    def update(self):
        if self.start_time is None:
            return py_trees.common.Status.RUNNING
        if time.time() - self.start_time < 2.0:
            return py_trees.common.Status.RUNNING
        else:
            self.node.publish_stopping_state(False)
            blackboard.aktuelle_geste = None  # Zurücksetzen für nächste Geste
            return py_trees.common.Status.SUCCESS

# ---------------------
# Behavior Tree Aufbau
# ---------------------
def create_behavior_tree(node):
    root = py_trees.composites.Selector(name="Root", memory=False)

    # --- Stopp immer prüfen ---
    stopp_sequence = py_trees.composites.Sequence(name="Sofortiger Stopp", memory=False)
    stopp_sequence.add_children([
        GesteIst("stopp"),
        StoppeAktion(node)
    ])

    # --- Login-Phase ---
    login_phase_selector = py_trees.composites.Selector(name="Login-Check-Selector", memory=False)
    login_success_sequence = py_trees.composites.Sequence(name="Login-Erfolgreich", memory=False)
    login_success_sequence.add_children([
        GesteIst("login"),
        SetLogin(True),
        ZeigeText("Login OK", node)
    ])
    login_phase_selector.add_children([
        login_success_sequence,
        ZeigeText("standby, warten auf Person", node)
    ])
    login_sequence = py_trees.composites.Sequence(name="Login-Phase", memory=False)
    login_sequence.add_children([
        CheckNotLogin(),
        login_phase_selector
    ])

    # --- Logoff ---
    logoff_sequence = py_trees.composites.Sequence(name="Logoff", memory=False)
    logoff_sequence.add_children([
        GesteIst("logoff"),
        SetLogin(False),
        ZeigeText("Abgemeldet", node)
    ])

    # --- Folgen ---
    folgen_sequence = py_trees.composites.Sequence(name="Folgen", memory=False)
    folgen_sequence.add_children([
        GesteIst("folgen"),
        py_trees.decorators.Repeat(
            name="Wiederhole Folge bis Fehler",
            child=FolgeAktion(),
            num_success=-1
        )
    ])

    # --- Zeigen ---
    zeigen_sequence = py_trees.composites.Sequence(name="Zeigen", memory=False)
    zeigen_sequence.add_children([
        GesteIst("zeigen"),
        ZeigeText("Ziel erkannt", node),
        py_trees.decorators.Repeat(
            name="Wiederhole Fahrt bis Fehler",
            child=FahreAktion(),
            num_success=-1
        )
    ])

    # --- Steuerung ---
    interaktion_selector = py_trees.composites.Selector(name="Steuerung", memory=False)
    interaktion_selector.add_children([
        logoff_sequence,
        folgen_sequence,
        zeigen_sequence,
        ZeigeText("Warte auf Geste", node)
    ])

    interaktion_sequence = py_trees.composites.Sequence(name="Interaktion", memory=False)
    interaktion_sequence.add_children([
        CheckLogin(),
        interaktion_selector
    ])

    # --- Root ---
    root.add_children([
        stopp_sequence,        # höchste Priorität
        interaktion_sequence,
        login_sequence,
        ZeigeGeste(node)
    ])

    return root

# ---------------------
# Hauptfunktion
# ---------------------
def main():
    rclpy.init()
    gesture_node = GestureSubscriber(blackboard)
    tree = py_trees.trees.BehaviourTree(create_behavior_tree(gesture_node))
    tree.setup(timeout=15)

    try:
        while rclpy.ok():
            rclpy.spin_once(gesture_node, timeout_sec=0.1)
            tree.tick()
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    gesture_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
