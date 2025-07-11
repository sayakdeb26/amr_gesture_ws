import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from py_trees.common import ParallelPolicy 

class GestureChecker(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name, gesture_to_check):
        py_trees.behaviour.Behaviour.__init__(self, name)
        Node.__init__(self, name)
        self.gesture_received = None
        self.gesture_to_check = gesture_to_check  # Set the gesture to check

        # Subscriber for the /gesture topic
        self.subscription = self.create_subscription(
            String,
            "/gesture",
            self.gesture_callback,
            10
        )

    def gesture_callback(self, msg):
        """Stores the received gesture"""
        self.gesture_received = msg.data
        # Optionally, you can log the received gesture
        # self.get_logger().info(f"Received gesture: {msg.data}")

    def update(self):
        """Checks if the received gesture matches the expected one"""
        if self.gesture_received == self.gesture_to_check:
            self.get_logger().info(f"{self.gesture_to_check} gesture detected!")
            return py_trees.common.Status.SUCESS
        else:
            self.get_logger().info(f"No {self.gesture_to_check} gesture detected.")
            return py_trees.common.Status.FAILURE

class Stopping(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name):
        py_trees.behaviour.Behaviour.__init__(self, name)
        Node.__init__(self, "Stopping")
    
    def update(self):
        self.get_logger().info("Stopping")
        return py_trees.common.Status.SUCCESS

class SetMode(py_trees.behaviour.Behaviour, Node):
    def __init__(self, mode):
        py_trees.behaviour.Behaviour.__init__(self, mode)
        Node.__init__(self, mode)
        self.mode = mode  # Set the gesture to check

    def update(self):
        """Setzt den Modus im BlackBoard oder innerhalb des Verhaltensbaums"""
        # Loggt die Information, dass der Modus gesetzt wurde
        self.get_logger().info(f"Set mode to {self.mode}.")
        # Hier könntest du den Modus auch im BlackBoard speichern, wenn du das möchtest:
        py_trees.blackboard.Blackboard.set('current_mode', self.mode)
        return py_trees.common.Status.SUCCESS

class CheckBoard(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name, parameter, value):       
        # Initialize the py_trees.behaviour.Behaviour with the fixed name
        py_trees.behaviour.Behaviour.__init__(self, name,)
        # Initialize the Node with the fixed name
        Node.__init__(self, name)
        
        # Store the parameter and value to check on the blackboard
        self.parameter = parameter
        self.value = value

    def update(self):
        """Check if the parameter value matches the expected value on the BlackBoard"""
        if self.parameter is None or self.value is None:
            # Handle the case where no parameter or value was provided
            self.get_logger().info("No parameter or value set. Returning FAILURE.")
            return py_trees.common.Status.FAILURE
        
        # Log the information about what we're checking
        #self.get_logger().info(f"Checking if {self.parameter} is set to {self.value}.")
        if py_trees.blackboard.Blackboard.exists(self.parameter):
        # Access the blackboard and get the value of the parameter
            param_value = py_trees.blackboard.Blackboard.get(self.parameter)  # Fetch the value from the blackboard
        
            # Return SUCCESS if the value matches, otherwise FAILURE
            if param_value == self.value:
                self.get_logger().info(f"Success! {self.parameter} is {self.value}.")
                return py_trees.common.Status.SUCCESS
            else:
                self.get_logger().info(f"Failure! {self.parameter} is not {self.value}.")
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.FAILURE

class WriteBoard(py_trees.behaviour.Behaviour, Node):
    def __init__(self, parameter=None, value=None):
        """
        Constructor for CheckBoard behavior.
        :param parameter: (optional) The parameter to check in the blackboard (default: None)
        :param value: (optional) The value to check against (default: None)
        """
        # Fixed name for the behavior node
        name = "WriteBoard" + (parameter if parameter else "")     
        
        # Initialize the py_trees.behaviour.Behaviour with the fixed name
        py_trees.behaviour.Behaviour.__init__(self, name)
        
        # Initialize the Node with the fixed name
        Node.__init__(self, name)
        
        # Store the parameter and value to check on the blackboard
        self.parameter = parameter
        self.value = value

    def update(self):
        """Check if the parameter value matches the expected value on the BlackBoard"""
        if self.parameter is None or self.value is None:
            # Handle the case where no parameter or value was provided
            self.get_logger().info("No parameter or value set. Returning FAILURE.")
            return py_trees.common.Status.FAILURE

        py_trees.blackboard.Blackboard.set(self.parameter,self.value)  # Fetch the value from the blackboard

        return py_trees.common.Status.SUCCESS

class AcknowledgeHuman(py_trees.behaviour.Behaviour, Node):
    def __init__(self):
        py_trees.behaviour.Behaviour.__init__(self, "acknowledgeHuman")
        Node.__init__(self, "acknowledgeHuman")
    
    def update(self):
        self.get_logger().info("Acknowledging Human")
        return py_trees.common.Status.SUCCESS

class display(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name, parameter):       
        # Initialize the py_trees.behaviour.Behaviour with the fixed name
        py_trees.behaviour.Behaviour.__init__(self, name,)
        # Initialize the Node with the fixed name
        Node.__init__(self, name)
        # Store the parameter and value to check on the blackboard
        self.parameter = parameter

    def update(self):        
        # Log the information about what we're checking
        #self.get_logger().info(f"Checking if {self.parameter} is set to {self.value}.")
        if py_trees.blackboard.Blackboard.exists(self.parameter):
        # Access the blackboard and get the value of the parameter
            param_value = py_trees.blackboard.Blackboard.get(self.parameter)  # Fetch the value from the blackboard
            self.get_logger().info(f"{param_value}")

class DriveTo(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name):       
        # Initialize the py_trees.behaviour.Behaviour with the fixed name
        py_trees.behaviour.Behaviour.__init__(self,name)
        # Initialize the Node with the fixed name
        Node.__init__(self, name)

    def update(self):        
            self.get_logger().info(f"Driving To.")

class FollowPerson(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name):       
        # Initialize the py_trees.behaviour.Behaviour with the fixed name
        py_trees.behaviour.Behaviour.__init__(self,name)
        # Initialize the Node with the fixed name
        Node.__init__(self, name)

    def update(self):        
            self.get_logger().info(f"Following")

class SaveLocation(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name):       
        # Initialize the py_trees.behaviour.Behaviour with the fixed name
        py_trees.behaviour.Behaviour.__init__(self,name)
        # Initialize the Node with the fixed name
        Node.__init__(self, name)

    def update(self):        
            self.get_logger().info(f"Following")

class ChangePaused(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name):       
        # Initialize the py_trees.behaviour.Behaviour with the fixed name
        py_trees.behaviour.Behaviour.__init__(self,name)
        # Initialize the Node with the fixed name
        Node.__init__(self, name)

    def update(self):      
        pause = py_trees.blackboard.Blackboard.get("pause")
        if pause == True:
            py_trees.blackboard.Blackboard.set("pause",False)
            self.get_logger().info(f"Set Pause: {pause}")
            return py_trees.common.Status.RUNNING
        else:
            py_trees.blackboard.Blackboard.set("pause",True)
            self.get_logger().info(f"Set Pause: {pause}")
            return py_trees.common.Status.SUCCESS

class CheckBoard_Pause(py_trees.behaviour.Behaviour, Node):
    def __init__(self, name, parameter, value):       
        # Initialize the py_trees.behaviour.Behaviour with the fixed name
        py_trees.behaviour.Behaviour.__init__(self, name)
        # Initialize the Node with the fixed name
        Node.__init__(self, name)
        
        # Store the parameter and value to check on the blackboard
        self.parameter = parameter
        self.value = value

    def update(self):
        """Check if the parameter value matches the expected value on the BlackBoard"""
        if self.parameter is None or self.value is None:
            # Handle the case where no parameter or value was provided
            self.get_logger().info("No parameter or value set. Returning FAILURE.")
            return py_trees.common.Status.RUNNING
        
        # Log the information about what we're checking
        #self.get_logger().info(f"Checking if {self.parameter} is set to {self.value}.")
        if py_trees.blackboard.Blackboard.exists(self.parameter):
        # Access the blackboard and get the value of the parameter
            param_value = py_trees.blackboard.Blackboard.get(self.parameter)  # Fetch the value from the blackboard
        
            # Return SUCCESS if the value matches, otherwise FAILURE
            if param_value == self.value:
                self.get_logger().info(f"pausing.")
                return py_trees.common.Status.RUNNING
            else:
                self.get_logger().info(f"Running/Resuming")
                return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
def create_behavior_tree():
    root = py_trees.composites.Parallel("Root",policy=ParallelPolicy.SuccessOnOne(),children=None)
    notstopp_selector = py_trees.composites.Selector("Notstopp",False)
    notstoppSequence = py_trees.composites.Sequence("StoppSeq",False)
    hriSequence = py_trees.composites.Sequence("hriSelector",False)
    gesture_checkerStop = GestureChecker("GestureCheckerStop", "stop")
    #invert = py_trees.decorators.Inverter("Inverter1",gesture_checkerStop)
    stopping = Stopping("Stopping") 
    SetModeStopped = SetMode("Stopped")
    personRegistrationSelector = py_trees.composites.Selector("personRegistrationSelector",False)
    CheckPersonLoggedIn = CheckBoard("PersonLoggedIn","PersonLoggedIn",True)
    PersonRegistrationSequence = py_trees.composites.Sequence("PersonRegistrationSequence",False)
    isWaving = GestureChecker("GestureCheckerWaving","is waving")
    RegisterPerson = WriteBoard("RegisterPerson",True)
    acknowledgeHuman = AcknowledgeHuman()  
    isLearningGesture = GestureChecker("isLearningGesture","isLearningGesture")  

    maneuverSequence = py_trees.composites.Sequence("maneuverSequence",True)
    LoginSelector = py_trees.composites.Selector("LoginSelector",False)
    CheckMode_Maneuvering = CheckBoard("CheckManeuverMode","Mode","Maneuvering")
    LoginSequence = py_trees.composites.Selector("LoginSequence",False)
    setManeuveringMode = SetMode("Maneuvering")
    displayMode = display("DisplayMode","Mode")
    followPerson = FollowPerson("followPerson")
    isPointing = GestureChecker("IsPointing","isPointing")
    isThumbsUp = GestureChecker("isThumbsUp","Thumbs Up")
    driveTo = DriveTo("driveTo")
    saveLocation = SaveLocation("SaveLocation")
    isFollowingGesture = GestureChecker("isFollowingGesture","follow")

    followingSequence= py_trees.composites.Sequence("followingSequence",False,[isFollowingGesture,followPerson])
    
    DriveToAuto = DriveTo("DriveToAuto")
    setModeAuto = SetMode("autonomous")
    autoSequence_2 = py_trees.composites.Sequence("autoSequence_2",True,[DriveToAuto])
    repeatAuto = py_trees.decorators.Repeat("RepeatAuto",autoSequence_2,3)

    pointingSequence = py_trees.composites.Sequence("pointingSequence",False,[isPointing,driveTo])  
    ManeuverSelector_1 = py_trees.composites.Selector("ManeuverSelector_1",True,[pointingSequence,followingSequence])
    ManeuverSequence_1 = py_trees.composites.Sequence("ManeuverSequence_1",True,[ManeuverSelector_1, isThumbsUp, saveLocation])
    ManeuverRepeat_1 = py_trees.decorators.Repeat("ManeuverRepeat_1",ManeuverSequence_1,-1) 
    
    autoSequence= py_trees.composites.Sequence("autoSequence",True,[setModeAuto,repeatAuto])

    #Log Off Block
    isPersonLoggedIn_2 = CheckBoard("PersonLoggedIn_2","PersonLoggedIn",True)
    logOffPerson = WriteBoard("RegisterPerson_2",False)
    displayLogOff = display("RegisterPersonOff","RegisterPerson")
    isWaving_2 = GestureChecker("GestureCheckerWaving_2","is waving")
    logOffSequence = py_trees.composites.Sequence("logOffSequence",True,[isWaving_2,logOffPerson,displayLogOff])
    logOffSelector = py_trees.composites.Selector("logOffSelector",True,[isPersonLoggedIn_2,logOffSequence])
    
    checkPause = GestureChecker("GestureCheckerPause","pause/resume")
    changePaused = ChangePaused("IfPaused")
    ifPaused = CheckBoard_Pause("IsPaused","pause",True)
    pauseSequence = py_trees.composites.Sequence("PauseSeq",False,[checkPause,changePaused])

    PauseSequence = py_trees.composites.Sequence("PauseSelector",True,[ifPaused,hriSequence])

    root.add_children([notstopp_selector,PauseSequence])
    notstopp_selector.add_children([notstoppSequence, pauseSequence])
    notstoppSequence.add_children([gesture_checkerStop, stopping, SetModeStopped])
    hriSequence.add_children([personRegistrationSelector,maneuverSequence,logOffSelector,autoSequence])
    personRegistrationSelector.add_children([CheckPersonLoggedIn,PersonRegistrationSequence])
    PersonRegistrationSequence.add_children([isWaving,RegisterPerson,acknowledgeHuman])
    maneuverSequence.add_children([LoginSelector,ManeuverRepeat_1])
    LoginSelector.add_children([CheckMode_Maneuvering, LoginSequence])
    LoginSequence.add_children([isLearningGesture,setManeuveringMode,displayMode])

    return py_trees.trees.BehaviourTree(root)

def main():
    rclpy.init()
    tree = create_behavior_tree()
    blackboard = py_trees.blackboard.Blackboard()
    
    # Initiale Werte für den Test setzen
    blackboard.set("Mode", "Maneuvering")
    blackboard.set("PersonLoggedIn", True)
    blackboard.set("pause",True)

    executor = rclpy.executors.SingleThreadedExecutor()

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)  # ROS 2 Callback verarbeiten
            tree.tick()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        #tree.root.children[0].children[0].destroy_node()  # Zerstöre GestureChecker-Node
        rclpy.shutdown()

if __name__ == "__main__":
    main()
