#!/usr/bin/env python3
import rospy
import py_trees
import time
import requests
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
from py_trees.common import Status, ParallelPolicy
from requests.exceptions import RequestException

# OctoPrint configuration
OCTOPRINT_URL = "http://10.1.1.115"
API_KEY = "11BB9B7D64F449D89DC466F8DA050163"
PRINT_FILE = "Printer_test_final.bgcode"

class StartPrinting(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(StartPrinting, self).__init__(name)
        
    def update(self):
        try:
            headers = {"X-Api-Key": API_KEY}
            payload = {
                "command": "select",
                "print": True
            }
            
            response = requests.post(
                f"{OCTOPRINT_URL}/api/files/local/{PRINT_FILE}",
                headers=headers,
                json=payload,
                timeout=10
            )
            
            if response.status_code == 204:
                rospy.loginfo(f"Successfully started printing {PRINT_FILE}")
                return Status.SUCCESS
            else:
                rospy.logwarn(f"Failed to start print. Status code: {response.status_code}")
                return Status.FAILURE
                
        except RequestException as e:
            rospy.logerr(f"Error starting print: {e}")
            return Status.FAILURE
        
class MonitorPrinterStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(MonitorPrinterStatus, self).__init__(name)
        self.previous_status = False  # Changed initial state to False
        self.current_status = False   # Changed initial state to False
        self.received_status = False
        self.print_started = False
        self.subscriber = rospy.Subscriber("/octoprint_status", Bool, self.status_callback)

    def status_callback(self, msg):
        self.current_status = msg.data
        self.received_status = True
        rospy.loginfo(f"Monitor received printer status: {self.current_status}")

    def initialise(self):
        self.received_status = False
        self.print_started = False
        self.previous_status = False
        self.current_status = False

    def update(self):
        if not self.received_status:
            rospy.logwarn("Waiting for printer status update...")
            return Status.RUNNING
        
        # First time we detect printing has started
        if self.current_status and not self.print_started:
            rospy.loginfo("Print job has started")
            self.print_started = True
            self.previous_status = True
            return Status.RUNNING
        
        # Detect transition from printing to not printing
        if self.print_started and self.previous_status and not self.current_status:
            rospy.loginfo("Print job has completed")
            return Status.SUCCESS
            
        self.previous_status = self.current_status
        rospy.loginfo(f"Print in progress. Status: {self.current_status}")
        return Status.RUNNING

class CheckPrinterIdle(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckPrinterIdle, self).__init__(name)
        self.printer_status = False
        self.status_received = False
        
    def setup(self, **kwargs):
        self.status_sub = rospy.Subscriber("/octoprint_status", Bool, self.status_callback)
        return True
        
    def initialise(self):
        self.status_received = False
        
    def status_callback(self, msg):
        self.printer_status = msg.data
        self.status_received = True
        
    def update(self):
        if not self.status_received:
            rospy.logwarn("Waiting for printer status in CheckPrinterIdle...")
            return Status.RUNNING
            
        if not self.printer_status:  # Printer is idle
            rospy.loginfo("Printer is idle, ready to start new print")
            return Status.SUCCESS
        else:
            rospy.loginfo("Printer is busy, waiting...")
            return Status.RUNNING

class MoveUR1Robot(py_trees.behaviour.Behaviour):

    def __init__(self, name, target_pose, is_joint_move=False):
        super(MoveUR1Robot, self).__init__(name)
        self.target_pose = target_pose
        self.is_joint_move = is_joint_move
        self.move_pub = None
        self.move_completed = False

    def setup(self, **kwargs):
        self.move_pub = rospy.Publisher('/ur1_move', PoseStamped, queue_size=10)
        self.move_completed_sub = rospy.Subscriber('/ur1_move_completed', Bool, self.move_completed_callback)
        return True
    
    def initialise(self):
        self.move_completed = False

    def update(self):
        if not self.move_completed:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "base_link"
            if self.is_joint_move:
                pose_msg.pose.position.x = self.target_pose[0]  # Joint angle in radians
                pose_msg.pose.position.y = float(self.is_joint_move)  # Use as a flag
            else:
                pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = self.target_pose[:3]
                pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z = self.target_pose[3:]
            self.move_pub.publish(pose_msg)
            return Status.RUNNING
        return Status.SUCCESS
    
    def move_completed_callback(self, msg):
        self.move_completed = msg.data

        
class MoveUR2Robot(py_trees.behaviour.Behaviour):
    
    def __init__(self, name, target_pose):
        super(MoveUR2Robot, self).__init__(name)
        self.target_pose = target_pose
        self.move_pub = None
        self.move_completed = False
        
    def setup(self, **kwargs):
        self.move_pub = rospy.Publisher('/ur2_move', Pose, queue_size=10)
        self.move_completed_sub = rospy.Subscriber('/ur2_move_completed', Bool, self.move_completed_callback)
        return True
    
    def initialise(self):
        self.move_completed = False
        
    def update(self):
        if not self.move_completed:
            pose_msg = Pose()
            pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = self.target_pose[:3]
            pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z = self.target_pose[3:]
            self.move_pub.publish(pose_msg)
            return Status.RUNNING
        return Status.SUCCESS
        
    
    def move_completed_callback(self, msg):
        self.move_completed = msg.data

class MoveRobotiqGripper1(py_trees.behaviour.Behaviour):

    def __init__(self, name, action):
        super(MoveRobotiqGripper1, self).__init__(name)
        self.action = action
        self.action_pub = None
        self.action_completed = False

    def setup(self, **kwargs):
        self.action_pub = rospy.Publisher(f'/gripper1_{self.action}', Bool, queue_size=10)
        self.action_completed_sub = rospy.Subscriber('/gripper1_action_completed', Bool, self.action_completed_callback)
        return True
    
    def initialise(self):
        self.action_completed = False

    def update(self):
        if not self.action_completed:
            self.action_pub.publish(Bool(True))
            return Status.RUNNING
        return Status.SUCCESS
    
    def action_completed_callback(self, msg):
        self.action_completed = msg.data
        
class MoveRobotiqGripper2(py_trees.behaviour.Behaviour):

    def __init__(self, name, action):
        super(MoveRobotiqGripper2, self).__init__(name)
        self.action = action
        self.action_pub = None
        self.action_completed = False

    def setup(self, **kwargs):
        self.action_pub = rospy.Publisher(f'/gripper2_{self.action}', Bool, queue_size=10)
        self.action_completed_sub = rospy.Subscriber('/gripper2_action_completed', Bool, self.action_completed_callback)
        return True
    
    def initialise(self):
        self.action_completed = False

    def update(self):
        if not self.action_completed:
            self.action_pub.publish(Bool(True))
            return Status.RUNNING
        return Status.SUCCESS
    
    def action_completed_callback(self, msg):
        self.action_completed = msg.data

class MoveMiRPlatform(py_trees.behaviour.Behaviour):

    def __init__(self, name, target_pose):
        super(MoveMiRPlatform, self).__init__(name)
        self.target_pose = target_pose
        self.move_pub = None
        self.move_completed = False

    def setup(self, **kwargs):
        self.move_pub = rospy.Publisher('/mir_move', PoseStamped, queue_size=10)
        self.move_completed_sub = rospy.Subscriber('/mir_move_completed', Bool, self.move_completed_callback)
        return True
    
    def initialise(self):
        self.move_completed = False

    def update(self):
        if not self.move_completed:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = self.target_pose[:3]
            pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = self.target_pose[3:]
            self.move_pub.publish(pose_msg)
            return Status.RUNNING
        return Status.SUCCESS
    
    def move_completed_callback(self, msg):
        self.move_completed = msg.data
        
class Wait(py_trees.behaviour.Behaviour):
    def __init__(self, name, duration):
        super(Wait, self).__init__(name)
        self.duration = duration
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()
        self.logger.debug(f"{self.name} waiting for {self.duration} seconds")
    
    def update(self):

        if time.time() - self.start_time > self.duration:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

def create_behavior_tree():

    # Define target positions
    ur1_targets = {
        "home" : [0.22958702102514253, -0.041313538025361585, 0.24310929742481088, -2.9299779137121815, 0.9482999958205441, -0.03715276916024623], #home 
        "first": [0.45082194648099827, -0.14891547449810874, 0.49135928494618286, 0.32402151689402287, -3.116830991526865, -0.1403629509630705], #Initial 
        "second": [0.657394989512138, -0.025193348489895687, 0.43176997545892987, 0.34441182344814153, -3.1226292206362465, -7.580410902757823e-05], #Down to Box  
        "third": [0.3370739153656716, -0.11916089871809299, 0.46186173174113987, 0.43940035162970636, -3.10775568029715, -0.050150417476322685], #Pick up the Box
        "fourth": [0.29834116046415643, -0.0032290908413096175, 0.29906822896152396, -2.3116526259882404, 2.1273790821964256, 2.0906522782249917e-05],#Back to Initial2
        "fifth": [-120 * 3.14159 / 180], #Over the MIR
        "sixth": [-0.42596107344515266, -0.40774926820278107, -0.4356262886381147, -2.8811715679781535, -1.2522633038358988, -5.14809440778677e-05], #Down to MIR
        "seventh":[-0.31241856356446096, -0.15461521869323255, 0.17097355544896606, 2.7953950018436795, 1.2713082378440095, 0.0019174952382227492], #Back to above MIR
        "eight": [120 * 3.14159 / 180] #Back to Initial 
    }
    mir_targets = {
        #"move_away": [35.704, 18.161, 0, 0, 0, 0.995, 0.102], #move away
        #"pos1": [15.759, 29.876, 0, 0, 0, -0.968, 0.252], #position 1 (start of path)
        "pos2": [34.112, 17.988, 0, 0, 0, 1.0, 0.0], #position 2 (in path)
        "pos3": [32.300, 16.500, 0, 0, 0, 1.0, 0.0], #position 3 (under UR1)
        "pose4": [27.700, 16.500, 0, 0, 0, 1.00, 0.0] #Position 4 (under UR2)
    }
    ur2_targets = {
        "home":[-0.09123437179469256, -0.3340628030878293, 0.4379345173609227, -2.9156587255666455, -1.0458002420512476, -0.23187279648229142], # Home
        "first":[0.3521667960509567, -0.9739033722000325, 0.24314727825098567, 2.8391449908411253, -1.1866521834095631, 0.06692984646793186], #Top of the Mir
        "second":[0.4394192447141492, -1.0149411049425174, -0.38849932385741853, 2.893093874830746, -1.2244760968520718, -1.645808625665849e-05], #Down to MiR
        "third":[0.3521667960509567, -0.9739033722000325, 0.24314727825098567, 2.8391449908411253, -1.1866521834095631, 0.06692984646793186], # Back up Again
        "fourth":[-0.1614216536936953, -0.5131600722024492, 0.3833848516306582, -1.1137651313583654, 2.9066581096072697, -0.0012972339832676275], # in the Path
        "final" : [-0.30325983458051475, -0.5252695073864808, 0.16476438062050236, 1.1342507525779086, -2.9295899224154693, 0.00015116074964716022] # Destination
    }

    # Create behavior tree
    root = py_trees.composites.Sequence("Root", memory=True)
    
 
    # Create the main sequence that will be repeated
    main_sequence = py_trees.composites.Sequence("Opration Sequence", memory=True)
    
    # Monitor print completion sequence
    print_completion = py_trees.composites.Sequence("Print Completion", memory=True)
    print_completion.add_children([
        MonitorPrinterStatus("Wait for Print Completion"),
        Wait("Cooling Delay", 2)  # Allow print to cool before pickup
    ])
    
    # Create the robot operation sequence
    robot_sequence = py_trees.composites.Sequence("Robot Operations", memory=True)

    # Initial parallel movement
    parallel_initial = py_trees.composites.Parallel("Initial Parallel Movement", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    ur_initial_sequence = py_trees.composites.Sequence("UR Initial Sequence", memory=True)
    ur_initial_sequence.add_children([
        MoveUR1Robot("UR1 to Home", ur1_targets["home"]),
        MoveUR1Robot("UR1 to First", ur1_targets["first"]),
        MoveUR1Robot("UR1 to Second", ur1_targets["second"])
    ])
    
    mir_initial_sequence = py_trees.composites.Sequence("MiR Initial Sequence", memory=True)
    mir_initial_sequence.add_children([
        #MoveMiRPlatform("MiR to Home", mir_targets["home"]),
        #MoveMiRPlatform("MiR to Pos1", mir_targets["pos1"]),
        MoveMiRPlatform("MiR to Pos2", mir_targets["pos2"]),
        MoveMiRPlatform("MiR to Pos3", mir_targets["pos3"])
    ])
    
    parallel_initial.add_children([ur_initial_sequence, mir_initial_sequence])

    #Gripper1 Close
    gripper1_close = py_trees.composites.Sequence("Gripper Closing", memory=True)
    gripper1_close.add_children([
        Wait(name="Wait", duration=3),
        MoveRobotiqGripper1("Close Gripper", "close"),
        Wait(name="Wait", duration=3)
    ])

    #Gripper1 Open
    gripper1_open = py_trees.composites.Sequence("Gripper Opening", memory=True)
    gripper1_open.add_children([
        Wait(name="Wait", duration=10),
        MoveRobotiqGripper1("Open Gripper", "open"),
        Wait(name="Wait", duration=3)
    ])
    
    # Intermediate sequence1
    intermediate_sequence1 = py_trees.composites.Sequence("Intermediate Sequence", memory=True)
    intermediate_sequence1.add_children([
        MoveUR1Robot("UR1 to Third", ur1_targets["third"]),
        MoveUR1Robot("UR1 to fourth", ur1_targets["fourth"]),
        #Wait(name="Wait", duration=3),
        MoveUR1Robot("UR1 to Fifth", ur1_targets["fifth"], is_joint_move=True),
        #Wait(name="Wait", duration=3),
        MoveUR1Robot("UR1 to Sixth", ur1_targets["sixth"])
    ])
    
    # intermediate_sequence2
    intermediate_sequence2 = py_trees.composites.Sequence("intermediate_sequence2", memory=True)
    intermediate_sequence2. add_children([
        MoveUR1Robot("UR1 to Seventh", ur1_targets["seventh"]),
        MoveUR1Robot("UR1 to Seventh", ur1_targets["eight"], is_joint_move=True),
        MoveMiRPlatform("MiR Move Pos4", mir_targets["pose4"])
    ])
    
    #UR2_sequence
    UR2_sequence = py_trees.composites.Sequence("UR2_sequence", memory=True)
    UR2_sequence.add_children([
        MoveUR2Robot("UR2 to Home", ur2_targets["home"]),
        MoveUR2Robot("UR2 to First", ur2_targets["first"]),
        MoveUR2Robot("UR2 to Second", ur2_targets["second"])    
    ])
    
    #Gripper2 Close
    gripper2_close = py_trees.composites.Sequence("Gripper2 Closing", memory=True)
    gripper2_close.add_children([
        Wait(name="Wait", duration=7),
        MoveRobotiqGripper2("Close Gripper", "close"),
        Wait(name="Wait", duration=3)
    ])
    
    #final_sequence
    final_sequence = py_trees.composites.Sequence("final Sequence", memory=True)
    final_sequence.add_children([
        MoveUR2Robot("UR2 to first", ur2_targets["first"]),
        MoveUR2Robot("UR2 to final", ur2_targets["final"]),
        MoveMiRPlatform("MiR Move Away", mir_targets["pos2"])   
    ])

    #Gripper2 Open
    gripper2_open = py_trees.composites.Sequence("Gripper2 Opening", memory=True)
    gripper2_open.add_children([
        Wait(name="Wait", duration=6),
        MoveRobotiqGripper2("Open Gripper", "open")
        ])
    
    robot_sequence.add_children([
        parallel_initial,
        gripper1_close,
        intermediate_sequence1,
        gripper1_open,
        intermediate_sequence2,
        UR2_sequence,
        gripper2_close,
        final_sequence,
        gripper2_open
        #Wait("Post_Opration Delay", 40)
    ])
    
    # Start new print sequence
    start_new_print = py_trees.composites.Sequence("Start New Print", memory=True)
    start_new_print.add_children([
        CheckPrinterIdle("Check Printer Idle"),
        StartPrinting("Start New Print"),
        Wait("Print Start Delay", 5)
    ])
    
    # Add sequences to main sequence in strict order
    main_sequence.add_children([
        print_completion,     # First wait for current print to finish
        robot_sequence,       # Then do robot operations with delay
        start_new_print      # Finally start new print
    ])
    
    # Create an always-running decorator
    repeater = py_trees.decorators.FailureIsRunning(
        name="MainLoopRunner",
        child=main_sequence
    )
            
    # Add the repeating sequence to the root
    root.add_children([repeater])
    return root

class BehaviorTreeNode:
    def __init__(self):
        rospy.init_node('behavior_tree_node')
        self.tree = py_trees.trees.BehaviourTree(create_behavior_tree())
        self.tree.setup(timeout=15)
        
    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.tree.tick()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = BehaviorTreeNode()
        node.run()
    except rospy.ROSInterruptException:
        pass