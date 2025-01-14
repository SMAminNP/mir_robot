#!/usr/bin/env python3
import rospy
import py_trees
import time
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
from py_trees.common import Status, ParallelPolicy

class MoveUR1Robot(py_trees.behaviour.Behaviour):
    def __init__(self, name, target_pose, is_joint_move=False, use_force=False):
        super(MoveUR1Robot, self).__init__(name)
        self.target_pose = target_pose
        self.is_joint_move = is_joint_move
        self.use_force = use_force
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
            
            if self.use_force:
                # Special flag for force move
                pose_msg.pose.position.y = 2.0
            elif self.is_joint_move:
                pose_msg.pose.position.x = self.target_pose[0]
                pose_msg.pose.position.y = 1.0  # Joint move flag
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

class MonitorPrinterStatus(py_trees.behaviour.Behaviour):

    def __init__(self, name):
        super(MonitorPrinterStatus, self).__init__(name)
        self.previous_status = True
        self.current_status = True
        self.recieved_status = False
        self.subscriber = rospy.Subscriber("/octoprint_status", Bool, self.status_callback)

    def status_callback(self, msg):
        """Callback function to update the current printer status from the topic"""
        self.current_status = msg.data
        self.recieved_status = True
        rospy.loginfo(f"Recieved printer status: {self.current_status}")

    def initialise(self):
        self.recieved_status = False

    def update(self):

        if not self.recieved_status:
            rospy.logwarn("Waiting for the printer status update...")
            return py_trees.common.Status.RUNNING #running
        
        if self.previous_status and not self.current_status:
            rospy.loginfo("Printer status changed from printing to not printing")
            self.previous_status = self.current_status
            return py_trees.common.Status.SUCCESS
        else: 
            rospy.loginfo(f"Printer is still printing: {self.current_status}")
            self.previous_status = self.current_status
            return py_trees.common.Status.RUNNING #running
        
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
        "second": [0.6797201961503877, -0.03721008731275484, 0.3859765506061469, 0.5268792252627897, -3.0125888388371784, -0.11218252833676023], #Down to Box  
        "third": [0.3370739153656716, -0.11916089871809299, 0.46186173174113987, 0.43940035162970636, -3.10775568029715, -0.050150417476322685], #Pick up the Box
        "fourth": [0.29834116046415643, -0.0032290908413096175, 0.29906822896152396, -2.3116526259882404, 2.1273790821964256, 2.0906522782249917e-05],#Back to Initial2
        "fifth": [-120 * 3.14159 / 180], #Over the MIR
        "sixth": [-0.6221708047084094, -0.494671613357575, -0.4283739279418515, 2.963453865041609, 1.0427104884285991, -9.215188677219501e-05], #Down to MIR
        "seventh": [-0.12271670275696835, -0.4616131700483915, 0.3745133076862287, 1.0432596791864046, 2.963311054987053, -6.793681830900753e-05], #Back to above MIR
        "eight": [0.08998894980123119, -0.5729706890326716, 0.7708751079795744, -1.1894344742346799, 2.9076961538819317, 3.454532851916363e-05] #Back to Initial 
    }
    
    mir_targets = {
        "move_away": [35.704, 18.161, 0, 0, 0, 0.995, 0.102], #move away
        "pos1": [15.759, 29.876, 0, 0, 0, -0.968, 0.252], #position 1 (start of path)
        "pos2": [33.098, 16.045, 0, 0, 0, 0.993, 0.117], #position 2 (in path)
        "pos3": [31.032, 15.080, 0, 0, 0, 0.994, 0.106], #position 3 (under UR1)
        "pose4": [26.598, 15.837, 0, 0, 0, 0.993, 0.122] #Position 4 (under UR2)
    }
    
    ur2_targets = {
        "home":[-0.09123437179469256, -0.3340628030878293, 0.4379345173609227, -2.9156587255666455, -1.0458002420512476, -0.23187279648229142], # Home
        "first":[0.3521667960509567, -0.9739033722000325, 0.24314727825098567, 2.8391449908411253, -1.1866521834095631, 0.06692984646793186], #Top of the Mir
        "second":[0.33906824598065677, -0.9898202302486906, -0.19240945661498285, 2.8362949530355532, -1.3508641406202209, 0.00011491149426883273], #Down to MiR
        "third":[0.3521667960509567, -0.9739033722000325, 0.24314727825098567, 2.8391449908411253, -1.1866521834095631, 0.06692984646793186], # Back up Again
        "fourth":[-0.1614216536936953, -0.5131600722024492, 0.3833848516306582, -1.1137651313583654, 2.9066581096072697, -0.0012972339832676275], # in the Path
        "final" : [-0.30325983458051475, -0.5252695073864808, 0.16476438062050236, 1.1342507525779086, -2.9295899224154693, 0.00015116074964716022] # Destination
    }

    # Create behavior tree
    root = py_trees.composites.Sequence("Main Sequence", memory=True)
 
    # Printer status monitor
    monitor_printer_status = MonitorPrinterStatus(name="Monitor Printer Status")

    # Initial parallel movement
    parallel_initial = py_trees.composites.Parallel("Initial Parallel Movement", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    
    # UR1 Initial Sequence
    ur_initial_sequence = py_trees.composites.Sequence("UR Initial Sequence", memory=True)
    ur_initial_sequence.add_children([
        MoveUR1Robot("UR1 to Home", ur1_targets["home"]),
        MoveUR1Robot("UR1 to First", ur1_targets["first"]),
        MoveUR1Robot("UR1 to Second", ur1_targets["second"])
    ])
    
    # MiR Initial Sequence
    mir_initial_sequence = py_trees.composites.Sequence("MiR Initial Sequence", memory=True)
    mir_initial_sequence.add_children([
        MoveMiRPlatform("MiR to Pos2", mir_targets["pos2"]),
        MoveMiRPlatform("MiR to Pos3", mir_targets["pos3"])
    ])
    
    parallel_initial.add_children([ur_initial_sequence, mir_initial_sequence])

    # Modified pickup sequence with force control
    pickup_sequence = py_trees.composites.Sequence("Pickup Sequence", memory=True)
    pickup_sequence.add_children([
        Wait(name="Pre-Gripper Wait", duration=3),
        MoveRobotiqGripper1("Close Gripper", "close"),
        Wait(name="Post-Gripper Wait", duration=3),
        MoveUR1Robot("UR1 Force Move", ur1_targets["second"], use_force=True),  # Apply 50N Upward force
        Wait(name="Force Application Wait", duration=2),
        MoveUR1Robot("UR1 to Third", ur1_targets["third"])
    ])

    # Intermediate sequence1
    intermediate_sequence1 = py_trees.composites.Sequence("Intermediate Sequence", memory=True)
    intermediate_sequence1.add_children([
        MoveUR1Robot("UR1 to fourth", ur1_targets["fourth"]),
        MoveUR1Robot("UR1 to Fifth", ur1_targets["fifth"], is_joint_move=True),
        MoveUR1Robot("UR1 to Sixth", ur1_targets["sixth"])
    ])
    
    # Gripper1 Open
    gripper1_open = py_trees.composites.Sequence("Gripper Opening", memory=True)
    gripper1_open.add_children([
        Wait(name="Pre-Open Wait", duration=5),
        MoveRobotiqGripper1("Open Gripper", "open"),
        Wait(name="Post-Open Wait", duration=3)
    ])
    
    # Intermediate sequence2
    intermediate_sequence2 = py_trees.composites.Sequence("intermediate_sequence2", memory=True)
    intermediate_sequence2.add_children([
        MoveUR1Robot("UR1 to Seventh", ur1_targets["seventh"]),
        MoveUR1Robot("UR1 to Eight", ur1_targets["eight"]),
        MoveMiRPlatform("MiR Move Pos4", mir_targets["pose4"])
    ])
    
    # UR2 sequence
    UR2_sequence = py_trees.composites.Sequence("UR2_sequence", memory=True)
    UR2_sequence.add_children([
        MoveUR2Robot("UR2 to Home", ur2_targets["home"]),
        MoveUR2Robot("UR2 to First", ur2_targets["first"]),
        MoveUR2Robot("UR2 to Second", ur2_targets["second"])    
    ])
    
    # Gripper2 Close
    gripper2_close = py_trees.composites.Sequence("Gripper2 Closing", memory=True)
    gripper2_close.add_children([
        Wait(name="Pre-Close Wait", duration=7),
        MoveRobotiqGripper2("Close Gripper", "close"),
        Wait(name="Post-Close Wait", duration=3)
    ])
    
    # Final sequence
    final_sequence = py_trees.composites.Sequence("final Sequence", memory=True)
    final_sequence.add_children([
        MoveUR2Robot("UR2 to first", ur2_targets["first"]),
        MoveUR2Robot("UR2 to final", ur2_targets["final"])   
    ])

    # Gripper2 Open
    gripper2_open = py_trees.composites.Sequence("Gripper2 Opening", memory=True)
    gripper2_open.add_children([
        Wait(name="Pre-Open Wait", duration=6),
        MoveRobotiqGripper2("Open Gripper", "open")
    ])

    # Add all sequences to the root
    root.add_children([
        monitor_printer_status,
        parallel_initial,
        pickup_sequence,        # Modified pickup sequence with force control
        intermediate_sequence1,
        gripper1_open,
        intermediate_sequence2,
        UR2_sequence,
        gripper2_close,
        final_sequence,
        gripper2_open
    ])
    
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