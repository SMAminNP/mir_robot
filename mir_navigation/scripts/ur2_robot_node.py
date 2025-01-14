#!/usr/bin/env python3

import rospy
import rtde_control
import rtde_io
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

class UR1RobotNode:
    def __init__(self):
        rospy.init_node('ur2_robot_node')
        self.ip_address = rospy.get_param('~ur2_ip_address', '10.1.1.1')
        self.rtde_c = None
        self.rtde_io = None
        
        self.connect()
        
        self.move_sub = rospy.Subscriber('/ur2_move', Pose, self.move_callback)
        self.move_completed_pub = rospy.Publisher('/ur2_move_completed', Bool, queue_size=10)

    def connect(self):
        try:
            self.rtde_c = rtde_control.RTDEControlInterface(self.ip_address)
            self.rtde_io = rtde_io.RTDEIOInterface(self.ip_address)
            rospy.loginfo(f"Successfully connected to UR robot at {self.ip_address}")
        except Exception as e:
            rospy.logerr(f"Failed to connect to UR robot at {self.ip_address}. Error: {str(e)}")

    def move_callback(self, pose_msg):
        target_pose = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z,
                       pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z]
        success = self.rtde_c.moveL(target_pose, 0.3, 0.3)
        self.move_completed_pub.publish(Bool(success))

    def run(self):
        rospy.spin()

    def shutdown(self):
        if self.rtde_c:
            self.rtde_c.disconnect()
        if self.rtde_io:
            self.rtde_io.disconnect()

if __name__ == '__main__':
    ur_node = UR1RobotNode()
    try:
        ur_node.run()
    finally:
        ur_node.shutdown()