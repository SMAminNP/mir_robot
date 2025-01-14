#!/usr/bin/env python3

import rospy
import socket
from std_msgs.msg import Bool

class RobotiqGripperNode:
    def __init__(self):
        rospy.init_node('robotiq_gripper_node_2')
        self.address = rospy.get_param('~gripper_ip', '10.1.1.1')
        self.port = rospy.get_param('~gripper_port', 63352)
        self.s = None
        
        self.connect()
        
        self.close_sub = rospy.Subscriber('/gripper2_close', Bool, self.close_callback)
        self.open_sub = rospy.Subscriber('/gripper2_open', Bool, self.open_callback)
        self.action_completed_pub = rospy.Publisher('/gripper2_action_completed', Bool, queue_size=10)

    def connect(self):
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((self.address, self.port))
            rospy.loginfo(f"Connected to Robotiq gripper at {self.address}:{self.port}")
        except Exception as e:
            rospy.logerr(f"Failed to connect to Robotiq gripper. Error: {str(e)}")

    def close_callback(self, msg):
        if msg.data and self.s:
            self.s.sendall(b'SET POS 255\n')
            self.action_completed_pub.publish(Bool(True))

    def open_callback(self, msg):
        if msg.data and self.s:
            self.s.sendall(b'SET POS 0\n')
            self.action_completed_pub.publish(Bool(True))

    def run(self):
        rospy.spin()

    def shutdown(self):
        if self.s:
            self.s.close()

if __name__ == '__main__':
    gripper_node = RobotiqGripperNode()
    try:
        gripper_node.run()
    finally:
        gripper_node.shutdown()