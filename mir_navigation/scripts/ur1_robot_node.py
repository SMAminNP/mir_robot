#!/usr/bin/env python3

import rospy
import rtde_control
import rtde_io
import rtde_receive
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import time
import math

class UR1RobotNode:
    def __init__(self):
        rospy.init_node('ur1_robot_node')
        self.ip_address = rospy.get_param('~ur1_ip_address', '10.1.1.3')
        self.rtde_c = None
        self.rtde_io = None
        self.rtde_r = None
        
        self.connect_with_retry()
        
        self.move_sub = rospy.Subscriber('/ur1_move', PoseStamped, self.move_callback)
        self.move_completed_pub = rospy.Publisher('/ur1_move_completed', Bool, queue_size=10)

    def connect_with_retry(self, max_attempts=5, retry_delay=5):
        for attempt in range(max_attempts):
            try:
                self.rtde_c = rtde_control.RTDEControlInterface(self.ip_address)
                self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ip_address)
                self.rtde_io = rtde_io.RTDEIOInterface(self.ip_address)
                rospy.loginfo(f"Successfully connected to UR robot at {self.ip_address}")
                return True
            except Exception as e:
                rospy.logerr(f"Attempt {attempt + 1}/{max_attempts}: Failed to connect to UR robot at {self.ip_address}. Error: {str(e)}")
                if attempt < max_attempts - 1:
                    rospy.loginfo(f"Retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
        
        rospy.logerr("Failed to connect after maximum attempts. Please check the robot and network connection.")
        return False

    def move_callback(self, pose_msg):
        if not self.rtde_c:
            rospy.logerr("RTDE Control interface is not connected. Attempting to reconnect...")
            if not self.connect_with_retry():
                self.move_completed_pub.publish(Bool(False))
                return
            
        try:
            if pose_msg.pose.position.y == 1.0:  # This is our flag for joint movement
                joint_angle = pose_msg.pose.position.x
                current_joints = self.rtde_r.getActualQ()
                target_joints = current_joints[:]
                target_joints[0] = joint_angle  # Modify only the first joint
                success = self.rtde_c.moveJ(target_joints, 0.5, 0.5)
            else:
                target_pose = [pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z,
                               pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z]
                success = self.rtde_c.moveL(target_pose, 0.2, 0.2)

            self.move_completed_pub.publish(Bool(success))
            if success:
                rospy.loginfo("Move completed successfully")
            else:
                rospy.logwarn("Move command failed")
        except Exception as e:
            rospy.logerr(f"Error during move command: {str(e)}")
            self.move_completed_pub.publish(Bool(False))

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if not self.rtde_c or not self.rtde_c.isConnected():
                rospy.logwarn("RTDE Control interface disconnected. Attempting to reconnect...")
                self.connect_with_retry()
            rate.sleep()

    def shutdown(self):
        if self.rtde_c:
            self.rtde_c.disconnect()
        if self.rtde_io:
            self.rtde_io.disconnect()
        rospy.loginfo("UR1 Robot Node shut down successfully")

if __name__ == '__main__':
    ur_node = UR1RobotNode()
    try:
        ur_node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        ur_node.shutdown()