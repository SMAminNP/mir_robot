#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from actionlib_msgs.msg import GoalStatusArray  
from mir_msgs.msg import GoalReached 

class MiRGoalManager:
    def __init__(self):
        rospy.init_node('send_goal', anonymous=True)
        
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.goal_reached_publisher = rospy.Publisher('/mir_goal_reached', GoalReached, queue_size=10)
        
        self.status_subscriber = None  
        self.position_subscriber = rospy.Subscriber('/mir/robot_pose', Pose, self.position_callback)
        
        self.goal_position = None
        self.current_position = None
        self.goal_reached_flag = False

    def send_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = 6.000      #6.986 
        goal.pose.position.y = 29.000     #28.874 
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 1.000     #0.661
        goal.pose.orientation.w = 0.0      #0.750

        self.goal_position = goal.pose

        rospy.loginfo("Sending goal")
        self.goal_publisher.publish(goal)
        rospy.loginfo("Goal sent")
        
        rospy.Timer(rospy.Duration(8), self.start_status_check, oneshot=True)

    def start_status_check(self, event):
        self.status_subscriber = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)

    def status_callback(self, data):
        for status in data.status_list:
            if status.status == 3 and not self.goal_reached_flag: 
                rospy.loginfo("MiR has reached its goal.")
                self.goal_reached_flag = True
                goal_reached_msg = GoalReached()
                goal_reached_msg.goal_reached = True
                #goal_reached_msg.position = self.current_position
                self.goal_reached_publisher.publish(goal_reached_msg)

    def position_callback(self, data):
        self.current_position = data

if __name__ == '__main__':
    try:
        mir_goal_manager = MiRGoalManager()
        rospy.sleep(1)  
        mir_goal_manager.send_goal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass