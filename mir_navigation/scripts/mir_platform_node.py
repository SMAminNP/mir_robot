#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool

class MiRPlatformNode:
    def __init__(self):
        rospy.init_node('mir_platform_node')
        self.move_base_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.status_sub = None
        self.move_sub = rospy.Subscriber('/mir_move', PoseStamped, self.move_callback)
        self.move_completed_pub = rospy.Publisher('/mir_move_completed', Bool, queue_size=1)
        self.goal_reached = False
        self.timer = None
        self.current_goal = None
        self.is_moving = False

    def move_callback(self, pose_msg):
        if self.is_moving:
            rospy.logwarn("MiR Platform: Ignoring new goal, robot is already moving")
            return

        self.current_goal = pose_msg
        self.goal_reached = False
        self.is_moving = True
        self.move_base_pub.publish(pose_msg)
        
        if self.timer:
            self.timer.shutdown()
        self.timer = rospy.Timer(rospy.Duration(8), self.start_status_check, oneshot=True)
        rospy.loginfo("MiR Platform: Goal published, status check will start in 8 seconds")

    def start_status_check(self, event):
        if self.status_sub is None:
            self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        rospy.loginfo("MiR Platform: Status check started")

    def status_callback(self, status_msg):
        if not self.is_moving:
            return

        for status in status_msg.status_list:
            if status.status == 3:  # 3 means 'SUCCEEDED'
                self.goal_reached = True
                self.move_completed_pub.publish(Bool(True))
                if self.status_sub:
                    self.status_sub.unregister()
                    self.status_sub = None
                rospy.loginfo("MiR Platform: Goal reached")
                self.is_moving = False
                break
            elif status.status in [4, 5, 9]:  # 4: ABORTED, 5: REJECTED, 9: LOST
                rospy.logwarn(f"MiR Platform: Goal failed with status {status.status}")
                self.move_completed_pub.publish(Bool(False))
                self.is_moving = False
                break

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        mir_node = MiRPlatformNode()
        mir_node.run()
    except rospy.ROSInterruptException:
        pass