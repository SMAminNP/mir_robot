#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

def main():
    rospy.init_node('bt_main_node')
    
    # Create a publisher to start the behavior tree
    start_pub = rospy.Publisher('/start_behavior_tree', Bool, queue_size=10)
    
    rospy.sleep(2)  # Wait for other nodes to initialize
    
    # Start the behavior tree
    start_pub.publish(Bool(True))
    
    rospy.spin()

if __name__ == '__main__':
    main()