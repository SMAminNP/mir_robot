#!/usr/bin/env python3

import rospy
import requests
from std_msgs.msg import String

OCTOPRINT_URL = "http://10.1.1.115/"
API_KEY = "11BB9B7D64F449D89DC466F8DA050163"
HEADERS = {"X-Api-Key": API_KEY}

def get_printer_status():
    """Query OctoPrint API for printer status"""
    try:
        response = requests.get(f"{OCTOPRINT_URL}/api/printer", headers=HEADERS)
        if response.status_code == 200:
            return response.json()
        else:
            rospy.logwarn("Failed to get status from OctoPrint API")
            return None
    except requests.exceptions.RequestException as e:
        rospy.logwarn(f"Error communication with OctoPrint: {e}")
        return None
    
def octoprint_status_publisher():
    """ROS node to publish OctoPrint printer status"""
    # Initialize ROS node
    rospy.init_node('octoprint_status_publisher', anonymous=True)
    # Create a ROS publisher on the 'octoprint_status' topic
    status_pub = rospy.Publisher('octoprint_status', String, queue_size=10)
    # Set loop rate (e.g., 1 Hz)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # Get the printer status from OctoPrint
        printer_status = get_printer_status()
        if printer_status:
            # Convert the status to a string and publish it
            status_pub.publish(str(printer_status))
        # Sleep to maintain the loop rate
        rate.sleep()
 
if __name__ == '__main__':
    try:
        octoprint_status_publisher()
    except rospy.ROSInterruptException:
        pass