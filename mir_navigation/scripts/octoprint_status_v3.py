#!/usr/bin/env python3

import rospy
import requests
from std_msgs.msg import Bool
import time
from requests.exceptions import RequestException

# OctoPrint API configuration
OCTOPRINT_URL = "http://10.1.1.115"  # Replace with your OctoPrint IP
API_KEY = "11BB9B7D64F449D89DC466F8DA050163"  # Replace with your OctoPrint API key

MAX_RETRIES = 5
INITIAL_RETRY_DELAY = 1  # seconds

def get_printer_status_with_retry():
    """Query the OctoPrint API for the printer status with retry mechanism."""
    headers = {"X-Api-Key": API_KEY}
    retry_delay = INITIAL_RETRY_DELAY

    for attempt in range(MAX_RETRIES):
        try:
            response = requests.get(f"{OCTOPRINT_URL}/api/job", headers=headers, timeout=10)
            if response.status_code == 200:
                data = response.json()
                state_info = data.get('state', '')
                is_printing = "Operational" not in state_info
                rospy.loginfo(f"Printer status: {'Printing' if is_printing else 'Not Printing'}")
                return is_printing
            else:
                rospy.logwarn(f"Failed to retrieve printer status, HTTP status code: {response.status_code}")
        except RequestException as e:
            rospy.logerr(f"Error connecting to OctoPrint (Attempt {attempt + 1}/{MAX_RETRIES}): {e}")
        
        if attempt < MAX_RETRIES - 1:  # Don't sleep after the last attempt
            rospy.loginfo(f"Retrying in {retry_delay} seconds...")
            time.sleep(retry_delay)
            retry_delay *= 2  # Exponential backoff

    rospy.logerr("Max retries reached. Unable to connect to OctoPrint.")
    return False

def octoprint_status_publisher():
    """ROS node that publishes the printer status to a ROS topic."""
    rospy.init_node('octoprint_status_node', anonymous=True)
    status_pub = rospy.Publisher('/octoprint_status', Bool, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz polling rate

    while not rospy.is_shutdown():
        current_status = get_printer_status_with_retry()
        status_pub.publish(bool(current_status))
        rate.sleep()

if __name__ == '__main__':
    try:
        octoprint_status_publisher()
    except rospy.ROSInterruptException:
        pass