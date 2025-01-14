#!/usr/bin/env python3

import rospy
import requests
from std_msgs.msg import Bool
import time
from requests.exceptions import RequestException
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

# OctoPrint API configuration
OCTOPRINT_URL = "http://10.1.1.115"  # Replace with your OctoPrint IP
API_KEY = "11BB9B7D64F449D89DC466F8DA050163"  # Replace with your OctoPrint API key

MAX_RETRIES = 5
INITIAL_RETRY_DELAY = 1  # seconds

def create_session():
    session = requests.Session()
    retry = Retry(total=MAX_RETRIES, backoff_factor=0.1)
    adapter = HTTPAdapter(max_retries=retry)
    session.mount('http://', adapter)
    session.mount('https://', adapter)
    return session

def get_printer_status(session):
    """Query the OctoPrint API for the printer status."""
    headers = {"X-Api-Key": API_KEY}
    try:
        start_time = time.time()
        response = session.get(f"{OCTOPRINT_URL}/api/job", headers=headers, timeout=10)
        end_time = time.time()
        
        rospy.loginfo(f"Request took {end_time - start_time:.2f} seconds")
        rospy.loginfo(f"Response status code: {response.status_code}")
        rospy.loginfo(f"Response headers: {response.headers}")
        
        if response.status_code == 200:
            data = response.json()
            rospy.loginfo(f"Response data: {data}")
            state_info = data.get('state', '')
            is_printing = "Operational" not in state_info
            rospy.loginfo(f"Printer status: {'Printing' if is_printing else 'Not Printing'}")
            return is_printing
        else:
            rospy.logwarn(f"Failed to retrieve printer status, HTTP status code: {response.status_code}")
    except RequestException as e:
        rospy.logerr(f"Error connecting to OctoPrint: {e}")
    
    return False

def octoprint_status_publisher():
    """ROS node that publishes the printer status to a ROS topic."""
    rospy.init_node('octoprint_status_node', anonymous=True)
    status_pub = rospy.Publisher('/octoprint_status', Bool, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz polling rate
    session = create_session()

    while not rospy.is_shutdown():
        current_status = get_printer_status(session)
        status_pub.publish(bool(current_status))
        rate.sleep()

if __name__ == '__main__':
    try:
        octoprint_status_publisher()
    except rospy.ROSInterruptException:
        pass