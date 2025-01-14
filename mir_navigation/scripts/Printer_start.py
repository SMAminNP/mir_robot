#!/usr/bin/env python3

import rospy
import requests
from std_msgs.msg import Bool
import time
from requests.exceptions import RequestException

# OctoPrint API configuration
OCTOPRINT_URL = "http://10.1.1.115"
API_KEY = "11BB9B7D64F449D89DC466F8DA050163"

def start_print(filename, location="local"):
    """Start printing a file"""
    headers = {"X-Api-Key": API_KEY}
    payload = {
        "command": "select",
        "print": True
    }
    
    rospy.loginfo(f"Attempting to start print of {filename} from {location}")
    try:
        response = requests.post(
            f"{OCTOPRINT_URL}/api/files/{location}/{filename}",
            headers=headers,
            json=payload,
            timeout=10
        )
        
        if response.status_code == 204:
            rospy.loginfo(f"Successfully started printing {filename}")
            return True
        else:
            rospy.logwarn(f"Failed to start print. Status code: {response.status_code}")
            if response.text:
                rospy.logwarn(f"Response text: {response.text}")
            return False
            
    except RequestException as e:
        rospy.logerr(f"Error starting print: {e}")
        return False

def main():
    rospy.init_node('octoprint_print_controller', anonymous=True)
    
    filename = "Test2_0.4n_0.2mm_PLA_XLIS_1h25m.bgcode"
    if start_print(filename):
        rospy.loginfo("Print job started successfully")
    else:
        rospy.logerr("Failed to start print job")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass