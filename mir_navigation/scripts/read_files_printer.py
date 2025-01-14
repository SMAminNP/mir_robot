#!/usr/bin/env python3

import rospy
import requests
from std_msgs.msg import Bool
import time
from requests.exceptions import RequestException

# OctoPrint API configuration
OCTOPRINT_URL = "http://10.1.1.115"
API_KEY = "11BB9B7D64F449D89DC466F8DA050163"

def list_all_files():
    """List files from both local and SD card storage"""
    headers = {"X-Api-Key": API_KEY}
    
    # First, try to get all files
    try:
        rospy.loginfo("Trying to list ALL files...")
        response = requests.get(
            f"{OCTOPRINT_URL}/api/files",
            headers=headers,
            timeout=10
        )
        
        if response.status_code == 200:
            rospy.loginfo("All files found:")
            files = response.json()
            rospy.loginfo(f"Response: {files}")
        else:
            rospy.logwarn(f"Failed to list all files, HTTP status code: {response.status_code}")
    
        # Now try local storage specifically
        rospy.loginfo("\nTrying to list local files...")
        response = requests.get(
            f"{OCTOPRINT_URL}/api/files/local",
            headers=headers,
            timeout=10
        )
        
        if response.status_code == 200:
            rospy.loginfo("Local files found:")
            files = response.json()
            rospy.loginfo(f"Response: {files}")
        else:
            rospy.logwarn(f"Failed to list local files, HTTP status code: {response.status_code}")

    except RequestException as e:
        rospy.logerr(f"Error connecting to OctoPrint: {e}")

def main():
    rospy.init_node('octoprint_print_controller', anonymous=True)
    list_all_files()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass