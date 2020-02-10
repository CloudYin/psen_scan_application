#!/usr/bin/env python

from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
import rospy
import math


pause_trigger = rospy.ServiceProxy('move_path_B', Trigger)
resume_trigger = rospy.ServiceProxy('continue_move_path_A', Trigger)

def laser_scan_callback(msg):
    # In angle between 0 to 90 degree
    # If distance < 0.5m, call "move_path_B" service
    # Else, call "continue_move_path_A" service
    if min(msg.ranges) < 0.5:
        try:
            pause_trigger_response = pause_trigger()
            print("move path B trigger: " + str(pause_trigger_response.message))
        except rospy.ServiceException:
            print("Service move_path_B call failed") 
    elif min(msg.ranges) >= 1:
        try:
            resume_trigger_response = resume_trigger()
            print("move path A trigger: " + str(resume_trigger_response.message))
        except rospy.ServiceException:
            print("Service continue_move_path_A call failed")


if __name__ == "__main__":
    rospy.init_node('psen_scan_app')
    rospy.Subscriber('/laser_scanner/scan', LaserScan, laser_scan_callback)
    rospy.spin()