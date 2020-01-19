#!/usr/bin/env python

from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
import rospy
import math

def laser_scan_callback(msg):
    # In angle between 0 to 90 degree
    # If distance < 0.5m, call "move_path_B" service
    # Else, call "continue_move_path_A" service
    for index in range(0, 900):
        if msg.ranges[index] < 0.5:
            rospy.wait_for_service('move_path_B', Trigger)
            try:
                pause_trigger = rospy.ServiceProxy('move_path_B', Trigger)
                pause_trigger_response = pause_trigger()
                print(pause_trigger_response.message)
            except rospy.ServiceException:
                print("Service move_path_B call failed") 
        else:
            rospy.wait_for_service('continue_move_path_A', Trigger)
            try:
                resume_trigger = rospy.ServiceProxy('continue_move_path_A', Trigger)
                resume_trigger_response = resume_trigger()
                print(resume_trigger_response.message)
            except rospy.ServiceException:
                print("Service continue_move_path_A call failed") 


if __name__ == "__main__":
    rospy.init_node('psen_scan_app')
    rospy.Subscriber('/laser_scanner/scan', LaserScan, laser_scan_callback)
    rospy.spin()