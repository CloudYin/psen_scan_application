#!/usr/bin/env python
# Copyright (c) 2020 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from geometry_msgs.msg import Point
from pilz_robot_programming.robot import *
from pilz_robot_programming.commands import *
import math
import rospy
from std_srvs.srv import Trigger
import threading
from sensor_msgs.msg import LaserScan

__REQUIRED_API_VERSION__ = "1"

# start position (unit: degree)
# A1: 0, A2: 0, A3: 90, A4: 0, A5: 90, A6: 0
home_pos = [0, 0, math.radians(90), 0, math.radians(90), math.radians(-45)]

__MOVE_VELOCITY__ = 0.2  # PTP velocity
__LIN_VELOCITY__ = 0.2 # LIN velocity

r = None

class HackRobot(Robot):
    def parallelMove(self, cmd):
        self._sequence_client.send_goal(cmd._get_sequence_request(self))
        self._sequence_client.wait_for_result()

def _pause_callback(request):
    # Pause needs to be called from another thread
    pause_thread = threading.Thread(target=r.pause)
    pause_thread.start()

    # Make sure robot has stopped
    rospy.sleep(.3)

    # Your motion command
    r.move(Ptp(goal=home_pos, vel_scale=__MOVE_VELOCITY__))

    pause_thread.join()
    return [True, "success"]

def _resume_callback(request):
    rospy.logerr("_resume_callback()")
    r.resume()
    return [True, "success"]

def _laser_scan_callback(msg):
    for index in range(0, 900):
        if msg.ranges[index] < 0.5:
            r.pause()

# The path A
def start_program(r):
    print("Executing " + __file__)

    # Move to start point 
    r.move(Ptp(goal=home_pos, vel_scale=__MOVE_VELOCITY__))  

    # Simple ptp and lin movement
    while not rospy.is_shutdown():
        r.move(Ptp(goal=Pose(position=Point(-0.268, -0.158, 0.2), orientation=from_euler(0, math.radians(180), math.radians(-45))), vel_scale=__MOVE_VELOCITY__, reference_frame="prbt_base_link"))
        r.move(Lin(goal=Pose(position=Point(-0.568, -0.158, 0.2), orientation=from_euler(0, math.radians(180), math.radians(-45))), vel_scale=__LIN_VELOCITY__, relative=False, reference_frame="prbt_base_link"))
        r.move(Lin(goal=Pose(position=Point(-0.568, 0.142, 0.2), orientation=from_euler(0, math.radians(180), math.radians(-45))), vel_scale=__LIN_VELOCITY__, relative=False, reference_frame="prbt_base_link"))
        r.move(Lin(goal=Pose(position=Point(-0.268, 0.142, 0.2), orientation=from_euler(0, math.radians(180), math.radians(-45))), vel_scale=__LIN_VELOCITY__, relative=False, reference_frame="prbt_base_link"))
        r.move(Lin(goal=Pose(position=Point(-0.268, -0.158, 0.2), orientation=from_euler(0, math.radians(180), math.radians(-45))), vel_scale=__LIN_VELOCITY__, relative=False, reference_frame="prbt_base_link"))


if __name__ == "__main__":
    # Init a ros node
    rospy.init_node('robot_program_node')

    r = HackRobot(__REQUIRED_API_VERSION__)

    rospy.Subscriber('/laser_scanner/scan', LaserScan, _laser_scan_callback)
    pause_service = rospy.Service("move_path_B", Trigger, _pause_callback)
    resume_service = rospy.Service("continue_move_path_A", Trigger, _resume_callback)

    start_program(r)