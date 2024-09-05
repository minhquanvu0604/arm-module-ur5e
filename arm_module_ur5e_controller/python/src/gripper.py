#! /usr/bin/env python3

import rospy
from onrobot_rg_control.msg import OnRobotRGOutput

from src.utility import *

class Gripper:

    def __init__(self) -> None:

        self._cmd_pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)
            

    def open(self, force=None):
        rospy.loginfo("Opening gripper")
        command = OnRobotRGOutput()
        command.rGFR = force
        command.rGWD = GRIPPER_OPEN
        command.rCTR = 16
        self._cmd_pub.publish(command)


    def close(self, force=None):
        rospy.loginfo("Closing gripper")
        command = OnRobotRGOutput()
        command.rGFR = force
        command.rGWD = GRIPPER_CLOSE
        command.rCTR = 16
        self._cmd_pub.publish(command)


    def open_to(self, width, force=None):
        rospy.loginfo(f"Opening gripper to {width}")
        command = OnRobotRGOutput()
        command.rGFR = force
        command.rGWD = width
        command.rCTR = 16
        self._cmd_pub.publish(command)

    def move_to(self, width, force=None):
        rospy.loginfo(f"Moving gripper to {width}")
        command = OnRobotRGOutput()
        command.rGFR = force
        command.rGWD = width
        command.rCTR = 16
        self._cmd_pub.publish(command)
