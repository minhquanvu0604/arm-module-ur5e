#! /usr/bin/env python3

import os, sys

import numpy as np
from threading import Thread
from enum import Enum
from copy import deepcopy

import rospy
import tf2_ros


# Add path
current_dir = os.path.dirname(__file__) # dir scripts
parent_dir = os.path.dirname(current_dir)  # dir python
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

sys.path.append('/home/quanvu/ros/apple_ws/src/simple_ur5_controller/python')


# Importing planner module
from src.UR5e import UR5e
from src.collision_manager import CollisionManager
from src.utility import *


CONTROL_RATE = 10  # Hz
HOME_CONFIG = "home"


class MissionPlanner:

    def __init__(self, run_srv_client: bool = True) -> None:

        rospy.init_node("simple_ur5_controller", log_level=2, anonymous=True)
        rospy.loginfo("Initialising MissionPlanner")
        self.rate = rospy.Rate(CONTROL_RATE)

        # Initialize the UR3e controller
        self.ur5e = UR5e()
        self._system_halted = False
        self._success = True

        # @NOTE: no Collision at the moment
        # self.collisions = CollisionManager(self.ur5e.get_scene())

        # Setup the scene with ur3e controller and homing
        self.ur5e.go_to_target_pose_name(HOME_CONFIG)
        # self.ur5e.open_gripper_to(width=580, force=200)

        # @TODO: review this method
        # # TF2 listener and broadcaster to deal with the transformation
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # TODO: review estop thread
        # Initialize the safety thread
        # self._safety_thread = Thread(target=self.safety_check)
        # self._safety_thread.start()

        rospy.on_shutdown(self._cleanup)

        self._system_loop()


    def _system_loop(self):
        # self.ur5e.go_to_pose_goal()
        pass

    def _cleanup(self):
        rospy.loginfo("Cleaning up")
        # self._safety_thread.join()
        self.ur5e.shutdown()
        self.collisions.remove_collision_object()
        rospy.loginfo("Clean-up completed")


if __name__ == "__main__":
    mp = MissionPlanner()

    # a = Pose()



    # mp.ur5e.go_to_target_pose_name()
