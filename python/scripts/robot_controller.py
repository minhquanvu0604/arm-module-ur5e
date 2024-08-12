#! /usr/bin/env python3

import os, sys

# Add path
# current_dir = os.path.dirname(__file__) # dir scripts
# parent_dir = os.path.dirname(current_dir)  # dir python
# if parent_dir not in sys.path:
#     sys.path.append(parent_dir)
sys.path.append('/home/quanvu/ros/apple_ws/src/simple_ur5_controller/python')
sys.path.append('/root/apple_ws/src/python')


# import numpy as np
import math
from threading import Thread
from enum import Enum
from copy import deepcopy
import json

import rospy
import tf2_ros
import tf.transformations
from sensor_msgs.msg import JointState
# from srv import MoveToPose, MoveToPoseResponse


# Importing planner module
from src.UR5e import UR5e
from src.collision_manager import CollisionManager
from src.utility import extract_waypoints_rpy, WAYPOINT_PATH
from geometry_msgs.msg import Pose


CONTROL_RATE = 10  # Hz

# MoveIt
# HOME_MOVEIT = "home"
# UP_CONFIG = "up"

class PoseTest:
    """
    To get sim groundtruth pose: rosrun tf tf_echo world tool0

    """

    def __init__(self, goal_list:bool = True) -> None:
        rospy.init_node("simple_ur5_controller", log_level=1, anonymous=True)
        rospy.loginfo("Initialising PoseTest")
        self._rate = rospy.Rate(CONTROL_RATE)

        # Initialize the UR3e controller
        self.robot = UR5e()

        # @NOTE: is this needed?
        # self._system_halted = False
        # self._success = True

        # @NOTE: no Collision at the moment
        # self.collisions = CollisionManager(self.robot.get_scene())

        # Setup the scene with ur3e controller and homing
        # self.robot.go_to_target_pose_name(UP_CONFIG)
        # self.robot.open_gripper_to(width=580, force=200)

        # @TODO: review this method
        # # TF2 listener and broadcaster to deal with the transformation
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # TODO: review estop thread
        # Initialize the safety thread
        # self._safety_thread = Thread(target=self.safety_check)
        # self._safety_thread.start()

        rospy.on_shutdown(self.cleanup)


        # Move to home position
        self.robot.go_to_joint_goal(self.robot.JOINT_TARGET_RAD, wait=True)

        # Main loop
        if goal_list:
            self.goal_list_main_loop()

        else: # Using ROS service
            self._service = rospy.Service('move_to_pose', MoveToPose, self.handle_move_to_pose)
            rospy.loginfo("Robot controller service is ready.")
            rospy.spin()


    def goal_list_main_loop(self) -> None:
        # waypoints = PoseTest.extract_waypoints_quartenion(WAYPOINT_PATH)
        waypoints = utility.extract_waypoints_rpy(WAYPOINT_PATH)
    
        parent_frame_id = "world"
        goal_id = 0
        for pose_goal in waypoints:
            print(f"[PoseTest] Start planning goal {goal_id}")
            # self.robot.go_to_pose_goal(pose, None, parent_frame_id)
            self.robot.go_to_pose_goal_simple(pose_goal)
            print(f"[PoseTest] Goal {goal_id} completed\n")
            goal_id += 1


    def handle_move_to_pose(self, req):
        pose_goal = req.pose

        success = self.robot.go_to_pose_goal_simple(pose_goal)

        response = MoveToPoseResponse()
        response.success = success

        return response


    def cleanup(self) -> None:
        rospy.loginfo("[PoseTest] Cleaning up")
        # self._safety_thread.join()
        self.robot.shutdown()
        # self.collisions.remove_collision_object()
        rospy.loginfo("[PoseTest] Clean-up completed")


if __name__ == "__main__":
    mp = PoseTest()