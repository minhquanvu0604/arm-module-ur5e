#! /usr/bin/env python3

import os, sys
top_level_package = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..',)) # arm_module_ur5e_controller
sys.path.insert(0, top_level_package)

import math
import threading
from copy import deepcopy
import yaml

import rospy
import tf2_ros
import tf.transformations
from sensor_msgs.msg import JointState

from arm_module_ur5e_controller.srv import MoveToPose, MoveToPoseResponse
from arm_module_ur5e_controller.srv import PoseService, PoseServiceResponse

from python.src.UR5e import UR5e
from python.src.collision_manager import CollisionManager
from python.src.utility import read_waypoints_rpy, read_joint_path_rad
from geometry_msgs.msg import Pose

WAYPOINT = top_level_package + "/cfg/list_poses_ur5e_demo.json" # List of random poses initially picked to test MoveIt
JOINT_PATH = top_level_package + "/cfg/near_q_list_deg.yaml" # List of joint values in 9 configs for each near/far scenario

# Joint values guess - aid the solver to come up with elegant solutions
# HOME_JOINT_DEG = [0, -46.86, 44.32, -177.46, -90.16, 0.02]
# HOME_JOINT_RAD = [math.radians(angle) for angle in HOME_JOINT_DEG]
# HOME_JOINT_RAD = [-1.79734006, -0.75991636, 1.4489723, -3.8833576, -1.3446017, 0.01186824]
HOME_JOINT_RAD = [-1.7973400948921046, -0.7555930298280389, 1.455735132902352, 2.401335236663474, -1.3445789828512975, 0.0020295319930268008]

class RobotController:

    """
    To get sim groundtruth pose: rosrun tf tf_echo world tool0
    """
    RATE = 10  # Hz

    def __init__(self, cfg) -> None:

        rospy.init_node("arm_module_ur5e", log_level=1, anonymous=True)
        rospy.loginfo("Initialising RobotController")

        # Configs
        goal_pose_ros_service = cfg['goal_pose_ros_service']
        current_pose_ros_service = cfg['current_pose_ros_service']

        self._rate = rospy.Rate(RobotController.RATE)

        # Initialize the UR3e controller
        self.robot = UR5e()

        # @NOTE: is this needed?
        # self._system_halted = False
        # self._success = True
        self._secure_data = threading.Lock()

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

        joints = self.robot.group.get_current_joint_values()
        print(f"[RobotController] Current joint values: {joints}")
        # raise Exception("Stop here")


        # Move to home position
        print("[RobotController] Moving to home position")
        self.robot.go_to_joint_goal(HOME_JOINT_RAD, wait=True)
        print("[RobotController] Moving to home position DONE")
            
        if current_pose_ros_service:
            self._pose_service = rospy.Service('mvps/arm_module/query_data', PoseService, self._query_pose_callback)
            rospy.loginfo("[RobotController] Service to query current pose is ready.")

        if goal_pose_ros_service:
            self._move_service = rospy.Service('mvps/arm_module/pose', MoveToPose, self._move_to_pose_callback)
            rospy.loginfo("[RobotController] Service to receive goal pose is ready.")   
        else:
            self._control_thread = threading.Thread(target=self._execute_demo_joint_list)
            self._control_thread.start()

        rospy.spin()


    def _execute_demo_pose_list(self) -> None:

        waypoints = read_waypoints_rpy(WAYPOINT)    
        goal_id = 0
        for pose_goal in waypoints:
            print(f"[RobotController] Start planning goal {goal_id}")
            # self.robot.go_to_pose_goal(pose, None, parent_frame_id)
            self.robot.go_to_pose_goal_simple(pose_goal)
            print(f"[RobotController] Goal {goal_id} completed\n")
            goal_id += 1
        print("[RobotController] Demo completed")


    def _execute_demo_joint_list(self) -> None:

        joint_path = read_joint_path_rad(JOINT_PATH)
        joint_path = [[math.radians(joint) for joint in config] for config in joint_path] # convert to rad
        goal_id = 0
        for joint_goal in joint_path:
            print(f"[RobotController] Start planning goal {goal_id}")
            self.robot.go_to_joint_goal(joint_goal)
            print(f"[RobotController] Goal {goal_id} completed\n")
            goal_id += 1
        print("[RobotController] Demo completed")


    def _move_to_pose_callback(self, req):
        pose_goal = req.pose
        # rospy.loginfo("Starting to move to the goal pose")
        success = self.robot.go_to_pose_goal_simple(pose_goal, wait=True)
        # rospy.loginfo("Finished moving to the goal pose")

        response = MoveToPoseResponse()
        response.success = success

        return response


    def _query_pose_callback(self, req):
        self._secure_data.acquire()
        try:
            # Obtain the current pose (which is a PoseStamped object
            # IMPORTANT: this in in base_link frame, while ing UR teach pendant Move tab, frame Base shows XYZRPY in base frame
            pose_stamped = self.robot.group.get_current_pose()
        finally:
            self._secure_data.release()

        response = PoseServiceResponse()
        response.pose.header = pose_stamped.header
        response.pose.pose.position = pose_stamped.pose.position
        response.pose.pose.orientation = pose_stamped.pose.orientation

        return response


    def cleanup(self) -> None:
        rospy.loginfo("[RobotController] Cleaning up")
        # self._safety_thread.join()
        self.robot.shutdown()
        # self.collisions.remove_collision_object()
        rospy.loginfo("[RobotController] Clean-up completed")


if __name__ == "__main__":

    yaml_file = '/home/quanvu/git/arm-module-ur5e/arm_module_ur5e_controller/cfg/robot_controller_config.yaml'
    with open(yaml_file, 'r') as file:
        cfg = yaml.safe_load(file)

    try:    
        mp = RobotController(cfg)
    except rospy.ROSInterruptException:
        pass