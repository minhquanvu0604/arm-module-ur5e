#! /usr/bin/env python3

import sys
import numpy as np
from copy import deepcopy

import tf2_ros
import rospy

import moveit_commander
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory, Constraints, JointConstraint, OrientationConstraint

from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

# from src.gripper import Gripper
from python.src.utility import all_close, pose_to_transformstamped, transformstamped_to_pose, create_marker
# from src.collision_manager import CollisionManager
# from scipy.spatial.transform import Rotation as R


class ArmModuleUR5e():
    r"""
    A class to control the UR3e robot arm using moveit_commander.
    Dependency includes MoveIt and ROS 
    """

    POS_TOL = 0.01  # m
    ORI_TOL = 0.01  # m
    TOL_CHECK = 0.02  # m
    MAX_VEL_SCALE_FACTOR = 0.1 #0.05
    MAX_ACC_SCALE_FACTOR = 0.1 #0.05


    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("manipulator")

        # SET AND TEST
        # self.group.set_pose_reference_frame("base_link")
        self.group.set_pose_reference_frame("world")
        print("POSE REFERENCE FRAME: ", self.group.get_pose_reference_frame())

        # self.group_names = self.robot.get_group_names()
        # self._cur_js = self.group.get_current_joint_values()

        self.group.set_end_effector_link("camera_color_optical_frame")
        print("EELINK: ", self.group.get_end_effector_link())

        current_pose = self.group.get_current_pose().pose
        print("Current pose: ", current_pose)

        # exit(1)


        # self._gripper = Gripper()
        self.constraints = Constraints()

        self._marker_pub = rospy.Publisher(
            "/visualization_marker", Marker, queue_size=10)

        self._display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=20)

        # For lookup transform
        # TODO: check 
        # self._tf_buffer = tf2_ros.Buffer()
        # self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        # self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        self._movegroup_setup()

        rospy.logdebug(f"=== PLANNING FRAME DETAILS ===")
        rospy.logdebug(f"============ Planning frame: {self.group.get_planning_frame()}")
        rospy.logdebug(f"============ End effector link: {self.group.get_end_effector_link()}")
        rospy.logdebug(f"============ Pose reference frame: {self.group.get_pose_reference_frame()}")


    def _movegroup_setup(self):
        r"""
        Setup the move group.
        """
        # self.group.set_start_state_to_current_state()
        self.group.set_planner_id("RRTConnect") # ompl_planning.yaml
        self.group.set_planning_time(20)
        self.group.set_num_planning_attempts(10)
        self.group.set_goal_position_tolerance(ArmModuleUR5e.POS_TOL)
        self.group.set_goal_joint_tolerance(ArmModuleUR5e.POS_TOL)
        self.group.set_max_velocity_scaling_factor(ArmModuleUR5e.MAX_VEL_SCALE_FACTOR)
        self.group.set_max_acceleration_scaling_factor(ArmModuleUR5e.MAX_ACC_SCALE_FACTOR)

        # self.init_path_constraints()

        # Dynamicall attach the gripper wire and camera as collision objects -----------------------------
        # # Add a small fragment piece as gripper cable node
        # cable_cap_pose = PoseStamped()
        # cable_cap_pose.pose = list_to_pose([-0.045, -0.01, 0.01, 1.57, 0, 1.57])
        # cable_cap_pose.header.frame_id = "tool0"
        # self.scene.add_cylinder("cable_cap", cable_cap_pose, 0.02, 0.01)
        # self.scene.attach_mesh("tool0", "cable_cap", touch_links=[
        #     "onrobot_rg2_base_link", "wrists_3_link"])

        # # Add box to wrap around the camera mounter
        # camera_mount_pose = PoseStamped()
        # camera_mount_pose.pose = list_to_pose( [0.0, -0.0455, 0.0732, 0.0, 0.0, 0.0])
        # camera_mount_pose.header.frame_id = "tool0"
        # self.scene.add_box("camera_mount", camera_mount_pose, size=(0.08, 0.1, 0.035))
        # self.scene.attach_mesh("tool0", "camera_mount", touch_links=[
        #     "onrobot_rg2_base_link"])
        # ------------------------------------------------------------------------------------------------


        # Add fake obstacles as ceilings and walls to avoid unwanted configurations like going overhead
        # Can be replaced by path constraints ------------------------------------------------------------
        # # Add a thin box above the crate to restraint the robot motion not to go downward too much
        # wall_pose = PoseStamped()
        # wall_pose.pose = list_to_pose([0.0, -0.71, -0.07, 0.0, 0.0, 0.0])
        # wall_pose.header.frame_id = "base_link"
        # bound_id = "wall"
        # self.scene.add_box(bound_id, wall_pose, size=(2, 0.01, 2))

        # # Add a ceiling to avoid robot with bottle to go overhead
        # ceilling_pose = PoseStamped()
        # ceilling_pose.pose = list_to_pose([0.0, 0.0, 0.7, 0.0, 0.0, 0.0])
        # ceilling_pose.header.frame_id = "base_link"
        # ceilling_id = "ceilling"
        # self.scene.add_box(ceilling_id, ceilling_pose, size=(2, 2, 0.01))
        # ------------------------------------------------------------------------------------------------


    def init_path_constraints(self):
        """
        Constraints to maintain elbow up cofig 
        """
        constraints = Constraints()
        constraints.name = "elbow_up"

        joint_constraint_04 = JointConstraint()
        joint_constraint_04.joint_name = "shoulder_pan_joint"
        joint_constraint_04.position = 2.0944
        joint_constraint_04.tolerance_above = 2.5
        joint_constraint_04.tolerance_below = 2.5
        joint_constraint_04.weight = 1
        constraints.joint_constraints.append(joint_constraint_04)

        joint_constraint_02 = JointConstraint()
        joint_constraint_02.joint_name = "shoulder_lift_joint"
        joint_constraint_02.position = -2.443
        joint_constraint_02.tolerance_above = np.pi/4
        joint_constraint_02.tolerance_below = np.pi/4
        joint_constraint_02.weight = 1
        constraints.joint_constraints.append(joint_constraint_02)

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = "elbow_joint"
        joint_constraint.position = -2.3562/2
        joint_constraint.tolerance_above = 2.3562/2
        joint_constraint.tolerance_below = 2.3562/2
        joint_constraint.weight = 1
        constraints.joint_constraints.append(joint_constraint)

        # joint_constraint_03 = JointConstraint()
        # joint_constraint_03.joint_name = "wrist_2_joint"
        # joint_constraint_03.position = pi/2
        # joint_constraint_03.tolerance_above = 0.7
        # joint_constraint_03.tolerance_below = 0.7
        # joint_constraint_03.weight = 1
        # constraints.joint_constraints.append(joint_constraint_03)

        # joint_constraint_05 = JointConstraint()
        # joint_constraint_05.joint_name = "wrist_3_joint"
        # joint_constraint_05.position = self.group.get_current_joint_values()[5]
        # joint_constraint_05.tolerance_above = 3.14
        # joint_constraint_05.tolerance_below = 3.14
        # joint_constraint_05.weight = 1
        # constraints.joint_constraints.append(joint_constraint_05)

        self.constraints = constraints
        self.group.set_path_constraints(constraints)


    def shutdown(self):
        r"""
        Shutdown the moveit_commander.
        """
        self.group.stop()
        moveit_commander.roscpp_shutdown()
 

    # ============ Robot control basic actions ===============================================================
    def go_to_pose_goal_simple(self, pose: Pose, wait=True, timeout=10):
        r"""
        Move the robot to the specified pose.
        @param: pose A Pose instance
        @param: wait A bool to wait for the robot to reach the goal
        @returns: bool True if successful by comparing the goal and actual poses
        """
        # self._visualise_pose_in_loop(pose)

        # Set q guess
        self.group.set_joint_value_target(ArmModuleUR5e.JOINT_TARGET_RAD)
        # Set target
        self.group.set_pose_target(pose)
        self.group.go(wait=wait)
        self.group.stop()
        self.group.clear_pose_targets()

        cur_pose = self.group.get_current_pose().pose
        return all_close(pose, cur_pose, ArmModuleUR5e.TOL_CHECK)


    def go_to_joint_goal_rad(self, joint_goal, wait=True, timeout=10):
        r"""
        Move the robot to the specified joint angles.

        @param: joint_angle A list of floats
        @returns: bool True if successful by comparing the goal and actual joint angles
        """
        if not isinstance(joint_goal, list):
            rospy.logerr("Invalid joint angle")
            return False
        
        self.group.go(joint_goal, wait=wait)
        self.group.stop()

        # Print the current pose - [DEBUG]
        # cur_pose = self.group.get_current_pose().pose
        # orientation = cur_pose.orientation
        # qx = orientation.x
        # qy = orientation.y
        # qz = orientation.z
        # qw = orientation.w
        # roll, pitch, yaw = tf.transformations.euler_from_quaternion([qx, qy, qz, qw])
        # print(f"[go_to_joint_goal] Current pose: {cur_pose}")
        # print(f"[go_to_joint_goal] Orientation in RPY: roll={roll}, pitch={pitch}, yaw={yaw}")
        
        cur_joint = self.group.get_current_joint_values()
        
        return all_close(joint_goal, cur_joint, ArmModuleUR5e.TOL_CHECK)


    def go_to_pose_goal(self, pose: Pose, child_frame_id, parent_frame_id):
        r"""
        Move the robot to the specified pose.
        Current expected behavior of this planner
        go_to_pose_goal() then execute_plan()

        @param: pose A Pose instance
        @param: wait A bool to wait for the robot to reach the goal
        @returns: bool True if successful by comparing the goal and actual poses
        """
        # TODO: Review a better way to do this
        # have to do this because the pose is in the camera frame


        # self._visualise_pose_in_loop(pose)
        # self.visualize_target_pose(pose, frame_id='world')


        if parent_frame_id != self._planning_frame:
            pose = self._get_transform_in_planning_frame(
                pose, child_frame_id, parent_frame_id)

        plan, _ = self._gen_cartersian_path(pose)

        # self._display_traj(plan)
        self.execute_plan(plan, wait=True)

        # self.group.set_pose_target(pose)
        # self.group.go(wait=True)
        # self.group.stop()
        # self.group.clear_pose_targets()
        cur_pose = self.group.get_current_pose().pose
        return all_close(pose, cur_pose, 0.001)

    def execute_plan(self, plan: RobotTrajectory, wait=True):
        r"""
        Execute the plan.
        @param: plan A RobotTrajectory instance
        @returns: bool True if successful by comparing the goal and actual poses
        """
        try:
            result = self.group.execute(plan, wait=wait)
            rospy.loginfo("[Execution result] ,", result)
        except Exception as e:
            rospy.logerr(e)
            return False
        return all_close(plan.joint_trajectory.points[-1], self.group.get_current_pose().pose, 0.001)

    def stop(self):
        self.group.stop()


    # ============ Template actions =================================================================================
    def go_to_target_pose_name(self, name):
        """
        MoveIt:
        HOME_MOVEIT = "home"
        UP_CONFIG = "up"
        """
        self.group.set_named_target(name)
        self.group.go(wait=True)

        joint_goal = self.group.get_named_target_values(name)
        cur_joint = self.group.get_current_joint_values()
        return all_close(joint_goal, cur_joint, 0.001)

    def move_ee_along_axis(self, axis: str, delta: float) -> bool:
        r"""
        Move the end effector along the specified axis of the planning frame.
        @param: axis The axis to move along
        @param: delta The distance to move
        @returns: bool True if successful by comparing the goal and actual poses
        """
        goal = deepcopy(self.group.get_current_pose().pose)
        if axis == "x":
            goal.position.x += delta
        elif axis == "y":
            goal.position.y += delta
        elif axis == "z":
            goal.position.z += delta
        else:
            rospy.logerr("Invalid axis")
            return False
        plan, _ = self._gen_cartersian_path(target_pose=goal)
        if plan is None:
            return False
        done = self.execute_plan(plan=plan, wait=True)
        return done

    # Gripper control
    # def open_gripper(self, force=None):
    #     self._gripper.open(force)
    #     rospy.sleep(1)

    # def close_gripper(self, force=None):
    #     self._gripper.close(force)
    #     rospy.sleep(1)

    # def open_gripper_to(self, width, force=None):
    #     self._gripper.open_to(width, force)
    #     rospy.sleep(1)

    
    def _gen_cartersian_path(self, target_pose: Pose, ee_step=0.01, jump_thresh=0.0):
        r"""
        Generate a cartesian path as a straight line to a desired pose .

        @returns: RobotTrajectory instance for the cartesian path

        @TODO: Check this function
        """
        current_pose = self.group.get_current_pose().pose
        # generate a straight line path using a scaling 0 to 1 applied to the target pose differ to current pose
        waypoints = [current_pose, target_pose]

        fraction = 0.0
        fix_itterations = 0
        # max_i = 50 # Maxium fix iterations
        # matching_tolerance = 0.7
        # Blocking loop to ensure the validity of the cartesian path
        # while fraction < matching_tolerance:
        #     plan, fraction = self.group.compute_cartesian_path(
        #         waypoints, ee_step, jump_thresh, avoid_collisions=True, path_constraints=self.constraints)

        #     fix_itterations += 1
        #     if fix_itterations > max_i:  
        #         rospy.logerr(f"Failed to find a plan after {max_i} iterations")
        #         exit(1)
        plan, fraction = self.group.compute_cartesian_path(
                waypoints, ee_step, jump_thresh, avoid_collisions=True,) # path_constraints=self.constraints)

        # check if the plan is valid with timestamp duplication
        path = [plan.joint_trajectory.points[0]]
        for i in range(1, len(plan.joint_trajectory.points)):
            cur_point_stamp = plan.joint_trajectory.points[i].time_from_start.to_sec()
            prev_point_stamp = plan.joint_trajectory.points[i-1].time_from_start.to_sec()

            if cur_point_stamp == prev_point_stamp == 0:
                continue    

            path.append(plan.joint_trajectory.points[i])

        plan.joint_trajectory.points = path    
        # print(len(plan.joint_trajectory.points))   
        rospy.loginfo(
            f"[Generate Cartesian path] Fraction planned: {fraction}; Fix itteration {fix_itterations}")
        
        return plan, fraction


    def _get_transform_in_planning_frame(self, pose, child_frame_id: str, parent_frame_id: str, lookup_frame_id=None,):
        r"""
        Get the transform between two frames in the tf tree using lookup_transform.

        @param: pose A Pose instance
        @param: child_frame_id The child frame id
        @param: parent_frame_id The parent frame id
        @returns: Pose The pose in the planning frame
        
        @NOTE: Only for complex path
        """

        if lookup_frame_id is None:
            lookup_frame_id = self._planning_frame

        # Broadcast the target pose to the tf tree
        transform_target = pose_to_transformstamped(
            pose=pose, child_frame_id=child_frame_id, parent_frame_id=parent_frame_id)
        self._tf_broadcaster.sendTransform(transform_target)

        rospy.sleep(0.1) # Give tf time to update

        while True:
            try:
                tf_received = self._tf_buffer.lookup_transform(
                    lookup_frame_id, child_frame_id, rospy.Time(0), rospy.Duration(1.0))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(0.01)
                continue

        transformed_pose = transformstamped_to_pose(tf_received)

        return transformed_pose


    # ============ Visualization =================================================================
    def visualize_target_pose(self, pose: Pose, type: int=2, frame_id: str="world"):
        target_marker = create_marker(frame_id, type, pose)
        self._marker_pub.publish(target_marker)

    def _display_traj(self, plan):
        r"""
        Display the trajectory.
        """
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self._display_trajectory_publisher.publish(display_trajectory)

    def _visualise_pose_in_loop(self, pose):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            print(f"Visualizing target pose: {pose.position.x, pose.position.y, pose.position.z}")
            self.visualize_target_pose(pose, frame_id='world')
            rate.sleep()