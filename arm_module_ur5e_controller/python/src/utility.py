#! /usr/bin/env python3
import os
import json, yaml
import math
from collections import OrderedDict

import rospy
import tf
import tf.transformations

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from moveit_commander.conversions import pose_to_list

from math import pi, tau, dist, fabs, cos

# Apple Picking
# import rospkg
# rospack = rospkg.RosPack()
# package_path = rospack.get_path('arm_module_ur5e_controller')
top_level_package = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..',))


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)
    elif type(goal) is Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance #and cos_phi_half >= cos(tolerance / 2.0)
    return True

def list_to_pose(pose: list) -> Pose:
    p = Pose()
    p.position.x = pose[0]
    p.position.y = pose[1]
    p.position.z = pose[2]
    ori_in_quat = tf.transformations.quaternion_from_euler(
        pose[3], pose[4], pose[5], axes='rxyz')
    p.orientation.x = ori_in_quat[0]
    p.orientation.y = ori_in_quat[1]
    p.orientation.z = ori_in_quat[2]
    p.orientation.w = ori_in_quat[3]
    return p

def transformstamped_to_posestamped(ts: TransformStamped) -> PoseStamped:
    r"""
    Convert a TransformStamped to a PoseStamped instance

    @param: ts The TransformStamped to be converted
    @returns: PoseStamped A PoseStamped instance

    """
    ps = PoseStamped()
    ps.header.stamp = ts.header.stamp
    ps.header.frame_id = ts.header.frame_id
    ps.pose.position.x = ts.transform.translation.x
    ps.pose.position.y = ts.transform.translation.y
    ps.pose.position.z = ts.transform.translation.z
    ps.pose.orientation = ts.transform.rotation
    return ps


def transformstamped_to_pose(ts: TransformStamped) -> Pose:
    r"""
    Convert a TransformStamped to a Pose instance

    @param: ts The TransformStamped to be converted
    @returns: Pose A Pose instance
    """
    p = Pose()
    p.position.x = ts.transform.translation.x
    p.position.y = ts.transform.translation.y
    p.position.z = ts.transform.translation.z
    p.orientation = ts.transform.rotation
    return p

def pose_to_transformstamped(pose: Pose, child_frame_id: str, parent_frame_id: str) -> TransformStamped:
    r"""
    Convert a pose to a TransformStamped instance

    @param: pose The pose to be converted
    @param: frame_id The frame id of the pose
    @param: child_frame_id The child frame id of the pose
    @returns: TransformStamped A TransformStamped instance

    """
    ts = TransformStamped()
    ts.header.stamp = rospy.Time.now()
    ts.header.frame_id = parent_frame_id
    ts.child_frame_id = child_frame_id
    ts.transform.translation.x = pose.position.x
    ts.transform.translation.y = pose.position.y
    ts.transform.translation.z = pose.position.z
    ts.transform.rotation = pose.orientation
    return ts

def create_marker(frame: str, type: int, pose: Pose, scale=[0.05, 0.05, 0.05], color=[0, 1, 0, 0.2]) -> Marker:
    marker = Marker()

    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()

    # marker.ns = "markers"
    marker.id = 100
    marker.type = type
    # marker.action = Marker.ADD
    marker.pose = pose

    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    return marker

def list_to_PoseStamped(pose: list, frame_id: str = "base_link_inertia") -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x = pose[0]
    ps.pose.position.y = pose[1]
    ps.pose.position.z = pose[2]
    ori_in_quat = tf.transformations.quaternion_from_euler(
        pose[3], pose[4], pose[5], axes='sxyz')
    ps.pose.orientation.x = ori_in_quat[0]
    ps.pose.orientation.y = ori_in_quat[1]
    ps.pose.orientation.z = ori_in_quat[2]
    ps.pose.orientation.w = ori_in_quat[3]
    return ps

def read_waypoints_quartenion(file): 
    with open(file, 'r') as f:
        if file.endswith('.json'):
            config = json.load(f)
            config = config["posePlanner"]["list"] 
        elif file.endswith('.yaml'):
            config = yaml.safe_load(f)
        else:
            raise ValueError("Not JSON or YAML file")
        
    waypoints = []
    for waypoint in config.values():
        pose = Pose()
        pose.position.x = waypoint["x"]
        pose.position.y = waypoint["y"]
        pose.position.z = waypoint["z"]
        pose.orientation.x = waypoint["qx"]
        pose.orientation.y = waypoint["qy"]
        pose.orientation.z = waypoint["qz"]
        pose.orientation.w = waypoint["qw"]
        waypoints.append(pose)
        
    return waypoints

def read_waypoints_rpy(file): 
    with open(file, 'r') as f:
        config = json.load(f) 
    waypoints = []
    for waypoint in config["posePlanner"]["list"]:
        pose = Pose()
        pose.position.x = waypoint["x"]
        pose.position.y = waypoint["y"]
        pose.position.z = waypoint["z"]
        # Convert RPY to quaternion
        roll = waypoint["roll"]
        pitch = waypoint["pitch"]
        yaw = waypoint["yaw"]
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        waypoints.append(pose)
    return waypoints

def read_joint_path(yaml_file):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
    configs = []
    for key in data:
        config = [
            data[key]['shoulder_pan_joint'],
            data[key]['shoulder_lift_joint'],
            data[key]['elbow_joint'],
            data[key]['wrist_1_joint'],
            data[key]['wrist_2_joint'],
            data[key]['wrist_3_joint']
        ]
        configs.append(config)
    return configs


def convert_joint_values_to_rad(input_file, output_file):
    """
    Converts joint values from degrees to radians in a YAML file and saves the result.
    """
    with open(input_file, 'r') as file:
        data = yaml.safe_load(file)
    # Convert all joint values from degrees to radians 
    for key in data:
        for joint, value in data[key].items():
            # Choose the convsersion type
            data[key][joint] = round(math.radians(value), 2) # round to 2 decimal places
            
            # data[key][joint] = round(math.degrees(value), 2)
            
            # if joint != 'wrist_1_joint':
            #     continue            
            # data[key][joint] += round(math.pi*2, 2) # Add 2*pi to all joint values
    # Preserve the original order of the keys in the YAML file
    with open(output_file, 'w') as file:
        yaml.dump(data, file, default_flow_style=False, sort_keys=False)



if __name__ == "__main__":
    convert_joint_values_to_rad('/home/quanvu/git/arm-module-ur5e/arm_module_ur5e_controller/cfg/near_q_list_rad_sim.yaml',
                                '/home/quanvu/git/arm-module-ur5e/arm_module_ur5e_controller/cfg/near_q_list_deg_sim.yaml')