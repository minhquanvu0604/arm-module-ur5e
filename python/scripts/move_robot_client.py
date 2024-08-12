#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from srv import MoveToPose, MoveToPoseRequest

from src.utility import extract_waypoints_rpy, WAYPOINT_PATH


def move_robot_client():
    rospy.init_node('move_robot_client')
    rospy.wait_for_service('move_to_pose')

    waypoints = utility.extract_waypoints_rpy(WAYPOINT_PATH)
    
    try:
        move_to_pose = rospy.ServiceProxy('move_to_pose', MoveToPose)

        # Create a Pose object (Example values)
        # pose = Pose()
        # pose.position.x = 0.4
        # pose.position.y = -0.2
        # pose.position.z = 0.3
        # pose.orientation.x = 0.0
        # pose.orientation.y = 0.0
        # pose.orientation.z = 0.0
        # pose.orientation.w = 1.0

        for id, pose_goal in enumerate(waypoints):
            # Create the service request
            request = MoveToPoseRequest(pose=pose_goal)

            # Call the service and receive the response
            response = move_to_pose(request)

            if response.success:
                rospy.loginfo(f"The robot successfully moved to pose {id}.")
            else:
                rospy.logwarn(f"The robot failed to move to pose {id}.")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    move_robot_client()
