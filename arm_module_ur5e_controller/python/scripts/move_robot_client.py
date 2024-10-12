import sys
sys.path.append('/home/quanvu/git/arm-module-ur5e/arm_module_ur5e_controller/python')
sys.path.append('/root/apple_ws/src/python')

import rospy
from geometry_msgs.msg import Pose
from arm_module_ur5e.srv import MoveToPose, MoveToPoseRequest

from src.utility import extract_waypoints_rpy, WAYPOINT


def move_robot_client():
    rospy.init_node('move_robot_client')
    rospy.wait_for_service('mvps/arm_module/pose')

    waypoints = extract_waypoints_rpy(WAYPOINT)
    
    try:
        move_to_pose = rospy.ServiceProxy('mvps/arm_module/pose', MoveToPose)

        for id, pose_goal in enumerate(waypoints):
            request = MoveToPoseRequest(pose=pose_goal)
            response = move_to_pose(request)

            if response.success:
                rospy.loginfo(f"The robot successfully moved to pose {id}.")
            else:
                rospy.logwarn(f"The robot failed to move to pose {id}.")
            
            rospy.sleep(1)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


if __name__ == "__main__":
    move_robot_client()
