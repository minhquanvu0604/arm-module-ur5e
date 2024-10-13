import os, sys
arm_module_ur5e_controller_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')) # arm_module_ur5e_controller
sys.path.insert(0, arm_module_ur5e_controller_path)

import yaml
import rospy
from geometry_msgs.msg import PoseStamped
from arm_module_ur5e_controller.srv import PoseService, PoseServiceResponse

from cfg.config import CONFIG_PATH
from python.src.utility import read_waypoints_quartenion

# Output obtained from following the q list from near_q_list_sim_deg.yaml
# and return the pose using moveit_commander.get_current_pose()
NEAR_Q_LIST_SIM_DEG_POSE_PATH = os.path.join(CONFIG_PATH, 'near_q_list_sim_deg_pose.yaml')


class FakePoseService:
    """
    Fake pose service for testing purposes. Mimics the behavior of the real pose service.
    """
    def __init__(self):
        rospy.loginfo("Starting fake pose service...")
        rospy.init_node('fake_pose_service')
        self._service = rospy.Service('mvps/arm_module/pose', PoseService, self._query_pose_callback)
    
        self._waypoints = read_waypoints_quartenion(NEAR_Q_LIST_SIM_DEG_POSE_PATH)
        self._i = 0    
        rospy.loginfo(f"Read {len(self._waypoints)} waypoints from {NEAR_Q_LIST_SIM_DEG_POSE_PATH}")
        rospy.loginfo("Fake pose service is ready.")
        rospy.spin()

    def _query_pose_callback(self, req):
        res = PoseServiceResponse()
        pose_stamped = PoseStamped()

        if self._i >= len(self._waypoints):
            self._i = 0
        pose_stamped.pose = self._waypoints[self._i]
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "world"

        res.pose = pose_stamped
        self._i += 1

        return res
    
if __name__ == "__main__":
    fake_pose = FakePoseService()