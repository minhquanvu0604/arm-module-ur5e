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

from python.src.UR5e import ArmModuleUR5e
from python.src.collision_manager import CollisionManager
from python.src.utility import read_waypoints_rpy, read_joint_path
from geometry_msgs.msg import Pose

WAYPOINT = top_level_package + "/cfg/list_poses_ur5e_demo.json" # List of random poses initially picked to test MoveIt
JOINT_PATH_DEG = top_level_package + "/cfg/near_q_list_sim_deg.yaml" # List of joint values in 9 configs for each near/far scenario

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
        send_goal_pose_ros_service = cfg['send_goal_pose_ros_service']
        query_current_pose_ros_service = cfg['query_current_pose_ros_service']

        self._rate = rospy.Rate(RobotController.RATE)

        # Initialize the UR3e controller
        self.robot = ArmModuleUR5e()

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

        # joints = self.robot.group.get_current_joint_values()
        # print(f"[RobotController] Starting joint values: {joints}")
        # raise Exception("Stop here")

        # Move to home position
        print("[RobotController] Moving to home position")
        self.robot.go_to_joint_goal_rad(HOME_JOINT_RAD, wait=True)
        print("[RobotController] Moving to home position DONE")
        
        # exit(0)

        if query_current_pose_ros_service:
            self._pose_service = rospy.Service('mvps/arm_module/query_data', PoseService, self._query_pose_callback)
            rospy.loginfo("[RobotController] Service to query current pose is ready.")

        if send_goal_pose_ros_service:
            self._move_service = rospy.Service('mvps/arm_module/pose', MoveToPose, self._move_to_pose_callback)
            rospy.loginfo("[RobotController] Service to receive goal pose is ready.")   
        else:
            self._control_thread = threading.Thread(target=self._execute_demo_joint_list)
            self._control_thread.start()
            pass

        # ROS will keep spinning until the node is shutdown with Ctrl+C
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
        print("[RobotController] DEMO COMPLETE")

    def _execute_demo_joint_list(self) -> None:
        joint_path = read_joint_path(JOINT_PATH_DEG)
        joint_path = [[math.radians(joint) for joint in config] for config in joint_path] # convert to rad
        goal_id = 0

        for joint_goal in joint_path:
            print(f"[RobotController] Start planning goal {goal_id}")
            self.robot.go_to_joint_goal_rad(joint_goal)

            # DEBUGGING
            current_pose = self.robot.group.get_current_pose().pose # PoseStamped -> Pose
            print(f"[RobotController] POSE AT GOAL {goal_id}: {current_pose}")
            # self.robot.visualize_target_pose(current_pose)

            print(f"[RobotController] Goal {goal_id} completed\n")
            goal_id += 1

            input("Press Enter to continue...")

        print("[RobotController] All goals completed")

        # Move to home position
        print("[RobotController] Moving back to home position")
        self.robot.go_to_joint_goal_rad(HOME_JOINT_RAD, wait=True)
        print("[RobotController] Moving back to home position DONE")
        print("[RobotController] DEMO COMPLETE")
        print("Press Ctrl+C to stop the program...")


        # [RobotController] POSE AT GOAL 0: header: 
        #   seq: 0
        #   stamp: 
        #     secs: 2324
        #     nsecs: 491000000
        #   frame_id: "world"
        # pose: 
        #   position: 
        #     x: -1.1463648012854286
        #     y: -0.18907574326605014
        #     z: 1.216520233063161
        #   orientation: 
        #     x: -0.8164150192083564
        #     y: -0.41107552144254766
        #     z: 0.18013850094998465
        #     w: 0.363364214744998
        # [RobotController] Start planning goal 1
        # [RobotController] Goal 1 completed

        # [RobotController] POSE AT GOAL 1: header: 
        #   seq: 0
        #   stamp: 
        #     secs: 2327
        #     nsecs: 123000000
        #   frame_id: "world"
        # pose: 
        #   position: 
        #     x: -1.136601715974801
        #     y: -0.0015548998115068081
        #     z: 1.2484860508520794
        #   orientation: 
        #     x: -0.6754464488102534
        #     y: -0.6708334808526397
        #     z: 0.2304237406275697
        #     w: 0.20164184960455844
        # [RobotController] Start planning goal 2
        # [RobotController] Goal 2 completed

        # [RobotController] POSE AT GOAL 2: header: 
        #   seq: 0
        #   stamp: 
        #     secs: 2328
        #     nsecs: 856000000
        #   frame_id: "world"
        # pose: 
        #   position: 
        #     x: -1.1282925605422138
        #     y: 0.11473068914673325
        #     z: 1.1809346559126976
        #   orientation: 
        #     x: -0.5765912283688761
        #     y: -0.7012164259384711
        #     z: 0.38830146561662704
        #     w: 0.1583036675574085
        # [RobotController] Start planning goal 3
        # [RobotController] Goal 3 completed

        # [RobotController] POSE AT GOAL 3: header: 
        #   seq: 0
        #   stamp: 
        #     secs: 2334
        #     nsecs: 885000000
        #   frame_id: "world"
        # pose: 
        #   position: 
        #     x: -1.2147758569354663
        #     y: -0.18469088968210876
        #     z: 1.0194713498220531
        #   orientation: 
        #     x: -0.6879161872862256
        #     y: -0.18965173845372166
        #     z: 0.21605898629211884
        #     w: 0.6664248283291114
        # [RobotController] Start planning goal 4
        # [RobotController] Goal 4 completed

        # [RobotController] POSE AT GOAL 4: header: 
        #   seq: 0
        #   stamp: 
        #     secs: 2339
        #     nsecs: 419000000
        #   frame_id: "world"
        # pose: 
        #   position: 
        #     x: -1.1095279168670267
        #     y: -0.02652976243717884
        #     z: 0.9953543219001667
        #   orientation: 
        #     x: -0.4872294476099591
        #     y: -0.49354118154025933
        #     z: 0.5078530290599778
        #     w: 0.5109891078879707
        # [RobotController] Start planning goal 5
        # [RobotController] Goal 5 completed

        # [RobotController] POSE AT GOAL 5: header: 
        #   seq: 0
        #   stamp: 
        #     secs: 2341
        #     nsecs: 955000000
        #   frame_id: "world"
        # pose: 
        #   position: 
        #     x: -1.1165469842915055
        #     y: 0.1340856699056785
        #     z: 0.9814541459402137
        #   orientation: 
        #     x: -0.34842469358809436
        #     y: -0.5910913303387061
        #     z: 0.6237018000257155
        #     w: 0.37444270154610243
        # [RobotController] Start planning goal 6
        # [RobotController] Goal 6 completed

        # [RobotController] POSE AT GOAL 6: header: 
        #   seq: 0
        #   stamp: 
        #     secs: 2347
        #     nsecs: 190000000
        #   frame_id: "world"
        # pose: 
        #   position: 
        #     x: -1.1396769782524507
        #     y: -0.15019272737497463
        #     z: 0.8743253909205784
        #   orientation: 
        #     x: -0.4558565605690492
        #     y: -0.2322508680798385
        #     z: 0.3973592717310248
        #     w: 0.7618135858802386
        # [RobotController] Start planning goal 7
        # [RobotController] Goal 7 completed

        # [RobotController] POSE AT GOAL 7: header: 
        #   seq: 0
        #   stamp: 
        #     secs: 2348
        #     nsecs: 921000000
        #   frame_id: "world"
        # pose: 
        #   position: 
        #     x: -1.1374895547383916
        #     y: -0.03985902092410588
        #     z: 0.8484701122380741
        #   orientation: 
        #     x: -0.3311110136179093
        #     y: -0.2795823951162415
        #     z: 0.5364328446703883
        #     w: 0.7241815961213194
        # [RobotController] Start planning goal 8
        # [RobotController] Goal 8 completed

        # [RobotController] POSE AT GOAL 8: header: 
        #   seq: 0
        #   stamp: 
        #     secs: 2351
        #     nsecs: 544000000
        #   frame_id: "world"
        # pose: 
        #   position: 
        #     x: -1.1270009995559218
        #     y: 0.11834504569974748
        #     z: 0.844744956223418
        #   orientation: 
        #     x: -0.23917163848626047
        #     y: -0.40975936961710974
        #     z: 0.7547127963406675
        #     w: 0.4531034996492927
        # [RobotController] All goals completed
        # [RobotController] Moving back to home position
        # [RobotController] Moving back to home position DONE
        # [RobotController] DEMO COMPLETE

        


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

    yaml_file = '/home/quanvu/git/arm-module-ur5e/arm_module_ur5e_controller/cfg/arm_module_controller_config.yaml'
    with open(yaml_file, 'r') as file:
        cfg = yaml.safe_load(file)

    try:    
        mp = RobotController(cfg)
    except rospy.ROSInterruptException:
        pass