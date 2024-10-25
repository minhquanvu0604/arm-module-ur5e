import os, sys
arm_module_ur5e_controller_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')) # arm_module_ur5e_controller
sys.path.insert(0, arm_module_ur5e_controller_path)

import math
import threading
import time
from enum import Enum
import yaml

import rospy
from std_srvs.srv import Trigger, TriggerResponse
# import tf2_ros
# import tf.transformations
# from sensor_msgs.msg import JointState

from arm_module_ur5e_controller.srv import MoveToPose, MoveToPoseResponse
from arm_module_ur5e_controller.srv import PoseService, PoseServiceResponse

from cfg.config import CONTROLLER_CONFIG_PATH, WAYPOINT, JOINT_PATH_DEG

from python.src.UR5e import ArmModuleUR5e
# from python.src.collision_manager import CollisionManager
from python.src.utility import read_waypoints_rpy, read_joint_path
# from geometry_msgs.msg import Pose


class ArmState(Enum):
    IDLE = 0
    RUNNING = 1

class RobotController:
    """
    To get sim groundtruth pose: rosrun tf tf_echo world tool0
    """
    RATE = 10  # Hz
    # Joint values guess - aid the solver to come up with elegant solutions - this one is for MVPS Apple Picking
    HOME_JOINT_RAD = [-1.7973400948921046, -0.7555930298280389, 1.455735132902352, 2.401335236663474, -1.3445789828512975, 0.0020295319930268008]

    def __init__(self, cfg) -> None:

        rospy.init_node("arm_module_ur5e", log_level=rospy.INFO, anonymous=True)
        rospy.loginfo("Initialising RobotController")

        # Configs
        mode = cfg['mode']
        query_current_pose_service = cfg['query_current_pose_ros_service']
        # publish_state = cfg['publish_state']

        self._state = ArmState.IDLE
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
        self._shutdown_servie = rospy.Service('/mvps/arm_module/shutdown', Trigger, self.shutdown_callback)

        # joints = self.robot.group.get_current_joint_values()
        # rospy.loginfo(f"Starting joint values: {joints}")
        # raise Exception("Stop here")

        # Move to home position
        rospy.loginfo("Moving to home position")
        self.robot.go_to_joint_goal_rad(RobotController.HOME_JOINT_RAD, wait=True)
        rospy.loginfo("Moved to home position")
        
        # exit(0)

        if query_current_pose_service:
            self._pose_service = rospy.Service('/mvps/arm_module/query_data', PoseService, self._query_pose_callback)
            rospy.loginfo("Service to query current pose is ready.")

        # if publish_state:
        #     self._state_publisher = rospy.Publisher('mvps/arm_module/state', ArmState, queue_size=10)
        #     rospy.loginfo("State publisher is ready.")

        if mode == 1:
            self._control_thread = threading.Thread(target=self._execute_demo_joint_list)
            self._control_thread.start()
        elif mode == 2:
            self._move_service = rospy.Service('mvps/arm_module/pose', MoveToPose, self._move_to_pose_callback)
            rospy.loginfo("Service to receive goal pose is ready. Waiting for service call to supply goal pose.")   
        elif mode == 3:
            self._next_pose_setup()
            self._next_pose_service = rospy.Service('mvps/arm_module/next_pose', Trigger, self._next_pose_callback)
            rospy.loginfo("Service to go to next pose is ready. Waiting for service call to signal moving to next pose.")
        else:
            rospy.logerr("Invalid mode selected. Exiting...")
            exit(1)

        # ROS will keep spinning until the node is shutdown with Ctrl+C
        rospy.spin()
        rospy.loginfo("Shutting down Arm Module")


    def _execute_demo_pose_list(self) -> None:
        waypoints = read_waypoints_rpy(WAYPOINT)    
        goal_id = 0
        for pose_goal in waypoints:
            rospy.loginfo(f"Start planning goal {goal_id}")
            # self.robot.go_to_pose_goal(pose, None, parent_frame_id)
            self.robot.go_to_pose_goal_simple(pose_goal)
            rospy.loginfo(f"Goal {goal_id} completed\n")
            goal_id += 1
        rospy.loginfo("DEMO COMPLETE")

    def _execute_demo_joint_list(self) -> None:
        joint_path = read_joint_path(JOINT_PATH_DEG)
        joint_path = [[math.radians(joint) for joint in config] for config in joint_path] # convert to rad
        goal_id = 0
        self._state = ArmState.RUNNING

        for joint_goal in joint_path:
            rospy.loginfo(f"Start planning goal {goal_id}")
            self.robot.go_to_joint_goal_rad(joint_goal)

            # DEBUGGING
            current_pose = self.robot.group.get_current_pose().pose # PoseStamped -> Pose
            rospy.loginfo(f"POSE AT GOAL {goal_id}: {current_pose}")
            # self.robot.visualize_target_pose(current_pose)

            rospy.loginfo(f"Goal {goal_id} completed\n")
            goal_id += 1

            input("Press Enter to continue...")

        rospy.loginfo("All goals completed")
        
        # Move to home position
        rospy.loginfo("Moving back to home position")
        self.robot.go_to_joint_goal_rad(RobotController.HOME_JOINT_RAD, wait=True)
        self._state = ArmState.IDLE

        rospy.loginfo("Moving back to home position DONE")
        rospy.loginfo("DEMO COMPLETE")
        rospy.loginfo("Press Ctrl+C to stop the program...")

    def _move_to_pose_callback(self, req):
        self._state = ArmState.RUNNING
        pose_goal = req.pose
        # rospy.loginfo("Starting to move to the goal pose")
        success = self.robot.go_to_pose_goal_simple(pose_goal, wait=True)
        # rospy.loginfo("Finished moving to the goal pose")
        self._state = ArmState.IDLE
        response = MoveToPoseResponse()
        response.success = success
        return response

    def _next_pose_setup(self):
        self.joint_path = read_joint_path(JOINT_PATH_DEG)
        self.joint_path = [[math.radians(joint) for joint in config] for config in self.joint_path] # convert to rad
        self.current_goal = 0

    def _next_pose_callback(self, req):
        rospy.logdebug(f"Next pose service is queried")
        response = TriggerResponse()
        if self.current_goal < len(self.joint_path):
            self._state = ArmState.RUNNING
            
            joint_goal = self.joint_path[self.current_goal]

            self.robot.go_to_joint_goal_rad(joint_goal, wait=True)
            self.current_goal += 1
            rospy.loginfo(f"Moved to goal {self.current_goal}")

            self._state = ArmState.IDLE
            response.success = True
            return response
        else:
            rospy.loginfo("All goals completed")
            response.success = False
            return response

    def _query_pose_callback(self, req):
        rospy.loginfo("Pose is queried")
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
    
    def shutdown_callback(self, req):
        rospy.loginfo("Service call to shutdown the node")

        while self._state == ArmState.RUNNING:
            rospy.loginfo("Waiting for the robot to finish the current task...")
            time.sleep(1) 

        rospy.signal_shutdown("Requested shutdown.")
        response = TriggerResponse()
        response.success = True
        return response

    def cleanup(self) -> None:
        rospy.logdebug("Cleaning up")
        # self._safety_thread.join()
        self.robot.shutdown()
        # self.collisions.remove_collision_object()
        rospy.logdebug("Clean-up completed")


if __name__ == "__main__":

    with open(CONTROLLER_CONFIG_PATH, 'r') as file:
        cfg = yaml.safe_load(file)

    try:    
        mp = RobotController(cfg)  
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted by user")

    #               OUTPUT OBTAINED FROM FOLLOWING near_q_list_sim_deg.yaml
    # POSE AT GOAL 0: header: 
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
    # Start planning goal 1
    # Goal 1 completed

    # POSE AT GOAL 1: header: 
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
    # Start planning goal 2
    # Goal 2 completed

    # POSE AT GOAL 2: header: 
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
    # Start planning goal 3
    # Goal 3 completed

    # POSE AT GOAL 3: header: 
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
    # Start planning goal 4
    # Goal 4 completed

    # POSE AT GOAL 4: header: 
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
    # Start planning goal 5
    # Goal 5 completed

    # POSE AT GOAL 5: header: 
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
    # Start planning goal 6
    # Goal 6 completed

    # POSE AT GOAL 6: header: 
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
    # Start planning goal 7
    # Goal 7 completed

    # POSE AT GOAL 7: header: 
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
    # Start planning goal 8
    # Goal 8 completed

    # POSE AT GOAL 8: header: 
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
    # All goals completed
    # Moving back to home position
    # Moving back to home position DONE
    # DEMO COMPLETE
