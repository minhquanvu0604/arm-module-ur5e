import os, sys

ARM_MODULE_UR5E_CONTROLLER_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..')) # arm_module_ur5e_controller
CONFIG_PATH = os.path.join(ARM_MODULE_UR5E_CONTROLLER_PATH, 'cfg')


CONTROLLER_CONFIG_PATH = os.path.join(CONFIG_PATH, 'arm_module_controller_config.yaml')
WAYPOINT = os.path.join(CONFIG_PATH, 'list_poses_ur5e_demo.json') # List of random poses initially picked to test MoveIt
JOINT_PATH_DEG = os.path.join(CONFIG_PATH, 'near_q_list_sim_deg.yaml') # List of joint values in 9 configs for each near/far scenario
