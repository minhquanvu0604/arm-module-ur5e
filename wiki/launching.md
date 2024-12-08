# Launching
The launch file structure in this package mimics the UR robot ROS Wrapper ([universal_robot](https://github.com/ros-industrial/universal_robot)) launch files and urdf structure, creating a corresponding version for each of the original one and modify as necessary to include the base, camera, etc.

# Launching Robot
## 1.1 UR5 model with Base
### Gazebo
Modify launch files and urdf to add the base of the robot arm (in UTS:RI lab it is a pillar)

If you include the pointcloud_normals nodelet, you need to build and source the MVPS package
```bash
roslaunch arm_module_gazebo arm_module_ur5e_bringup.launch
```

Starting up RViz with a configuration including the MoveIt! Motion Planning plugin
```bash
roslaunch arm_module_ur5e_moveit_config moveit_planning_execution.launch sim:=true
```

### Real Hardware
Configure the UR software in the teach pendant
- Find your computer's IP
- Add that IP to Installation/URCap
- Add new program in Program/URCap

Start the robot driver. To check robot IP, use the teach pendant, go to Settings/System/Network
```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=150.22.0.93 robot_description_file:=$(rospack find arm_module_gazebo)/launch/inc/load_ur5e_pillar_camera.launch.xml
```

Launch UR5e MoveIt config
```bash
roslaunch arm_module_ur5e_moveit_config moveit_planning_execution.launch
```

## 1.2 UR5 without Base (Alternative)
This is the default launch set up provided by the original universal_robot package

<details><summary><b>Show</b></summary>

### Gazebo
Bring up the simulated robot in Gazebo
```bash
roslaunch ur_gazebo ur5e_bringup.launch
```

### Real Hardware
Similar to launching with the Base, now without supplying the urdf for robot_description
```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.169
```

Launch UR5e MoveIt config
```bash
roslaunch arm_module_ur5e_moveit_config moveit_rviz.launch
```

### Motion Planning
Turn on the MoveIt! nodes to allow motion planning
```bash
roslaunch arm_module_ur5e_moveit_config moveit_planning_execution.launch sim:=true
```

Start up RViz with a configuration including the MoveIt! Motion Planning plugin
```bash
roslaunch arm_module_ur5e_moveit_config moveit_rviz.launch
```
</details>

## 2. Launch the controller
Choose the demo mode in `arm_module_ur5e_controller/cfg/arm_module_controller_config.yaml`
- Mode 1: Controller script acts as a service server. Use service call to supply goal pose to it
- Mode 2: Launch the test node to go through a set of waypoints in `cfg/list_poses.json`
- Mode 3: MVPS integration. Read a list of poses from `cfg/near_q_list_sim_deg.yaml`, reach goal poses one by one,
use service call to signal moving next pose of that list

```bash
rosrun arm_module_ur5e_controller run_robot_controller.py
```

## Launch Camera Module only (Testing Mode)
Includes the RealSense camera looking at a chair 3D model
```bash
roslaunch arm_module_gazebo test_realsense.launch
```

## Launching UR5e + Base (Dinh Bach)
Use this launch file instead of arm_module_ur5e_bringup.launch, the other launch files remain the same
```bash
roslaunch arm_module_gazebo ur5e_base_camera.launch
```