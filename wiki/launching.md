# Launching
The launch file structure in this package mimics the UR robot ROS Wrapper ([universal_robot](https://github.com/ros-industrial/universal_robot)) launch files and urdf structure, creating a corresponding version for each of the original one and modify as necessary to include the base, camera, etc.

## Launching UR5 model with base
Modify launch files and urdf to add the base of the robot arm (in UTS:RI lab it is a pillar)

If you include the pointcloud_normals nodelet, you need to build and source the MVPS package
```bash
roslaunch arm_module_gazebo arm_module_ur5e_bringup.launch
```

Visualising the perception system in RViz
```bash
# To be moved to description
roslaunch arm_module_gazebo arm_module_perception_rviz.launch
```

Turn on the MoveIt! nodes to allow motion planning
```bash
roslaunch arm_module_ur5e_moveit_config moveit_planning_execution.launch sim:=true
```

## Launching only the UR5 
This is the default launch set up provided by the original universal_robot package
### Launching Real Hardware
Start the robot driver. To check robot IP, use the teach pendant, go to Settings/System/Network
```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.169
# or
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=150.22.0.93 robot_description_file:=$(rospack find arm_module_gazebo)/launch/inc/load_ur5e_pillar_camera.launch.xml
```

Launch UR5e MoveIt config
```bash
roslaunch arm_module_ur5e_moveit_config moveit_planning_execution.launch
```

Starting up RViz with a configuration including the MoveIt! Motion Planning plugin
```bash
roslaunch arm_module_ur5e_moveit_config moveit_rviz.launch
```
- Find your computer's IP
- Add that IP to Installation/URCap
- Add new program in Program/URCap


### Launching Simulation
Bring up the simulated robot in Gazebo
```bash
roslaunch ur_gazebo ur5e_bringup.launch
```

Turn on the MoveIt! nodes to allow motion planning
```bash
roslaunch arm_module_ur5e_moveit_config moveit_planning_execution.launch sim:=true
```

Start up RViz with a configuration including the MoveIt! Motion Planning plugin
```bash
roslaunch arm_module_ur5e_moveit_config moveit_rviz.launch
```

Launch the test node to go through a set of waypoints in cfg/list_poses.json. First cd to arm_module_ur5e/python/scripts/
```bash
python3 run_robot_controller.py
```

