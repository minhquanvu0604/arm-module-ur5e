# Launching UR5 model with base
To be done: Modify urdf to add the base

# Launching only the UR5 
## Launching Real Hardware
Start the robot driver. To check robot IP, use the teach pendant, go to Settings/System/Network

```roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.169```

Launch UR5e MoveIt config

```roslaunch ur5e_moveit_config moveit_planning_execution.launch```

Starting up RViz with a configuration including the MoveIt! Motion Planning plugin

```roslaunch ur5e_moveit_config moveit_rviz.launch```


## Launching Simulation
Bring up the simulated robot in Gazebo

```roslaunch ur_gazebo ur5e_bringup.launch```

Setting up the MoveIt! nodes to allow motion planning

```roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true```

Starting up RViz with a configuration including the MoveIt! Motion Planning plugin

```roslaunch ur5e_moveit_config moveit_rviz.launch```

Launch the test node to go through a set of waypoints in cfg/list_poses.json. First cd to simple_ur5_controller/python/scripts/

```python3 reach_waypoints.py```