# Launching
The launch file structure in this package mimics the UR robot ROS Wrapper ([universal_robot](https://github.com/ros-industrial/universal_robot)) launch files and urdf structure, creating a corresponding version for each of the original one and modify as necessary to include the base, camera, etc.

## Original Structure from UR
This section explains the complex structure of the original package. Understanding it will allow modifications for your own setup.

<details><summary><b>Show Instruction</b></summary>

### **`roslaunch ur_gazebo ur5e_bringup.launch`**
Top-Level Launch File, source code on [GitHub](https://github.com/ros-industrial/universal_robot/blob/noetic-devel/ur_gazebo/launch/ur5e_bringup.launch) 

- **`inc/load_ur5e.launch.xml`**  
  Specifies UR the robot model's specific parameters. It acts as a wrapper for `load_ur.launch.xml`, which is generic for all UR robot models.
  
  - **`load_ur.launch.xml`**  
    Loads the top-level (i.e., stand-alone and complete) Xacro for the UR variant defined by the set of YAML parameter files. This should only be included by a wrapper launch file (like `load_ur5e.launch.xml`).
    
    - **Sets ROS Parameter `robot_description`**  
      Points to `ur_gazebo/urdf/ur.xacro`. This is the top-level file that describes the robot, which is formatted in Xacro and later parsed into URDF. For more details, see the [Details](#ur_gazebourdfurxacro) section below.


- **`robot_state_publisher`**  
  A ROS node that publishes the transforms (TF tree) of the robot based on the joint states. This node is essential for both simulation and real-world operation because it broadcasts the robot's current configuration, allowing:
  - Visualization in RViz
  - Integration with other ROS tools

- **`inc/ur_control.launch.xml`**  
  This file configures the control interfaces and plugins used to control the UR5e robot in ROS. It sets up controllers such as joint trajectory controllers, which allow for smooth motion planning and execution using MoveIt or other control packages.


### **`ur_gazebo/urdf/ur.xacro`** 
Top-Level Robot Description File, source code on [GitHub](https://github.com/ros-industrial/universal_robot/blob/noetic-devel/ur_gazebo/urdf/ur.xacro).

It contains a Gazebo-specific variant of the file with the same name in the ur_description package - same arguments, but instead of configuring everything for a real robot, it generates a Gazebo-compatible URDF with a ros_control hardware_interface attached to it.

All the UR-robot-variant-specific parameters are passed from the upper-level launch files. 

**This file is not meant to be edited.** To modify your robot model:
1. Create a new top-level Xacro file.
2. Include other required `.xacro` files that define the macros (e.g., UR robot macro **`ur_macro.xacro`** and other macros for your camera, base, etc.).
3. In the new top-level file, instantiate the models (i.e., call the macros) and connect everything by adding the appropriate joints.

The file includes:

- **`ur_macro.xacro`**  
  This wraps the model of the real robot and adds all elements and parameters required by Gazebo. It receives parameters from the higher-level file that includes it, to specify the UR robot variant.
  
  - **`ur_description/urdf/inc/ur_macro.xacro`**  
    This file instantiates the model for the **real robot**. By convention, **`ur_description`** is the description package of the physical aspects of the real robot.
  
  - **Additional Configuration for Gazebo**  
    Additional configurations make the real robot model work in Gazebo, such as:
    - **Self-collision properties** per link.
    - **`gazebo_ros_control` plugin**:  
      Enables control of the robot using ROS controllers within the Gazebo environment. It connects ROS control messages to the simulated robot, allowing it to respond to commands. You'll often see `EffortJointInterface` or `PositionJointInterface` used to define how the robot's joints should be controlled.

</details>


## Launching UR5 model with base
Modify launch files and urdf to add the base of the robot arm (in UTS:RI lab it is a pillar)

```bash
roslaunch arm_module_gazebo arm_module_ur5e_bringup.launch
```
Each file is equivalent with one from the original UR robot package for ease for understanding. Check out the original structure [above](#original-structure-from-ur)

arm_module_gazebo/arm_module_ur5e_bringup.launch - equivalent to ur_gazebo/ur5e_bringup.launch
- inc/load_ur5e_pillar_camera.launch.xml - equivalent to inc/load_ur5e.launch.xml
  - inc/load_ur_pillar_camera.launch.xml - equivalent to inc/load_ur.launch.xml, it includes robot description file arm_module_gazebo/urdf/ur_gazebo_pillar_camera.xacro, description below
- robot_state_publisher - same as the original package
- ur_gazebo/launch/inc/ur_control.launch.xml - same as the original package

arm_module_gazebo/urdf/ur_gazebo_pillar_camera.xacro - equivalent to ur.xacro
- ur_gazebo_pillar_camera_macro



## Launching only the UR5 
This is the default launch set up provided by the original universal_robot package
### Launching Real Hardware
Start the robot driver. To check robot IP, use the teach pendant, go to Settings/System/Network

```bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.169
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

Setting up the MoveIt! nodes to allow motion planning

```bash
roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true
```

Starting up RViz with a configuration including the MoveIt! Motion Planning plugin

```bash
roslaunch ur5e_moveit_config moveit_rviz.launch
```

Launch the test node to go through a set of waypoints in cfg/list_poses.json. First cd to arm_module_ur5e/python/scripts/

```bash
python3 run_robot_controller.py
```

