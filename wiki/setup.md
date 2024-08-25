# Dependency
You can either install dependencies directly to your host OS using apt, or check Docker section [below](#docker)
- ROS Noetic - Ubuntu 20.04

- Control real hardware - [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
```bash
sudo apt install ros-noetic-ur-robot-driver
```

- Control in Gazebo Simulation - ur_gazebo
```bash
sudo apt-get install ros-noetic-ur-gazebo
```

- MoveIt!
```bash
sudo apt install ros-noetic-moveit
```

- MoveIt! Config
```bash
sudo apt-get install ros-noetic-ur5e-moveit-config
```

# Docker 
Allowing the root user to connect to X server - for applications inside Docker container to display GUI 
```bash
xhost +local:root
```
Build Docker image
```bash
docker build -t moveit_ur5_image -f docker/Dockerfile .
```
Run the container. You will need to modify the bind mounting directory
```bash
./run_image.sh
```
Open a iteractive shell inside the container (the ROS workspace inside the container is named apple_ws)
```bash
docker exec -it moveit_ur5_container /bin/bash

# docker exec -it moveit_ur5_container bash -c "source /opt/ros/noetic/setup.bash && source /root/apple_ws/devel/setup.bash && exec bash"
```
Run the program with the above intructions as you normall do in you host OS