docker run \
    --name moveit_ur5_container \
    --detach \
    --privileged \
    -e DISPLAY \
    --network host \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -v /dev:/dev \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -v /home/$(whoami)/ros/apple_ws:/root/apple_ws \
    moveit_ur5_image:latest 

# -v /home/git/arm_module_ur5e:/root/apple_ws/src/arm_module_ur5e \