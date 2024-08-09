FROM moveit/moveit:noetic-source

RUN apt-get update && apt-get install -y \
    ros-noetic-ur-robot-driver \
    ros-noetic-ur-gazebo \
    ros-noetic-moveit \
    ros-noetic-ur5e-moveit-config \
    && rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

WORKDIR /root/apple_ws
CMD ["/bin/bash"]