#!/bin/bash
set -e

# Source the ROS setup script
source /opt/ros/noetic/setup.bash
source /root/apple_ws/devel/setup.bash  # Adjust this path if necessary

# Execute the command passed to the entrypoint
exec "$@"
