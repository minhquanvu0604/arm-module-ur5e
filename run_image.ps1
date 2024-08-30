docker run `
    --name moveit_ur5_container `
    --detach `
    --privileged `
    --gpus all `
    -e DISPLAY=host.docker.internal:0.0 `
    --network host `
    -e ROS_MASTER_URI=http://localhost:11311 `
    -v "C:\PERSONAL DATA ROG\git\arm_module_ur5e:/root/apple_ws/src/arm_module_ur5e" `
    moveit_ur5_image:latest
