docker run `
    --name moveit_ur5_container `
    --detach `
    --privileged `
    --gpus all `
    -e DISPLAY=host.docker.internal:0.0 `
    --network host `
    -e ROS_MASTER_URI=http://localhost:11311 `
    -v "C:\PERSONAL DATA ROG\git\simple_ur5_controller:/root/apple_ws/src/simple_ur5_controller" `
    moveit_ur5_image:latest
