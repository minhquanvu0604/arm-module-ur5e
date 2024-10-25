# Arm Module 
- [arm_module_ur5e_controller](wiki/arm_module_controller_readme.md)
- arm_module_ur5e_description
- arm_module_gazebo
- arm_module_ur5e_moveit_config

## arm_module_ur5e_controller
## arm_module_ur5e_description
The robot URDF here is not used now. Robot description is published by arm_module_gazebo. Will need to enable load_robot_description in move_group.launch to add scene data if using real robot. 

## arm_module_gazebo
## arm_module_ur5e_moveit_config

## Launching
Check out the [launch instructions](wiki/launching.md)


## Depth Image Processing for Gazebo
The realsense_gazebo_plugin outputs the following topics:
- /arm_module_camera/color/image_raw - RGB Image
- /arm_module_camera/depth/image_raw - Depth Image
- /arm_module_camera/color/camera_info and /arm_module_camera/depth/camera_info - Camera Calibration Info
  
### Image Processing Pipeline
- **RGB Image Rectification - image_proc/rectify**
    - Input: 
      - /arm_module_camera/color/image_raw - Raw RGB image
      - /arm_module_camera/color/camera_info - RGB camera info
    - Output: 
      - /arm_module_camera/color/image_rect_color - Rectified color image
- **Depth Image Registration - depth_image_proc/register**
    - Input: 
      - /arm_module_camera/depth/image_raw - Raw depth image
      - /arm_module_camera/color/camera_info - RGB camera info
    - Output: 
      - /arm_module_camera/aligned_depth_to_color/image_raw - Aligned depth image
- **Point Cloud Generation - depth_image_proc/point_cloud_xyzrgb**
  - Input: 
    - /arm_module_camera/color/image_rect_color - Rectified RGB image
    - /arm_module_camera/aligned_depth_to_color/image_raw - Aligned depth image
    - /arm_module_camera/color/camera_info - RGB camera info
  - Output: 
    - /arm_module_camera/depth_registered/points - XYZRGB point cloud

    <arg name="world_name" value="$(find arm_module_gazebo)/worlds/apple_trellis.world"/> <!-- World file with the apple trellis -->