#!/bin/bash

rosrun xacro xacro $(rospack find arm_module_gazebo)/urdf/ur_gazebo_pillar_camera.xacro \
  robot_model:=ur5e \
  joint_limit_params:=$(rospack find ur_description)/config/ur5e/joint_limits.yaml \
  kinematics_params:=$(rospack find ur_description)/config/ur5e/default_kinematics.yaml \
  physical_params:=$(rospack find ur_description)/config/ur5e/physical_parameters.yaml \
  visual_params:=$(rospack find ur_description)/config/ur5e/visual_parameters.yaml \
  transmission_hw_interface:=hardware_interface/EffortJointInterface \
  safety_limits:=false \
  safety_pos_margin:=0.15 \
  safety_k_position:=20 > generated_from_ur_gazebo_pillar_camera.urdf
