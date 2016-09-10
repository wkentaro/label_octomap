#!/usr/bin/env bash

mkdir -p ~/.ros/icra2017

rosbag record -o ~/.ros/icra2017/look_around_bin \
  /tf \
  /robot/joint_states \
  /right_hand_camera/depth_registered/camera_info /right_hand_camera/depth_registered/image_raw \
  /right_hand_camera/rgb/camera_info /right_hand_camera/rgb/image_raw \
  /passthrough_cloud/output
