#!/bin/bash
# 测试修复后的红色色块检测节点

# 设置ROS2环境
source /opt/ros/jazzy/setup.bash
source ~/cv-ros2-px4/ros_ws/install/setup.bash

# 运行修复后的红色色块检测节点，使用整数publish_rate参数
echo "测试修复后的红色色块检测节点..."
python3 ~/cv-ros2-px4/ros_ws/src/cv_ros/cv_ros/red_color_detector_ros2.py --ros-args -p camera_id:=1 -p publish_rate:=30 -p headless:=true