#!/bin/bash
# 测试修复后的颜色跟踪系统

# 设置ROS2环境
source /opt/ros/jazzy/setup.bash
source ~/cv-ros2-px4/ros_ws/install/setup.bash

# 运行修复后的节点
echo "启动红色色块检测节点..."
python3 ~/cv-ros2-px4/ros_ws/src/cv_ros/cv_ros/red_color_detector_ros2.py --ros-args -p camera_id:=1 -p publish_rate:=30 -p headless:=true &

# 等待节点启动
sleep 2

echo "启动颜色跟踪offboard控制节点..."
python3 ~/cv-ros2-px4/ros_ws/src/cv_ros/cv_ros/color_tracking_offboard.py

# 清理后台进程
kill %1