# 颜色跟踪系统启动指南

## 系统概述
这是一个基于ROS2和PX4的无人机颜色跟踪系统启动步骤指南。

## 启动前准备

### 1. 启动Micro XRCE-DDS Agent
连接飞控与地面站的通信桥梁：

```bash
sudo MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

### 2. 飞控硬件上电
将无人机飞控系统上电，确保硬件连接正常。

## ROS2节点启动

### 3. 配置ROS2环境
加载项目的ROS2工作空间环境变量：

```bash
source ~/cv-ros2-px4/ros_ws/install/setup.bash
```

### 4. 启动色块识别节点
启动红色色块检测节点，用于识别摄像头中的红色物体：

```bash
python ~/cv-ros2-px4/ros_ws/src/cv_ros/cv_ros/red_color_detector_ros2.py --ros-args \
    -p camera_id:=0 \
    -p publish_rate:=30 \
    -p headless:=true
```

**参数说明：**
- `camera_id`: 摄像头设备ID（默认为0）
- `publish_rate`: 检测结果发布频率（Hz）
- `headless`: 是否启用无头模式（不显示图像界面）

### 5. 启动色块跟踪节点
启动颜色跟踪Offboard控制节点，用于控制无人机跟踪识别到的红色物体：

```bash
python ~/cv-ros2-px4/ros_ws/src/cv_ros/cv_ros/color_tracking_offboard.py
```

## 操作流程

1. 按照上述步骤依次启动各个组件
2. 使用遥控器将无人机设置为Offboard模式
3. 确保无人机已经起飞并处于飞行状态
4. 将红色物体放置在摄像头视野范围内
5. 观察无人机是否能够成功跟踪红色物体

## 注意事项

- 确保所有硬件连接稳定可靠
- 首次使用前请在安全环境下进行测试
- 飞行过程中始终保持遥控器在手动模式的控制范围内
- 如遇异常情况，立即切换回手动模式控制无人机