# cv_ros 包测试指南

## 概述

cv_ros包包含两个主要的ROS2节点：

1. **红色色块检测节点** (`red_color_detector_ros2.py`)
   - 从摄像头读取视频流
   - 检测红色色块
   - 计算红色色块的中心点坐标
   - 将中心点坐标发布到 `/detection/red_center` 话题

2. **颜色跟踪offboard控制节点** (`color_tracking_offboard.py`)
   - 订阅 `/detection/red_center` 话题
   - 计算无人机的位置偏移
   - 发布轨迹设定点到PX4飞控
   - 控制无人机跟踪红色物体

本指南将指导您如何测试这些节点，从单独测试到集成测试。

## 前提条件

在开始测试之前，请确保：

1. 您已经成功编译了cv_ros包：
   ```bash
   cd ~/cv-ros2-px4/ros_ws/
   colcon build --symlink-install
   ```

2. 您已经配置了ROS2环境：
   ```bash
   source ~/cv-ros2-px4/ros_ws/install/setup.bash
   ```

3. 您已经连接了摄像头（用于红色色块检测）

4. 如果要测试无人机控制功能，您已经设置了PX4飞控和模拟器

## 测试步骤

### 1. 测试红色色块检测节点

红色色块检测节点是一个独立的节点，您可以单独测试它，验证它能否正确检测红色物体并发布中心点坐标。

#### 1.1 运行红色色块检测节点

```bash
cd ~/cv-ros2-px4/ros_ws/
source install/setup.bash
ros2 run cv_ros red_color_detector_ros2.py
```

#### 1.2 验证节点功能

- 您应该看到一个名为"Red Color Detection"的视频窗口
- 当您将红色物体放在摄像头前时，节点应该：
  - 在红色物体周围绘制绿色边框
  - 显示红色物体的面积
  - 在红色物体中心绘制红色圆点
  - 在视频窗口中心显示十字分割线

#### 1.3 检查发布的话题

打开一个新的终端窗口，检查节点是否正确发布了中心点坐标：

```bash
cd ~/cv-ros2-px4/ros_ws/
source install/setup.bash
ros2 topic echo /detection/red_center
```

当检测到红色物体时，您应该看到类似以下的输出：

```
x: 320.0
y: 240.0
z: 0.0
```

#### 1.4 测试节点参数

红色色块检测节点支持以下参数：

| 参数名 | 描述 | 默认值 |
|--------|------|--------|
| camera_id | 摄像头ID | 0 |
| publish_rate | 发布频率（Hz） | 10.0 |
| min_area | 最小色块面积阈值 | 500 |

您可以通过以下方式修改这些参数：

```bash
cd ~/cv-ros2-px4/ros_ws/
source install/setup.bash
ros2 run cv_ros red_color_detector_ros2.py --ros-args -p camera_id:=1 -p min_area:=1000
```

### 2. 测试颜色跟踪offboard控制节点

颜色跟踪offboard控制节点需要与PX4飞控或模拟器配合使用。在测试这个节点之前，您需要确保已经设置了PX4环境。

#### 2.1 运行颜色跟踪offboard控制节点

```bash
cd ~/cv-ros2-px4/ros_ws/
source install/setup.bash
ros2 run cv_ros color_tracking_offboard.py
```

#### 2.2 验证节点功能

- 节点应该订阅 `/detection/red_center` 话题
- 当接收到红色物体的中心点坐标时，节点应该计算位置偏移
- 节点应该发布轨迹设定点到 `/fmu/in/trajectory_setpoint` 话题

#### 2.3 检查节点输出

您可以通过以下方式查看节点的输出：

```bash
cd ~/cv-ros2-px4/ros_ws/
source install/setup.bash
ros2 topic echo /fmu/in/trajectory_setpoint
```

当检测到红色物体时，您应该看到类似以下的输出：

```
position:
- 0.0
- 0.0
- -5.0
yaw: 1.5707963267948966
timestamp: 1234567890
```

### 3. 集成测试

集成测试是同时运行两个节点，测试它们能否协同工作。

#### 3.1 启动ROS2环境

```bash
cd ~/cv-ros2-px4/ros_ws/
source install/setup.bash
```

#### 3.2 运行红色色块检测节点

在第一个终端窗口中运行：

```bash
ros2 run cv_ros red_color_detector_ros2.py
```

#### 3.3 运行颜色跟踪offboard控制节点

在第二个终端窗口中运行：

```bash
ros2 run cv_ros color_tracking_offboard.py
```

#### 3.4 验证集成功能

- 红色色块检测节点应该检测红色物体并发布中心点坐标
- 颜色跟踪offboard控制节点应该接收中心点坐标并计算位置偏移
- 颜色跟踪offboard控制节点应该发布轨迹设定点到PX4飞控

#### 3.5 使用rqt_graph查看节点关系

在第三个终端窗口中运行：

```bash
rqt_graph
```

您应该看到两个节点之间的连接关系：`red_color_detector_ros2.py` 发布到 `/detection/red_center` 话题，`color_tracking_offboard.py` 订阅这个话题。

## 常见问题及解决方案

### 问题1：红色色块检测节点无法打开摄像头

**错误信息：** `无法打开摄像头 0`

**解决方案：**

1. 检查摄像头是否正确连接
2. 检查摄像头权限：
   ```bash
   sudo chmod 666 /dev/video0
   ```
3. 尝试使用不同的摄像头ID：
   ```bash
   ros2 run cv_ros red_color_detector_ros2.py --ros-args -p camera_id:=1
   ```

### 问题2：红色色块检测节点无法检测到红色物体

**解决方案：**

1. 检查红色物体是否足够大（大于 `min_area` 参数）
2. 检查环境光线是否合适
3. 调整红色的HSV范围（在 `red_color_detector_ros2.py` 文件中修改）
4. 增加 `min_area` 参数的值

### 问题3：颜色跟踪offboard控制节点无法接收中心点坐标

**解决方案：**

1. 确保红色色块检测节点正在运行
2. 检查话题名称是否正确：
   ```bash
   ros2 topic list
   ```
3. 检查话题类型是否正确：
   ```bash
   ros2 topic info /detection/red_center
   ```

### 问题4：无人机无法响应控制命令

**解决方案：**

1. 确保PX4飞控正在运行并处于offboard模式
2. 检查飞控的连接状态
3. 检查飞控的参数设置
4. 确保节点正在发布正确的轨迹设定点

## 高级测试

### 使用launch文件启动节点

您可以创建一个launch文件，同时启动两个节点：

```bash
cd ~/cv-ros2-px4/ros_ws/src/cv_ros/
nano launch/cv_ros_launch.py
```

添加以下内容：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cv_ros',
            executable='red_color_detector_ros2.py',
            name='red_color_detector_ros2',
            output='screen'
        ),
        Node(
            package='cv_ros',
            executable='color_tracking_offboard.py',
            name='color_tracking_offboard',
            output='screen'
        )
    ])
```

然后使用以下命令启动节点：

```bash
cd ~/cv-ros2-px4/ros_ws/
ros2 launch cv_ros cv_ros_launch.py
```

### 使用模拟器测试无人机控制

如果您没有实际的无人机，可以使用PX4模拟器测试控制功能：

1. 启动PX4模拟器：
   ```bash
   cd ~/PX4-Autopilot/
   make px4_sitl gazebo
   ```

2. 启动两个节点：
   ```bash
   cd ~/cv-ros2-px4/ros_ws/
   ros2 run cv_ros red_color_detector_ros2.py
   ros2 run cv_ros color_tracking_offboard.py
   ```

3. 观察模拟器中的无人机是否响应控制命令

## 测试结果记录

在测试过程中，您可以记录以下信息：

1. 红色色块检测节点的性能：
   - 检测速度（FPS）
   - 检测准确率
   - 环境适应性

2. 颜色跟踪offboard控制节点的性能：
   - 响应速度
   - 控制稳定性
   - 跟踪精度

3. 集成测试的性能：
   - 系统延迟
   - 协同工作效果
   - 鲁棒性

记录这些信息可以帮助您优化节点的性能和稳定性。

## 总结

本指南介绍了cv_ros包的测试步骤，从单独测试到集成测试。通过这些测试，您可以验证红色色块检测节点和颜色跟踪offboard控制节点的功能，确保它们能正确工作并协同工作。

如果您在测试过程中遇到问题，可以参考"常见问题及解决方案"部分，或者查看节点的源代码进行调试。