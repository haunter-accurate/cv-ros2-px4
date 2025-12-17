# 飞行实验准备与方向对应关系说明

## 1. 节点功能概述

### 1.1 红色色块检测节点 (red_color_detector_ros2.py)

- **功能**：通过摄像头实时检测红色色块，计算并发布色块中心点坐标
- **发布话题**：`/detection/red_center` (类型: Point)
- **参数**：
  - `camera_id`: 摄像头ID (默认: 0)
  - `publish_rate`: 发布频率 (默认: 10Hz)
  - `min_area`: 最小色块面积阈值 (默认: 500)
  - `headless`: 是否启用无头模式 (默认: False)

### 1.2 颜色跟踪Offboard控制节点 (color_tracking_offboard.py)

- **功能**：订阅红色色块中心点坐标，计算无人机目标位置并发送控制指令
- **订阅话题**：`/detection/red_center`、`/fmu/out/vehicle_local_position`、`/fmu/out/vehicle_status`
- **发布话题**：`/fmu/in/offboard_control_mode`、`/fmu/in/trajectory_setpoint`

## 2. 方向对应关系说明

### 2.1 坐标系定义

- **PX4无人机NED坐标系**：
  - x轴：向前
  - y轴：向右
  - z轴：向下 (高度为负值)

- **摄像头图像坐标系**：
  - x轴：水平向右
  - y轴：垂直向下
  - 原点：左上角

### 2.2 方向对应关系 (摄像头朝向地面)

| 红色色块位置 | 像素误差 | 无人机移动方向 |
|--------------|----------|----------------|
| 图像中心上方 | error_y < 0 | 向前 (x轴正方向) |
| 图像中心下方 | error_y > 0 | 向后 (x轴负方向) |
| 图像中心左侧 | error_x < 0 | 向左 (y轴负方向) |
| 图像中心右侧 | error_x > 0 | 向右 (y轴正方向) |

### 2.3 坐标转换逻辑

```python
# 红色色块检测节点发布的是像素坐标
# center_msg.x: 红色色块中心的x像素坐标
# center_msg.y: 红色色块中心的y像素坐标

# 颜色跟踪节点计算像素误差
error_x = self.color_center.x - (self.image_width / 2)
error_y = self.color_center.y - (self.image_height / 2)

# 转换为无人机位置偏移
# 图像X轴 -> 无人机Y轴 (左右方向)
offset_y = error_x * self.control_gain
# 图像Y轴 -> 无人机X轴 (前后方向)
offset_x = -error_y * self.control_gain

# 计算目标位置
target_x = current_x + offset_x  # 前后方向
target_y = current_y + offset_y  # 左右方向
target_z = -5.0                  # 固定高度 (NED坐标系)
```

## 3. 飞行实验操作流程

### 3.1 准备工作

1. **启动摄像头**：确保摄像头已正确安装并朝向地面
2. **连接无人机**：确保无人机已连接到飞控和地面站
3. **启动ROS2环境**：
   ```bash
   source /opt/ros/foxy/setup.bash
   source ~/cv-ros2-px4/ros_ws/install/setup.bash
   ```

### 3.2 启动节点

1. **启动红色色块检测节点**：
   ```bash
   ros2 run cv_ros red_color_detector_ros2 --ros-args -p camera_id:=1 -p headless:=true
   ```

2. **启动颜色跟踪控制节点**：
   ```bash
   ros2 run cv_ros color_tracking_offboard
   ```

### 3.3 飞行实验步骤

1. **手动起飞**：使用遥控器手动起飞无人机到合适高度
2. **切换到OFFBOARD模式**：通过遥控器切换到OFFBOARD模式
3. **观察控制效果**：
   - 无人机将开始跟踪红色色块
   - 移动红色色块，观察无人机的跟随效果
4. **退出OFFBOARD模式**：通过遥控器切换回手动模式
5. **手动降落**：使用遥控器手动降落无人机

## 4. 安全注意事项

1. **速度限制**：无人机最大飞行速度已限制在 0.5 米/秒
2. **高度固定**：无人机将保持约 5 米的固定高度 (NED坐标系中的-5.0米)
3. **紧急情况**：
   - 立即通过遥控器切换出OFFBOARD模式
   - 必要时执行紧急停止操作
4. **测试环境**：
   - 在空旷的户外环境进行测试
   - 确保周围没有人员和障碍物

## 5. 调试与监控

### 5.1 查看节点状态

```bash
# 查看节点列表
ros2 node list

# 查看话题列表
ros2 topic list

# 查看色块中心点信息
ros2 topic echo /detection/red_center

# 查看轨迹设定点信息
ros2 topic echo /fmu/in/trajectory_setpoint
```

### 5.2 调整参数

可以通过修改 `color_tracking_offboard.py` 文件中的参数来调整控制效果：

```python
# 控制参数
self.target_altitude = -5.0  # 目标高度（米）
self.control_gain = 0.01  # 控制增益（像素到米的转换系数）
self.max_offset = 1.0  # 最大位置偏移（米）
max_speed = 0.5  # 最大速度（米/秒）
```

## 6. 预期效果

当无人机处于OFFBOARD模式且已经起飞时：
- 红色色块在图像上方 -> 无人机向前移动
- 红色色块在图像下方 -> 无人机向后移动
- 红色色块在图像左侧 -> 无人机向左移动
- 红色色块在图像右侧 -> 无人机向右移动

无人机将以较低速度 (0.5米/秒) 跟踪红色色块，便于观察和控制。