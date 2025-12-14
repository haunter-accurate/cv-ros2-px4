# 验证颜色跟踪Offboard控制节点

以下是在树莓派上验证 `color_tracking_offboard.py` 控制节点的步骤：

## 1. 启动必要的节点

### 1.1 启动红色色块检测节点

首先启动红色色块检测节点来提供跟踪目标：

```bash
# 进入工作空间
source ~/cv-ros2-px4/ros_ws/install/setup.bash

# 启动红色色块检测节点（在终端1中）
ros2 run cv_ros red_color_detector_ros2
```

### 1.2 启动颜色跟踪控制节点

在新的终端中启动控制节点：

```bash
# 进入工作空间
source ~/cv-ros2-px4/ros_ws/install/setup.bash

# 启动颜色跟踪控制节点（在终端2中）
ros2 run cv_ros color_tracking_offboard
```

## 2. 基本功能验证

### 2.1 检查节点是否正常运行

在新的终端中执行以下命令：

```bash
source ~/cv-ros2-px4/ros_ws/install/setup.bash

# 查看所有活动节点
ros2 node list
```

**预期输出**：应该能看到 `/red_color_detector_ros2` 和 `/color_tracking_offboard` 节点

### 2.2 检查话题连接情况

执行以下命令查看话题连接：

```bash
# 查看话题连接情况
ros2 topic info /detection/red_center
```

**预期输出**：应该显示有一个发布者（红色检测节点）和一个订阅者（控制节点）

```
Type: geometry_msgs/msg/Point
Publisher count: 1
Subscriber count: 1
```

### 2.3 检查控制指令话题

执行以下命令查看控制节点发布的话题：

```bash
# 查看所有话题
ros2 topic list | grep fmu
```

**预期输出**：应该能看到以下话题（这些是PX4飞控的控制话题）：
```
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
/fmu/out/vehicle_local_position
/fmu/out/vehicle_status
```

## 3. 验证消息传递

### 3.1 检查色块坐标是否正确传递

在终端中执行以下命令查看控制节点的日志输出：

```bash
# 查看控制节点的日志
ros2 node info /color_tracking_offboard
```

或者使用`rqt_console`查看更详细的日志：

```bash
# 启动rqt_console查看日志
rqt_console
```

**预期输出**：应该能看到控制节点接收到色块坐标的日志信息，例如：
```
[INFO] [1630000000.123456789] [color_tracking_offboard]: 接收到颜色中心: (320.0, 240.0)
```

### 3.2 验证控制指令是否正确发布

#### 3.2.1 检查offboard控制模式消息

执行以下命令查看offboard控制模式消息：

```bash
ros2 topic echo /fmu/in/offboard_control_mode
```

**预期输出**：应该显示类似以下内容（position设为true，其他设为false）：
```
position: true
velocity: false
acceleration: false
attitude: false
body_rate: false
timestamp: 1630000000123456789
```

#### 3.2.2 检查轨迹设定点消息

执行以下命令查看轨迹设定点消息：

```bash
ros2 topic echo /fmu/in/trajectory_setpoint
```

**预期输出**：当无人机处于offboard模式时，应该显示类似以下内容（包含计算后的目标位置）：
```
position:
- 1.23456789
- 2.34567890
- -5.0
yaw: 1.57079
velocity: []
acceleration: []
jerk: []
thrust: []
timestamp: 1630000000123456789
```

### 3.3 验证位置计算逻辑

可以通过调整红色物体在摄像头中的位置，观察控制节点计算的目标位置变化：

1. 将红色物体移到摄像头画面左侧
2. 查看轨迹设定点的x坐标变化（应该减小）
3. 将红色物体移到摄像头画面右侧
4. 查看轨迹设定点的x坐标变化（应该增加）
5. 将红色物体移到摄像头画面上方
6. 查看轨迹设定点的y坐标变化（应该减小）
7. 将红色物体移到摄像头画面下方
8. 查看轨迹设定点的y坐标变化（应该增加）

## 4. 不连接真实无人机的模拟测试

如果没有连接真实无人机，可以使用以下方法进行模拟测试：

### 4.1 使用模拟数据发布器

可以创建一个简单的模拟发布器来替代PX4飞控的位置信息：

```bash
# 创建一个临时的模拟位置发布器脚本
cat > /tmp/simulate_position.py << 'EOF'
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

class SimulatePosition(Node):
    def __init__(self):
        super().__init__('simulate_position')
        
        # 创建发布者
        self.position_pub = self.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position', 10)
        self.status_pub = self.create_publisher(VehicleStatus, '/fmu/out/vehicle_status', 10)
        
        # 定时器
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 初始化位置
        self.current_x = 0.0
        self.current_y = 0.0
    
    def timer_callback(self):
        # 模拟当前位置
        pos_msg = VehicleLocalPosition()
        pos_msg.x = self.current_x
        pos_msg.y = self.current_y
        pos_msg.z = -5.0  # 当前高度
        pos_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.position_pub.publish(pos_msg)
        
        # 模拟状态（处于offboard模式）
        status_msg = VehicleStatus()
        status_msg.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        status_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.status_pub.publish(status_msg)
        
        self.get_logger().info(f'发布模拟位置: ({pos_msg.x}, {pos_msg.y}, {pos_msg.z})')

def main(args=None):
    rclpy.init(args=args)
    simulate_position = SimulatePosition()
    rclpy.spin(simulate_position)
    simulate_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# 给脚本添加执行权限
chmod +x /tmp/simulate_position.py

# 运行模拟位置发布器（在终端3中）
ros2 run rclpy_components component_container --ros-args -p use_sim_time:=True
```

### 4.2 使用rqt_graph查看节点关系

执行以下命令查看节点和话题的关系图：

```bash
rqt_graph
```

**预期输出**：应该显示清晰的节点关系图：
- `/red_color_detector_ros2` 发布到 `/detection/red_center`
- `/color_tracking_offboard` 订阅 `/detection/red_center`
- `/color_tracking_offboard` 发布到各种 `/fmu/in/` 话题

## 5. 验证控制逻辑

### 5.1 检查控制增益和最大偏移

控制节点使用以下参数来计算位置偏移：
- `control_gain` = 0.01（像素误差转位置偏移的增益）
- `max_offset` = 1.0（最大允许偏移，单位：米）
- `target_altitude` = -5.0（目标高度，单位：米）

可以通过修改这些参数来调整控制响应：

```bash
# 启动控制节点并修改参数
ros2 run cv_ros color_tracking_offboard --ros-args -p control_gain:=0.02 -p max_offset:=0.5
```

### 5.2 验证位置计算

当红色物体位于摄像头画面不同位置时，控制节点会计算不同的目标位置：

| 红色物体位置 | 像素误差 | 计算的位置偏移 |
|------------|---------|--------------|
| 左上角      | x=-320, y=-240 | x=3.2m, y=2.4m（受max_offset限制为1.0m） |
| 左下角      | x=-320, y=240  | x=3.2m, y=-2.4m（受max_offset限制为1.0m） |
| 右上角      | x=320, y=-240  | x=-3.2m, y=2.4m（受max_offset限制为-1.0m） |
| 右下角      | x=320, y=240   | x=-3.2m, y=-2.4m（受max_offset限制为-1.0m） |
| 中心        | x=0, y=0       | x=0.0m, y=0.0m |

## 6. 故障排查

### 6.1 节点启动失败

- 检查是否已安装所有依赖：`pip install rclpy numpy`
- 检查是否已正确构建工作空间：`colcon build --symlink-install`
- 检查PX4飞控是否已连接（如果使用真实无人机）

### 6.2 没有接收到颜色中心数据

- 检查红色检测节点是否正常运行
- 检查摄像头是否能看到红色物体
- 检查话题名称是否正确：`/detection/red_center`

### 6.3 控制指令没有发送

- 检查是否满足offboard模式条件（需要连续发送10个控制指令）
- 检查车辆状态是否为offboard模式
- 检查飞控连接是否正常（如果使用真实无人机）

### 6.4 跟踪效果不佳

- 调整 `control_gain` 参数（增大提高响应速度，减小提高稳定性）
- 调整 `max_offset` 参数（根据实际需求调整最大移动范围）
- 检查红色检测的阈值是否适合当前环境

## 7. 高级调试

### 7.1 使用rqt_console查看详细日志

```bash
rqt_console
```

在rqt_console中，可以设置日志级别并过滤特定节点的日志信息。

### 7.2 使用ros2 topic hz检查消息频率

```bash
# 检查检测话题的发布频率
ros2 topic hz /detection/red_center

# 检查控制指令的发布频率
ros2 topic hz /fmu/in/trajectory_setpoint
```

### 7.3 录制和回放数据

可以使用rosbag录制数据以便后续分析：

```bash
# 录制相关话题
ros2 bag record /detection/red_center /fmu/in/trajectory_setpoint /fmu/out/vehicle_local_position

# 回放录制的数据
ros2 bag play rosbag2_xxx
```

通过以上步骤，您可以全面验证颜色跟踪Offboard控制节点的功能是否正常工作。