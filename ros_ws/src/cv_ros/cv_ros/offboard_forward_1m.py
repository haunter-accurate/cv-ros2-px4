#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus


class OffboardForward1m(Node):
    """在offboard模式下控制无人机向前前进1米的节点。"""

    def __init__(self) -> None:
        super().__init__('offboard_forward_1m')

        # 配置发布和订阅的QoS配置文件
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 创建发布者
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # 创建订阅者
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # 初始化变量
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.has_sent_forward_command = False  # 标记是否已经计算了目标位置
        self.offboard_entry_time = None  # 记录进入offboard模式的时间
        self.offboard_mode_maintained = False  # 标记offboard模式是否已经维持了2秒
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        
        # 控制参数
        self.target_altitude = -5.0  # 目标高度（米，负数因为PX4使用NED坐标系）
        self.forward_distance = 1.0  # 向前前进的距离（米）
        self.offboard_maintain_time = 2.0  # offboard模式需要维持的时间（秒）

        # 创建定时器来发布控制命令
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """vehicle_local_position话题订阅者的回调函数。"""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """vehicle_status话题订阅者的回调函数。"""
        self.vehicle_status = vehicle_status

    def arm(self):
        """向无人机发送解锁命令。"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('解锁命令已发送')

    def disarm(self):
        """向无人机发送锁定命令。"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('锁定命令已发送')

    def engage_offboard_mode(self):
        """切换到offboard模式。"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("正在切换到offboard模式")

    def land(self):
        """切换到降落模式。"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("正在切换到降落模式")

    def publish_offboard_control_heartbeat_signal(self):
        """发布offboard控制模式。"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """发布轨迹设定点。"""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90度) - 偏航角
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # 只在首次发布时记录日志，避免过于频繁的日志输出
        if not hasattr(self, 'position_logged'):
            self.get_logger().info(f"正在发布位置设定点 {[x, y, z]}")
            self.position_logged = True

    def publish_vehicle_command(self, command, **params) -> None:
        """发布无人机命令。"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """定时器的回调函数。"""
        # 发布offboard控制模式心跳信号（无论是否在OFFBOARD模式）
        self.publish_offboard_control_heartbeat_signal()

        # 记录当前导航状态和高度
        if self.offboard_setpoint_counter % 10 == 0:
            nav_state = self.vehicle_status.nav_state if hasattr(self.vehicle_status, 'nav_state') else "未知"
            altitude = self.vehicle_local_position.z if hasattr(self.vehicle_local_position, 'z') else "未知"
            self.get_logger().info(f"导航状态: {nav_state}, 高度: {altitude}")

        # 检查是否处于OFFBOARD模式
        is_offboard = hasattr(self.vehicle_status, 'nav_state') and \
                     self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        
        # 检查是否已经起飞（高度低于-0.5米，因为PX4使用NED坐标系）
        is_flying = self.vehicle_local_position.z < -0.5
        
        # 处理offboard模式计时逻辑
        if is_offboard:
            # 如果刚刚进入offboard模式，记录当前时间
            if self.offboard_entry_time is None:
                self.offboard_entry_time = self.get_clock().now()
                self.get_logger().info("已进入offboard模式，开始计时")
            else:
                # 计算已经在offboard模式的时间
                current_time = self.get_clock().now()
                elapsed_time = (current_time - self.offboard_entry_time).nanoseconds / 1e9  # 转换为秒
                
                # 检查是否已经维持了足够的时间
                if elapsed_time >= self.offboard_maintain_time and not self.offboard_mode_maintained:
                    self.offboard_mode_maintained = True
                    self.get_logger().info(f"offboard模式已维持 {self.offboard_maintain_time} 秒，准备执行前进命令")
        else:
            # 退出offboard模式，重置计时
            if self.offboard_entry_time is not None:
                self.offboard_entry_time = None
                self.offboard_mode_maintained = False
                self.get_logger().info("已退出offboard模式，重置计时")
        
        # 只有在OFFBOARD模式维持2秒且已经起飞的情况下才发送目标位置
        if self.offboard_mode_maintained and is_flying:
            # 只计算一次向前1米的目标位置
            if not self.has_sent_forward_command:
                # 计算目标位置（当前位置向前1米，在NED坐标系中X轴是向前的）
                self.target_x = self.vehicle_local_position.x + self.forward_distance
                self.target_y = self.vehicle_local_position.y
                self.target_z = self.target_altitude
                
                self.has_sent_forward_command = True  # 标记已经计算了目标位置
                self.get_logger().info(f"已设置向前1米的目标位置: X={self.target_x}, Y={self.target_y}, Z={self.target_z}")
            
            # 持续发布目标位置，确保无人机保持在目标位置
            self.publish_position_setpoint(self.target_x, self.target_y, self.target_z)
        elif is_offboard:
            self.get_logger().info("处于OFFBOARD模式，但飞机尚未起飞")
        else:
            self.get_logger().info("未处于OFFBOARD模式，等待遥控器切换")

        # 更新计数器
        if self.offboard_setpoint_counter < 100:
            self.offboard_setpoint_counter += 1

    def destroy_node(self):
        """节点销毁时的清理工作。"""
        super().destroy_node()


def main(args=None) -> None:
    print('启动offboard向前1米控制节点...')
    rclpy.init(args=args)
    offboard_forward_1m = OffboardForward1m()
    rclpy.spin(offboard_forward_1m)
    offboard_forward_1m.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
