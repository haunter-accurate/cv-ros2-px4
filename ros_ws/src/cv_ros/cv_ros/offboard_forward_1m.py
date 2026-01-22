#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleControlMode


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
        self.vehicle_control_mode_subscriber = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)

        # 参数配置
        self.declare_parameter('headless', False)  # 是否启用headless模式（无详细日志）
        self.declare_parameter('target_altitude', -5.0)  # 目标高度（米，负数因为PX4使用NED坐标系）
        self.declare_parameter('forward_distance', 1.0)  # 向前前进的距离（米）
        self.declare_parameter('offboard_maintain_time', 2.0)  # offboard模式需要维持的时间（秒）
        self.declare_parameter('position_tolerance', 0.1)  # 位置误差容忍度（米）
        self.declare_parameter('hover_time_after_completion', 3.0)  # 任务完成后悬停的时间（秒）
        self.declare_parameter('max_vertical_speed', 0.2)  # 最大垂直速度（米/秒）
        self.declare_parameter('max_horizontal_speed', 0.5)  # 最大水平速度（米/秒）
        
        # 获取参数
        self.headless = self.get_parameter('headless').value
        self.target_altitude = self.get_parameter('target_altitude').value
        self.forward_distance = self.get_parameter('forward_distance').value
        self.offboard_maintain_time = self.get_parameter('offboard_maintain_time').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.hover_time_after_completion = self.get_parameter('hover_time_after_completion').value
        self.max_vertical_speed = self.get_parameter('max_vertical_speed').value
        self.max_horizontal_speed = self.get_parameter('max_horizontal_speed').value
        
        # 初始化变量
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_control_mode = VehicleControlMode()
        self.has_sent_forward_command = False  # 标记是否已经计算了目标位置
        self.offboard_entry_time = None  # 记录进入offboard模式的时间
        self.offboard_mode_maintained = False  # 标记offboard模式是否已经维持了2秒
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.task_completed = False  # 标记任务是否完成
        self.task_start_time = None  # 记录任务开始时间
        self.task_completion_time = None  # 记录任务完成时间
        self.completion_time_start = None  # 记录开始悬停的时间
        self.last_position_time = None  # 记录上次位置时间
        self.last_position = None  # 记录上次位置
        
        # 输出初始化信息
        if not self.headless:
            self.get_logger().info(f"参数配置: headless={self.headless}, target_altitude={self.target_altitude}, forward_distance={self.forward_distance}")
            self.get_logger().info(f"速度限制: 垂直={self.max_vertical_speed}m/s, 水平={self.max_horizontal_speed}m/s")

        # 创建定时器来发布控制命令
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """vehicle_local_position话题订阅者的回调函数。"""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_control_mode_callback(self, vehicle_control_mode):
        """vehicle_control_mode话题订阅者的回调函数。"""
        self.vehicle_control_mode = vehicle_control_mode

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
        """发布轨迹设定点，带速度限制。"""
        # 计算速度限制后的目标位置
        limited_x, limited_y, limited_z = self.apply_speed_limits(x, y, z)
        
        msg = TrajectorySetpoint()
        msg.position = [limited_x, limited_y, limited_z]
        msg.yaw = 1.57079  # (90度) - 偏航角
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        
        # 只在首次发布时记录日志，避免过于频繁的日志输出
        if not hasattr(self, 'position_logged') and not self.headless:
            self.get_logger().info(f"正在发布位置设定点 {[limited_x, limited_y, limited_z]}")
            self.position_logged = True
    
    def apply_speed_limits(self, target_x, target_y, target_z):
        """应用速度限制，确保速度不超过设定值。"""
        current_time = self.get_clock().now()
        
        # 检查是否有上次位置和时间
        if not hasattr(self, 'last_position') or self.last_position is None or not hasattr(self, 'last_position_time') or self.last_position_time is None:
            # 第一次调用，直接返回目标位置
            self.last_position = (target_x, target_y, target_z)
            self.last_position_time = current_time
            return target_x, target_y, target_z
        
        # 计算时间差（秒）
        time_diff = (current_time - self.last_position_time).nanoseconds / 1e9
        if time_diff <= 0:
            return target_x, target_y, target_z
        
        # 获取当前位置
        current_x = self.vehicle_local_position.x if hasattr(self.vehicle_local_position, 'x') else self.last_position[0]
        current_y = self.vehicle_local_position.y if hasattr(self.vehicle_local_position, 'y') else self.last_position[1]
        current_z = self.vehicle_local_position.z if hasattr(self.vehicle_local_position, 'z') else self.last_position[2]
        
        # 计算目标位置与当前位置的差值
        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z
        
        # 计算水平距离和垂直距离
        horizontal_distance = (dx**2 + dy**2)**0.5
        vertical_distance = abs(dz)
        
        # 计算最大允许的水平和垂直移动距离
        max_horizontal_move = self.max_horizontal_speed * time_diff
        max_vertical_move = self.max_vertical_speed * time_diff
        
        # 应用水平速度限制
        if horizontal_distance > max_horizontal_move:
            scale_factor = max_horizontal_move / horizontal_distance
            limited_dx = dx * scale_factor
            limited_dy = dy * scale_factor
        else:
            limited_dx = dx
            limited_dy = dy
        
        # 应用垂直速度限制
        if vertical_distance > max_vertical_move:
            limited_dz = dz * (max_vertical_move / vertical_distance)
        else:
            limited_dz = dz
        
        # 计算限制后的目标位置
        limited_x = current_x + limited_dx
        limited_y = current_y + limited_dy
        limited_z = current_z + limited_dz
        
        # 更新上次位置和时间
        self.last_position = (limited_x, limited_y, limited_z)
        self.last_position_time = current_time
        
        return limited_x, limited_y, limited_z

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

        # 记录当前控制模式和高度（仅在非headless模式下）
        if self.offboard_setpoint_counter % 10 == 0 and not self.headless:
            is_offboard = self.vehicle_control_mode.flag_control_offboard_enabled if hasattr(self.vehicle_control_mode, 'flag_control_offboard_enabled') else False
            altitude = self.vehicle_local_position.z if hasattr(self.vehicle_local_position, 'z') else "未知"
            self.get_logger().info(f"OFFBOARD模式: {is_offboard}, 高度: {altitude}")

        # 检查是否处于OFFBOARD模式
        is_offboard = hasattr(self.vehicle_control_mode, 'flag_control_offboard_enabled') and \
                     self.vehicle_control_mode.flag_control_offboard_enabled
        
        # 检查是否已经起飞（高度低于-0.5米，因为PX4使用NED坐标系）
        is_flying = self.vehicle_local_position.z < -0.5
        
        # 处理offboard模式计时逻辑
        if is_offboard:
            # 如果刚刚进入offboard模式，记录当前时间
            if self.offboard_entry_time is None:
                self.offboard_entry_time = self.get_clock().now()
                if not self.headless:
                    self.get_logger().info("已进入offboard模式，开始计时")
            else:
                # 计算已经在offboard模式的时间
                current_time = self.get_clock().now()
                elapsed_time = (current_time - self.offboard_entry_time).nanoseconds / 1e9  # 转换为秒
                
                # 检查是否已经维持了足够的时间
                if elapsed_time >= self.offboard_maintain_time and not self.offboard_mode_maintained:
                    self.offboard_mode_maintained = True
                    if not self.headless:
                        self.get_logger().info(f"offboard模式已维持 {self.offboard_maintain_time} 秒，准备执行前进命令")
        else:
            # 退出offboard模式，重置计时
            if self.offboard_entry_time is not None:
                self.offboard_entry_time = None
                self.offboard_mode_maintained = False
                if not self.headless:
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
                self.task_start_time = self.get_clock().now()  # 记录任务开始时间
                if not self.headless:
                    self.get_logger().info(f"已设置向前1米的目标位置: X={self.target_x}, Y={self.target_y}, Z={self.target_z}")
            
            # 检查是否到达目标位置
            if not self.task_completed and hasattr(self.vehicle_local_position, 'x'):
                # 计算当前位置与目标位置的距离
                distance_to_target = ((self.vehicle_local_position.x - self.target_x) ** 2 +
                                     (self.vehicle_local_position.y - self.target_y) ** 2 +
                                     (self.vehicle_local_position.z - self.target_z) ** 2) ** 0.5
                
                # 检查是否在位置误差容忍度内
                if distance_to_target < self.position_tolerance:
                    self.task_completed = True
                    self.task_completion_time = self.get_clock().now()
                    self.completion_time_start = self.get_clock().now()
                    
                    # 计算任务完成时间
                    if self.task_start_time and not self.headless:
                        task_duration = (self.task_completion_time - self.task_start_time).nanoseconds / 1e9
                        self.get_logger().info(f"✅ 任务完成！已成功前进1米到达目标位置")
                        self.get_logger().info(f"   目标位置: X={self.target_x:.2f}, Y={self.target_y:.2f}, Z={self.target_z:.2f}")
                        self.get_logger().info(f"   当前位置: X={self.vehicle_local_position.x:.2f}, Y={self.vehicle_local_position.y:.2f}, Z={self.vehicle_local_position.z:.2f}")
                        self.get_logger().info(f"   完成时间: {task_duration:.2f}秒")
                        self.get_logger().info(f"   将在目标位置悬停 {self.hover_time_after_completion} 秒")
                    elif not self.headless:
                        self.get_logger().info(f"✅ 任务完成！已成功前进1米到达目标位置")
                        self.get_logger().info(f"   目标位置: X={self.target_x:.2f}, Y={self.target_y:.2f}, Z={self.target_z:.2f}")
                        self.get_logger().info(f"   当前位置: X={self.vehicle_local_position.x:.2f}, Y={self.vehicle_local_position.y:.2f}, Z={self.vehicle_local_position.z:.2f}")
                        self.get_logger().info(f"   将在目标位置悬停 {self.hover_time_after_completion} 秒")
            
            # 检查悬停时间
            if self.task_completed and self.completion_time_start:
                hover_elapsed = (self.get_clock().now() - self.completion_time_start).nanoseconds / 1e9
                if hover_elapsed < self.hover_time_after_completion:
                    # 继续悬停
                    remaining_hover_time = self.hover_time_after_completion - hover_elapsed
                    if int(remaining_hover_time * 10) % 10 == 0 and not self.headless:  # 每0.1秒更新一次，仅在非headless模式下
                        self.get_logger().info(f"任务已完成，正在悬停... 剩余时间: {remaining_hover_time:.1f}秒")
                else:
                    # 悬停时间结束
                    if not hasattr(self, 'hover_completed'):
                        self.hover_completed = True
                        if not self.headless:
                            self.get_logger().info(f"✅ 悬停时间已结束，任务流程全部完成！")
                            if self.task_start_time:
                                self.get_logger().info(f"   总任务时间: {(self.get_clock().now() - self.task_start_time).nanoseconds / 1e9:.2f}秒")
        
            # 持续发布目标位置，确保无人机保持在目标位置
            self.publish_position_setpoint(self.target_x, self.target_y, self.target_z)
        elif is_offboard and not self.headless:
            self.get_logger().info("处于OFFBOARD模式，但飞机尚未起飞")
        elif not self.headless:
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
