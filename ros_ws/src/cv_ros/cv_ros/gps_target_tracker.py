#!/usr/bin/env python3
"""
GPS目标位置跟踪器

此程序用于在另一架无人机上运行，通过socket接收GPS坐标，并在进入offboard模式后
计算并发布目标位置（接收到的GPS坐标南方2米，高度+2米）。

使用方法:
ros2 run cv_ros gps_target_tracker
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleControlMode, TrajectorySetpoint, VehicleGlobalPosition, OffboardControlMode, VehicleCommand
import socket
import json
import threading
import math


class GpsTargetTracker(Node):
    """GPS目标位置跟踪器节点。"""

    def __init__(self) -> None:
        super().__init__('gps_target_tracker')

        # 配置QoS配置文件
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 创建订阅者
        self.vehicle_control_mode_subscriber = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)
        
        # 订阅本机GPS位置
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile)

        # 创建发布者
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # 参数配置
        self.declare_parameter('headless', False)  # 是否启用headless模式
        self.declare_parameter('enable_control', True)  # 是否启用控制（默认启用）
        self.declare_parameter('offboard_wait_time', 2.0)  # 进入offboard模式后等待时间（秒）
        self.declare_parameter('socket_host', '10.5.211.119')  # Socket监听主机
        self.declare_parameter('socket_port', 5000)  # Socket监听端口
        self.declare_parameter('south_distance', 2.0)  # 向南移动距离（米）
        self.declare_parameter('altitude_offset', 2.0)  # 高度偏移（米）
        self.declare_parameter('max_horizontal_velocity', 1.0)  # 最大水平速度（米/秒）
        self.declare_parameter('max_vertical_velocity', 0.5)  # 最大垂直速度（米/秒）
        
        # 获取参数
        self.headless = self.get_parameter('headless').value
        self.enable_control = self.get_parameter('enable_control').value
        self.offboard_wait_time = self.get_parameter('offboard_wait_time').value
        self.socket_host = self.get_parameter('socket_host').value
        self.socket_port = self.get_parameter('socket_port').value
        self.south_distance = self.get_parameter('south_distance').value
        self.altitude_offset = self.get_parameter('altitude_offset').value
        self.max_horizontal_velocity = self.get_parameter('max_horizontal_velocity').value
        self.max_vertical_velocity = self.get_parameter('max_vertical_velocity').value

        # 初始化变量
        self.offboard_setpoint_counter = 0
        self.vehicle_control_mode = VehicleControlMode()
        self.vehicle_global_position = VehicleGlobalPosition()  # 本机GPS位置
        self.offboard_entry_time = None  # 记录进入offboard模式的时间
        self.offboard_mode_detected = False  # 标记是否检测到offboard模式
        self.target_position_active = False  # 标记目标位置是否激活
        self.vehicle_global_position_received = False  # 标记是否接收到本机GPS位置
        
        # GPS坐标相关变量
        self.received_gps_position = None  # 接收到的GPS坐标
        self.target_gps_position = None  # 计算出的目标GPS坐标
        self.target_ned_position = None  # 转换后的NED目标位置
        
        # 地球半径（米）
        self.earth_radius = 6378137.0
        
        # 输出初始化信息
        if not self.headless:
            self.get_logger().info("GPS目标位置跟踪器 启动")
            self.get_logger().info(f"参数配置: headless={self.headless}, enable_control={self.enable_control}, offboard_wait_time={self.offboard_wait_time}秒")
            self.get_logger().info(f"Socket配置: {self.socket_host}:{self.socket_port}")
            self.get_logger().info(f"目标位置偏移: 南方{self.south_distance}米, 高度+{self.altitude_offset}米")
            self.get_logger().info(f"速度限制: 水平最大{self.max_horizontal_velocity}米/秒, 垂直最大{self.max_vertical_velocity}米/秒")
            if not self.enable_control:
                self.get_logger().warn("控制已禁用，仅输出期望移动距离，不发布控制指令")
            self.get_logger().info("正在等待GPS坐标数据...")

        # 创建定时器
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 启动socket接收线程
        self.socket_thread = threading.Thread(target=self.receive_gps_coordinates, daemon=True)
        self.socket_thread.start()

    def vehicle_control_mode_callback(self, vehicle_control_mode):
        """vehicle_control_mode话题订阅者的回调函数。"""
        self.vehicle_control_mode = vehicle_control_mode

    def vehicle_global_position_callback(self, vehicle_global_position):
        """vehicle_global_position话题订阅者的回调函数。"""
        self.vehicle_global_position = vehicle_global_position
        if not self.vehicle_global_position_received:
            self.vehicle_global_position_received = True
            if not self.headless:
                self.get_logger().info("✓ 已接收到本机GPS位置数据")

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

    def receive_gps_coordinates(self):
        """通过socket接收GPS坐标的线程函数。"""
        try:
            # 创建socket服务器
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind((self.socket_host, self.socket_port))
                s.listen(5)
                s.settimeout(1.0)
                
                if not self.headless:
                    self.get_logger().info(f"Socket服务器已启动，监听 {self.socket_host}:{self.socket_port}")
                
                while rclpy.ok():
                    try:
                        # 等待连接
                        conn, addr = s.accept()
                        with conn:
                            if not self.headless:
                                self.get_logger().info(f"接收到来自 {addr} 的GPS坐标连接")
                            
                            # 接收数据
                            data = conn.recv(1024)
                            if data:
                                try:
                                    # 解析GPS数据
                                    gps_data = json.loads(data.decode('utf-8'))
                                    
                                    # 提取GPS坐标
                                    latitude = gps_data.get('latitude')
                                    longitude = gps_data.get('longitude')
                                    altitude = gps_data.get('altitude')
                                    
                                    if latitude and longitude and altitude:
                                        # 更新接收到的GPS坐标
                                        self.received_gps_position = {
                                            'latitude': latitude,
                                            'longitude': longitude,
                                            'altitude': altitude
                                        }
                                        
                                        if not self.headless:
                                            self.get_logger().info(f"✓ 接收到GPS坐标: lat={latitude}, lon={longitude}, alt={altitude}")
                                        
                                        # 计算目标位置
                                        self.calculate_target_position()
                                        
                                except json.JSONDecodeError as e:
                                    if not self.headless:
                                        self.get_logger().error(f"解析GPS数据失败: {e}")
                            
                    except socket.timeout:
                        # 超时是正常的，继续等待
                        continue
                    except Exception as e:
                        if not self.headless:
                            self.get_logger().error(f"Socket接收错误: {e}")
                        continue
                        
        except Exception as e:
            if not self.headless:
                self.get_logger().error(f"Socket服务器启动失败: {e}")

    def calculate_target_position(self):
        """计算目标GPS位置（南方2米，高度+2米）。"""
        if not self.received_gps_position:
            return
        
        # 获取当前GPS坐标
        current_lat = self.received_gps_position['latitude']
        current_lon = self.received_gps_position['longitude']
        current_alt = self.received_gps_position['altitude']
        
        # 计算目标GPS坐标
        # 向南移动2米（纬度减小）
        delta_lat = -self.south_distance / self.earth_radius  # 负号表示向南
        target_lat = current_lat + delta_lat
        target_lon = current_lon  # 经度不变
        target_alt = current_alt + self.altitude_offset  # 高度增加
        
        # 更新目标GPS位置
        self.target_gps_position = {
            'latitude': target_lat,
            'longitude': target_lon,
            'altitude': target_alt
        }
        
        if not self.headless:
            self.get_logger().info(f"计算目标位置: lat={target_lat}, lon={target_lon}, alt={target_alt}")

    def gps_to_ned(self, lat, lon, alt, ref_lat, ref_lon, ref_alt):
        """将GPS坐标转换为NED坐标。"""
        # 转换为弧度
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(ref_lat)
        ref_lon_rad = math.radians(ref_lon)
        
        # 计算差值
        delta_lat = lat_rad - ref_lat_rad
        delta_lon = lon_rad - ref_lon_rad
        
        # 计算NED坐标
        x = delta_lat * self.earth_radius  # 北方向
        y = delta_lon * self.earth_radius * math.cos(ref_lat_rad)  # 东方向
        z = ref_alt - alt  # 天方向（负号表示向下）
        
        return x, y, z

    def publish_target_position(self):
        """发布目标位置。"""
        if not self.target_gps_position:
            return
        
        # 获取本机GPS位置作为参考
        ref_lat = self.vehicle_global_position.lat
        ref_lon = self.vehicle_global_position.lon
        ref_alt = self.vehicle_global_position.alt
        
        # 获取目标GPS位置
        target_lat = self.target_gps_position['latitude']
        target_lon = self.target_gps_position['longitude']
        target_alt = self.target_gps_position['altitude']
        
        # 转换目标GPS坐标为NED坐标
        ned_x, ned_y, ned_z = self.gps_to_ned(target_lat, target_lon, target_alt, ref_lat, ref_lon, ref_alt)
        
        # 计算水平距离
        horizontal_distance = math.sqrt(ned_x**2 + ned_y**2)
        # 计算垂直距离
        vertical_distance = abs(ned_z)
        
        # 如果控制未启用，只输出期望移动距离
        if not self.enable_control:
            if not self.headless:
                self.get_logger().info(f"期望移动距离: X={ned_x:.2f}m, Y={ned_y:.2f}m, Z={ned_z:.2f}m (测试模式，未发布控制指令)")
                self.get_logger().info(f"目标GPS位置: lat={target_lat}, lon={target_lon}, alt={target_alt}")
            return
        
        # 创建TrajectorySetpoint消息
        trajectory_msg = TrajectorySetpoint()
        
        # 设置NED目标位置
        trajectory_msg.x = ned_x  # 北方向
        trajectory_msg.y = ned_y  # 东方向
        trajectory_msg.z = ned_z  # 天方向（负号表示向下）
        
        # 计算速度向量（基于目标位置和当前位置的差值）
        
        # 计算速度向量
        if horizontal_distance > 0.1:  # 避免除以零
            # 计算水平速度分量
            vx = ned_x / horizontal_distance * min(self.max_horizontal_velocity, horizontal_distance * 0.5)
            vy = ned_y / horizontal_distance * min(self.max_horizontal_velocity, horizontal_distance * 0.5)
        else:
            vx = 0.0
            vy = 0.0
        
        if vertical_distance > 0.1:  # 避免除以零
            # 计算垂直速度分量
            vz = ned_z / vertical_distance * min(self.max_vertical_velocity, vertical_distance * 0.5)
        else:
            vz = 0.0
        
        # 设置速度
        trajectory_msg.vx = vx  # 北方向速度
        trajectory_msg.vy = vy  # 东方向速度
        trajectory_msg.vz = vz  # 天方向速度（负号表示向下）
        
        # 设置加速度为0
        trajectory_msg.acceleration[0] = 0.0
        trajectory_msg.acceleration[1] = 0.0
        trajectory_msg.acceleration[2] = 0.0
        
        # 设置偏航角为0
        trajectory_msg.yaw = 0.0
        trajectory_msg.yawspeed = 0.0
        
        # 发布消息
        self.trajectory_setpoint_publisher.publish(trajectory_msg)
        
        if not self.headless:
            self.get_logger().info(f"发布目标位置: x={ned_x:.2f}, y={ned_y:.2f}, z={ned_z:.2f}")
            self.get_logger().info(f"目标GPS位置: lat={target_lat}, lon={target_lon}, alt={target_alt}")
            self.get_logger().info(f"速度设置: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")

    def timer_callback(self) -> None:
        """定时器的回调函数。"""
        # 发布offboard控制模式心跳信号（仅在启用控制时）
        if self.enable_control:
            self.publish_offboard_control_heartbeat_signal()

        # 记录当前控制模式和高度（仅在非headless模式下）
        if self.offboard_setpoint_counter % 10 == 0 and not self.headless:
            is_offboard = self.vehicle_control_mode.flag_control_offboard_enabled if hasattr(self.vehicle_control_mode, 'flag_control_offboard_enabled') else False
            altitude = self.vehicle_global_position.alt if hasattr(self.vehicle_global_position, 'alt') else "未知"
            control_status = "启用" if self.enable_control else "禁用"
            self.get_logger().info(f"OFFBOARD模式: {is_offboard}, 高度: {altitude}, 控制状态: {control_status}")

        # 如果控制未启用，只输出期望移动距离，不进行offboard控制逻辑
        if not self.enable_control:
            # 检查是否接收到GPS坐标
            if not self.received_gps_position:
                if not self.headless and rclpy.ok():
                    # 每5秒输出一次，避免日志过多
                    if not hasattr(self, 'last_no_gps_log') or (self.get_clock().now() - self.last_no_gps_log).nanoseconds / 1e9 > 5:
                        self.get_logger().info("等待GPS坐标数据...")
                        self.last_no_gps_log = self.get_clock().now()
                return
            
            # 检查是否接收到本机GPS位置
            if not self.vehicle_global_position_received:
                if not self.headless and rclpy.ok():
                    # 每5秒输出一次，避免日志过多
                    if not hasattr(self, 'last_no_local_gps_log') or (self.get_clock().now() - self.last_no_local_gps_log).nanoseconds / 1e9 > 5:
                        self.get_logger().info("等待本机GPS位置数据...")
                        self.last_no_local_gps_log = self.get_clock().now()
                return
            
            # 发布期望移动距离（不发布控制指令）
            self.publish_target_position()
            return

        # 检查是否接收到GPS坐标
        if not self.received_gps_position:
            if not self.headless and rclpy.ok():
                # 每5秒输出一次，避免日志过多
                if not hasattr(self, 'last_no_gps_log') or (self.get_clock().now() - self.last_no_gps_log).nanoseconds / 1e9 > 5:
                    self.get_logger().info("等待GPS坐标数据...")
                    self.last_no_gps_log = self.get_clock().now()
            return
        
        # 检查是否接收到本机GPS位置
        if not self.vehicle_global_position_received:
            if not self.headless and rclpy.ok():
                # 每5秒输出一次，避免日志过多
                if not hasattr(self, 'last_no_local_gps_log') or (self.get_clock().now() - self.last_no_local_gps_log).nanoseconds / 1e9 > 5:
                    self.get_logger().info("等待本机GPS位置数据...")
                    self.last_no_local_gps_log = self.get_clock().now()
            return
        
        # 检查是否处于OFFBOARD模式
        is_offboard = hasattr(self.vehicle_control_mode, 'flag_control_offboard_enabled') and \
                     self.vehicle_control_mode.flag_control_offboard_enabled
        
        if not self.headless and rclpy.ok():
            # 每5秒输出一次offboard状态，避免日志过多
            if not hasattr(self, 'last_offboard_status_log') or (self.get_clock().now() - self.last_offboard_status_log).nanoseconds / 1e9 > 5:
                self.get_logger().info(f"当前状态: offboard模式={'是' if is_offboard else '否'}, 目标位置激活={'是' if self.target_position_active else '否'}")
                self.last_offboard_status_log = self.get_clock().now()
        
        if is_offboard:
            if not self.offboard_mode_detected:
                # 第一次检测到offboard模式，记录时间
                self.offboard_entry_time = self.get_clock().now()
                self.offboard_mode_detected = True
                if not self.headless:
                    self.get_logger().info("已进入offboard模式，开始计时")
            else:
                # 计算已经在offboard模式的时间
                current_time = self.get_clock().now()
                elapsed_time = (current_time - self.offboard_entry_time).nanoseconds / 1e9
                
                if not self.headless and rclpy.ok():
                    # 每2秒输出一次计时信息，避免日志过多
                    if not hasattr(self, 'last_timer_log') or (current_time - self.last_timer_log).nanoseconds / 1e9 > 2:
                        self.get_logger().info(f"在offboard模式的时间: {elapsed_time:.1f}秒, 等待时间: {self.offboard_wait_time}秒")
                        self.last_timer_log = current_time
                
                # 检查是否已经等待了足够的时间
                if elapsed_time >= self.offboard_wait_time:
                    # 激活目标位置
                    if not self.target_position_active:
                        self.target_position_active = True
                        if not self.headless:
                            self.get_logger().info("开始追踪目标位置")
                    
                    # 发布目标位置
                    if not self.headless:
                        self.get_logger().info("准备发布目标位置...")
                    self.publish_target_position()
        else:
            # 退出offboard模式，重置状态
            if self.offboard_mode_detected:
                self.offboard_entry_time = None
                self.offboard_mode_detected = False
                self.target_position_active = False
                if not self.headless:
                    self.get_logger().info("已退出offboard模式，重置状态")

        # 更新计数器
        self.offboard_setpoint_counter += 1

    def destroy_node(self):
        """节点销毁时的清理工作。"""
        super().destroy_node()


def main(args=None) -> None:
    print('启动GPS目标位置跟踪器节点...')
    rclpy.init(args=args)
    gps_target_tracker = GpsTargetTracker()
    rclpy.spin(gps_target_tracker)
    gps_target_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
