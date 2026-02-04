#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleControlMode, VehicleGlobalPosition
import socket
import json


class GpsOffboardPublisher(Node):
    """在进入offboard模式2秒后通过socket发布当前GPS坐标的节点。"""

    def __init__(self) -> None:
        super().__init__('gps_offboard_publisher')

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
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile)

        # 参数配置
        self.declare_parameter('headless', False)  # 是否启用headless模式
        self.declare_parameter('offboard_wait_time', 2.0)  # 进入offboard模式后等待时间（秒）
        self.declare_parameter('socket_host', '10.5.9.8')  # Socket服务器主机
        self.declare_parameter('socket_port', 5000)  # Socket服务器端口
        
        # 获取参数
        self.headless = self.get_parameter('headless').value
        self.offboard_wait_time = self.get_parameter('offboard_wait_time').value
        self.socket_host = self.get_parameter('socket_host').value
        self.socket_port = self.get_parameter('socket_port').value

        # 初始化变量
        self.vehicle_control_mode = VehicleControlMode()
        self.vehicle_global_position = VehicleGlobalPosition()
        self.offboard_entry_time = None  # 记录进入offboard模式的时间
        self.gps_published = False  # 标记是否已经发布了GPS坐标
        self.offboard_mode_detected = False  # 标记是否检测到offboard模式
        
        # 输出初始化信息
        if not self.headless:
            self.get_logger().info("GPS Offboard Publisher 启动")
            self.get_logger().info(f"参数配置: headless={self.headless}, offboard_wait_time={self.offboard_wait_time}秒")
            self.get_logger().info(f"Socket配置: {self.socket_host}:{self.socket_port}")
            self.get_logger().info("正在等待进入offboard模式...")

        # 创建定时器
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_control_mode_callback(self, vehicle_control_mode):
        """vehicle_control_mode话题订阅者的回调函数。"""
        self.vehicle_control_mode = vehicle_control_mode

    def vehicle_global_position_callback(self, vehicle_global_position):
        """vehicle_global_position话题订阅者的回调函数。"""
        self.vehicle_global_position = vehicle_global_position

    def publish_gps_coordinates(self):
        """通过socket发布GPS坐标。"""
        if not hasattr(self.vehicle_global_position, 'lat') or not hasattr(self.vehicle_global_position, 'lon') or not hasattr(self.vehicle_global_position, 'alt'):
            if not self.headless:
                self.get_logger().warn("GPS数据无效，无法发布")
            return
        
        # 准备GPS数据
        gps_data = {
            'latitude': self.vehicle_global_position.lat,
            'longitude': self.vehicle_global_position.lon,
            'altitude': self.vehicle_global_position.alt,
            'timestamp': self.vehicle_global_position.timestamp
        }
        
        try:
            # 创建socket连接
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.socket_host, self.socket_port))
                # 发送GPS数据
                data = json.dumps(gps_data).encode('utf-8')
                s.sendall(data)
                if not self.headless:
                    self.get_logger().info(f"✅ GPS坐标已发布: {gps_data}")
                self.gps_published = True
        except Exception as e:
            if not self.headless:
                self.get_logger().error(f"发布GPS坐标失败: {str(e)}")

    def timer_callback(self) -> None:
        """定时器的回调函数。"""
        # 检查是否已经发布了GPS坐标
        if self.gps_published:
            return
        
        # 检查是否处于OFFBOARD模式
        is_offboard = hasattr(self.vehicle_control_mode, 'flag_control_offboard_enabled') and \
                     self.vehicle_control_mode.flag_control_offboard_enabled
        
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
                
                # 检查是否已经等待了足够的时间
                if elapsed_time >= self.offboard_wait_time:
                    # 发布GPS坐标
                    self.publish_gps_coordinates()
        else:
            # 退出offboard模式，重置状态
            if self.offboard_mode_detected:
                self.offboard_entry_time = None
                self.offboard_mode_detected = False
                if not self.headless:
                    self.get_logger().info("已退出offboard模式，重置状态")

    def destroy_node(self):
        """节点销毁时的清理工作。"""
        super().destroy_node()


def main(args=None) -> None:
    print('启动GPS Offboard Publisher节点...')
    rclpy.init(args=args)
    gps_offboard_publisher = GpsOffboardPublisher()
    rclpy.spin(gps_offboard_publisher)
    gps_offboard_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)