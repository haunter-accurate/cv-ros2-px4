#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleControlMode, VehicleGlobalPosition, VehicleLocalPosition
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
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

        # 参数配置
        self.declare_parameter('headless', False)  # 是否启用headless模式
        self.declare_parameter('offboard_wait_time', 2.0)  # 进入offboard模式后等待时间（秒）
        self.declare_parameter('socket_host', '192.168.43.175')  # Socket服务器主机
        self.declare_parameter('socket_port', 5000)  # Socket服务器端口
        self.declare_parameter('gps_filter_alpha', 0.3)  # GPS低通滤波系数（0-1，越小滤波越强）
        
        # 获取参数
        self.headless = self.get_parameter('headless').value
        self.offboard_wait_time = self.get_parameter('offboard_wait_time').value
        self.socket_host = self.get_parameter('socket_host').value
        self.socket_port = self.get_parameter('socket_port').value
        self.gps_filter_alpha = self.get_parameter('gps_filter_alpha').value

        # 初始化变量
        self.vehicle_control_mode = VehicleControlMode()
        self.vehicle_global_position = VehicleGlobalPosition()
        self.vehicle_local_position = VehicleLocalPosition()
        self.last_published_time = None  # 记录最后发布GPS坐标的时间
        self.publish_interval = 5.0  # GPS坐标发布间隔（秒）
        self.gps_position_received = False  # 标记是否接收到GPS位置数据
        self.local_position_received = False  # 标记是否接收到本地位置数据
        
        # GPS低通滤波变量
        self.filtered_lat = None  # 滤波后的纬度
        self.filtered_lon = None  # 滤波后的经度
        
        # 输出初始化信息
        if not self.headless:
            self.get_logger().info("GPS Offboard Publisher 启动")
            self.get_logger().info(f"参数配置: headless={self.headless}")
            self.get_logger().info(f"Socket配置: {self.socket_host}:{self.socket_port}")
            self.get_logger().info(f"GPS坐标发布间隔: {self.publish_interval}秒")
            self.get_logger().info("正在等待GPS位置数据...")

        # 创建定时器
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_control_mode_callback(self, vehicle_control_mode):
        """vehicle_control_mode话题订阅者的回调函数。"""
        self.vehicle_control_mode = vehicle_control_mode

    def vehicle_global_position_callback(self, vehicle_global_position):
        """vehicle_global_position话题订阅者的回调函数。"""
        self.vehicle_global_position = vehicle_global_position
        
        # 应用低通滤波
        if self.filtered_lat is None:
            # 第一次接收，直接使用当前值
            self.filtered_lat = vehicle_global_position.lat
            self.filtered_lon = vehicle_global_position.lon
        else:
            # 低通滤波：filtered = alpha * new + (1 - alpha) * filtered
            self.filtered_lat = self.gps_filter_alpha * vehicle_global_position.lat + (1 - self.gps_filter_alpha) * self.filtered_lat
            self.filtered_lon = self.gps_filter_alpha * vehicle_global_position.lon + (1 - self.gps_filter_alpha) * self.filtered_lon
        
        if not self.gps_position_received:
            self.gps_position_received = True
            if not self.headless:
                self.get_logger().info("✓ 已接收到GPS位置数据")

    def vehicle_local_position_callback(self, vehicle_local_position):
        """vehicle_local_position话题订阅者的回调函数。"""
        self.vehicle_local_position = vehicle_local_position
        if not self.local_position_received:
            self.local_position_received = True
            if not self.headless:
                self.get_logger().info("✓ 已接收到本地位置数据")

    def publish_gps_coordinates(self):
        """通过socket发布GPS坐标。"""
        if not self.headless:
            self.get_logger().info("开始发布GPS坐标...")
        
        # 检查GPS数据是否有效
        if not hasattr(self.vehicle_global_position, 'lat') or not hasattr(self.vehicle_global_position, 'lon'):
            if not self.headless:
                self.get_logger().warn("GPS数据无效，无法发布")
            return
        
        # 检查本地位置数据是否有效
        if not hasattr(self.vehicle_local_position, 'z'):
            if not self.headless:
                self.get_logger().warn("本地位置数据无效，无法获取高度")
            return
        
        # 检查GPS坐标有效性
        lat_lon_valid = getattr(self.vehicle_global_position, 'lat_lon_valid', False)
        z_valid = getattr(self.vehicle_local_position, 'z_valid', False)
        
        # 经纬度使用滤波后的GPS数据，高度使用本地位置（激光更可信）
        # 本地位置z在NED坐标系中负值表示向上，取绝对值作为高度
        altitude = abs(self.vehicle_local_position.z)
        
        # 使用滤波后的GPS坐标
        publish_lat = self.filtered_lat if self.filtered_lat is not None else self.vehicle_global_position.lat
        publish_lon = self.filtered_lon if self.filtered_lon is not None else self.vehicle_global_position.lon
        
        if not self.headless:
            self.get_logger().info(f"数据有效性: lat_lon_valid={lat_lon_valid}, z_valid={z_valid}")
            self.get_logger().info(f"原始GPS坐标: lat={self.vehicle_global_position.lat:.9f}, lon={self.vehicle_global_position.lon:.9f}")
            self.get_logger().info(f"滤波GPS坐标: lat={publish_lat:.9f}, lon={publish_lon:.9f}")
            self.get_logger().info(f"本地位置z: {self.vehicle_local_position.z:.3f}m, 发送高度: {altitude:.3f}m")
        
        # 准备GPS数据
        gps_data = {
            'latitude': publish_lat,
            'longitude': publish_lon,
            'altitude': altitude,
            'timestamp': getattr(self.vehicle_global_position, 'timestamp', 0)
        }
        
        try:
            if not self.headless:
                self.get_logger().info(f"尝试连接到Socket服务器: {self.socket_host}:{self.socket_port}")
            
            # 设置socket超时
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)  # 5秒超时
            
            # 创建socket连接
            s.connect((self.socket_host, self.socket_port))
            
            if not self.headless:
                self.get_logger().info("成功连接到Socket服务器")
            
            # 发送GPS数据
            data = json.dumps(gps_data).encode('utf-8')
            s.sendall(data)
            
            if not self.headless:
                self.get_logger().info(f"✅ GPS坐标已发布: {gps_data}")
            
            # 关闭连接
            s.close()
            
        except socket.timeout:
            if not self.headless:
                self.get_logger().error(f"Socket连接超时: 无法连接到 {self.socket_host}:{self.socket_port}")
                self.get_logger().error("请确保GPS接收器程序正在运行，并且IP地址和端口正确")
        except ConnectionRefusedError:
            if not self.headless:
                self.get_logger().error(f"连接被拒绝: {self.socket_host}:{self.socket_port}")
                self.get_logger().error("请确保GPS接收器程序正在运行")
        except Exception as e:
            if not self.headless:
                self.get_logger().error(f"发布GPS坐标失败: {str(e)}")
                import traceback
                self.get_logger().error(f"详细错误: {traceback.format_exc()}")

    def timer_callback(self) -> None:
        """定时器的回调函数。"""
        # 检查是否接收到GPS位置数据
        if not self.gps_position_received:
            return
        
        # 检查是否接收到本地位置数据
        if not self.local_position_received:
            return
        
        # 检查是否达到发布间隔
        current_time = self.get_clock().now()
        if self.last_published_time is None:
            # 第一次发布
            self.publish_gps_coordinates()
            self.last_published_time = current_time
        else:
            # 计算距离上次发布的时间
            elapsed_time = (current_time - self.last_published_time).nanoseconds / 1e9
            if elapsed_time >= self.publish_interval:
                # 达到发布间隔，发布GPS坐标
                self.publish_gps_coordinates()
                self.last_published_time = current_time

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