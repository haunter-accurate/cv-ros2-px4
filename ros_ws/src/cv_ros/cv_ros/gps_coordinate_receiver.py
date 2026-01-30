#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import json


class GpsCoordinateReceiver(Node):
    """接收GPS坐标并设置标志位的节点。"""

    def __init__(self) -> None:
        super().__init__('gps_coordinate_receiver')

        # 参数配置
        self.declare_parameter('headless', False)  # 是否启用headless模式
        self.declare_parameter('listen_host', 'localhost')  # 监听地址
        self.declare_parameter('listen_port', 5000)  # 监听端口
        self.declare_parameter('buffer_size', 1024)  # 缓冲区大小
        
        # 获取参数
        self.headless = self.get_parameter('headless').value
        self.listen_host = self.get_parameter('listen_host').value
        self.listen_port = self.get_parameter('listen_port').value
        self.buffer_size = self.get_parameter('buffer_size').value

        # 初始化变量
        self.received_flag = 0  # 标志位，0表示未接收，1表示已接收
        self.received_gps_data = None  # 存储接收到的GPS数据
        
        # 输出初始化信息
        if not self.headless:
            self.get_logger().info("GPS Coordinate Receiver 启动")
            self.get_logger().info(f"参数配置: headless={self.headless}")
            self.get_logger().info(f"监听配置: {self.listen_host}:{self.listen_port}")
            self.get_logger().info(f"标志位初始值: {self.received_flag}")
            self.get_logger().info("正在等待GPS坐标数据...")

        # 启动socket服务器
        self.start_socket_server()

    def start_socket_server(self):
        """启动socket服务器。"""
        try:
            # 创建socket服务器
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind((self.listen_host, self.listen_port))
                s.listen()
                if not self.headless:
                    self.get_logger().info(f"Socket服务器已启动，监听 {self.listen_host}:{self.listen_port}")
                
                # 等待连接
                conn, addr = s.accept()
                with conn:
                    if not self.headless:
                        self.get_logger().info(f"接收到来自 {addr} 的连接")
                    
                    # 接收数据
                    data = conn.recv(self.buffer_size)
                    if data:
                        try:
                            # 解析GPS数据
                            gps_data = json.loads(data.decode('utf-8'))
                            self.received_gps_data = gps_data
                            self.received_flag = 1  # 设置标志位为1
                            
                            if not self.headless:
                                self.get_logger().info("✅ 成功接收GPS坐标数据")
                                self.get_logger().info(f"   纬度: {gps_data['latitude']}")
                                self.get_logger().info(f"   经度: {gps_data['longitude']}")
                                self.get_logger().info(f"   高度: {gps_data['altitude']}")
                                self.get_logger().info(f"   标志位已设置为: {self.received_flag}")
                        except json.JSONDecodeError as e:
                            if not self.headless:
                                self.get_logger().error(f"解析GPS数据失败: {str(e)}")
                    else:
                        if not self.headless:
                            self.get_logger().warn("接收到空数据")
        except Exception as e:
            if not self.headless:
                self.get_logger().error(f"Socket服务器错误: {str(e)}")

    def get_received_flag(self):
        """获取标志位状态。"""
        return self.received_flag

    def get_received_gps_data(self):
        """获取接收到的GPS数据。"""
        return self.received_gps_data

    def destroy_node(self):
        """节点销毁时的清理工作。"""
        super().destroy_node()


def main(args=None) -> None:
    print('启动GPS Coordinate Receiver节点...')
    rclpy.init(args=args)
    gps_coordinate_receiver = GpsCoordinateReceiver()
    
    # 运行直到接收到数据
    while gps_coordinate_receiver.get_received_flag() == 0:
        rclpy.spin_once(gps_coordinate_receiver, timeout_sec=0.1)
    
    # 接收到数据后，保持节点运行以显示信息
    if not gps_coordinate_receiver.headless:
        print("\nGPS坐标接收成功！")
        print(f"标志位状态: {gps_coordinate_receiver.get_received_flag()}")
        gps_data = gps_coordinate_receiver.get_received_gps_data()
        if gps_data:
            print(f"接收到的GPS数据: {gps_data}")
        print("\n按Ctrl+C退出...")
    
    try:
        rclpy.spin(gps_coordinate_receiver)
    except KeyboardInterrupt:
        pass
    finally:
        gps_coordinate_receiver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)