#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
import time


class LaserHeightTest(Node):
    """激光高度测试节点，实时获取并显示激光高度"""

    def __init__(self) -> None:
        super().__init__('laser_height_test')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, qos_profile)

        self.current_height = 0.0
        self.last_height = 0.0
        self.height_received = False
        self.msg_count = 0

        self.get_logger().info("激光高度测试节点已启动")
        self.get_logger().info("订阅话题: /fmu/out/vehicle_local_position")
        self.get_logger().info("QoS设置: BEST_EFFORT, VOLATILE, KEEP_LAST(10)")

        self.timer = self.create_timer(1.0, self.timer_callback)

    def local_position_callback(self, msg):
        """本地位置回调函数"""
        self.current_height = msg.z
        self.height_received = True
        self.msg_count += 1
        
        if self.msg_count == 1:
            self.get_logger().info(f"首次收到本地位置数据")
            self.get_logger().info(f"位置: X={msg.x:.3f}m, Y={msg.y:.3f}m, Z={msg.z:.3f}m")
            self.get_logger().info(f"速度: VX={msg.vx:.3f}m/s, VY={msg.vy:.3f}m/s, VZ={msg.vz:.3f}m/s")
            self.get_logger().info(f"状态: xy_valid={msg.xy_valid}, z_valid={msg.z_valid}, v_xy_valid={msg.v_xy_valid}, v_z_valid={msg.v_z_valid}")

    def timer_callback(self):
        """定时器回调，每秒输出一次高度信息"""
        if self.height_received:
            if self.current_height != self.last_height:
                self.get_logger().info(f"本地高度(Z): {self.current_height:.3f}m (消息数: {self.msg_count})")
            else:
                if int(time.time()) % 10 == 0:
                    self.get_logger().info(f"本地高度(Z): {self.current_height:.3f}m (稳定, 消息数: {self.msg_count})")
            
            self.last_height = self.current_height
        else:
            self.get_logger().warn(f"未收到本地位置数据，请检查:")
            self.get_logger().warn("1. PX4是否正常运行")
            self.get_logger().warn("2. 话题名称是否正确 (当前: /fmu/out/vehicle_local_position)")
            self.get_logger().warn("3. 使用 'ros2 topic list' 查看可用话题")
            self.get_logger().warn("4. 使用 'ros2 topic echo /fmu/out/vehicle_local_position' 查看话题内容")


def main(args=None) -> None:
    print('启动激光高度测试节点...')
    rclpy.init(args=args)
    laser_height_test = LaserHeightTest()
    rclpy.spin(laser_height_test)
    laser_height_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
