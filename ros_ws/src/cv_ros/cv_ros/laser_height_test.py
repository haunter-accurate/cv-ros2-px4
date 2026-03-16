#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import DistanceSensor
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

        self.range_subscriber = self.create_subscription(
            DistanceSensor, '/fmu/in/distance_sensor', self.range_callback, qos_profile)

        self.current_height = 0.0
        self.last_height = 0.0
        self.height_received = False

        self.get_logger().info("激光高度测试节点已启动")
        self.get_logger().info("订阅话题: /fmu/in/distance_sensor")

        self.timer = self.create_timer(1.0, self.timer_callback)

    def range_callback(self, msg):
        """激光高度回调函数"""
        self.current_height = msg.current_distance
        self.height_received = True

    def timer_callback(self):
        """定时器回调，每秒输出一次高度信息"""
        if self.height_received:
            if self.current_height != self.last_height:
                self.get_logger().info(f"激光高度: {self.current_height:.3f}m")
            else:
                if int(time.time()) % 10 == 0:
                    self.get_logger().info(f"激光高度: {self.current_height:.3f}m (稳定)")
            
            self.last_height = self.current_height
        else:
            self.get_logger().warn("未收到激光高度数据")


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
