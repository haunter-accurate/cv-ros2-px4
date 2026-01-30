#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import RcChannels


class RcChannelsMonitor(Node):
    """实时读取并输出遥控器1-8通道数据的节点。"""

    def __init__(self) -> None:
        super().__init__('rc_channels_monitor')

        # 配置QoS配置文件
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 创建订阅者，订阅遥控器通道数据
        self.rc_channels_subscriber = self.create_subscription(
            RcChannels, '/fmu/out/rc_channels', self.rc_channels_callback, qos_profile)

        # 参数配置
        self.declare_parameter('headless', False)  # 是否启用headless模式
        self.declare_parameter('update_rate', 10)  # 更新频率（Hz）
        
        # 获取参数
        self.headless = self.get_parameter('headless').value
        self.update_rate = self.get_parameter('update_rate').value

        # 初始化变量
        self.last_update_time = self.get_clock().now()
        self.update_interval = 1.0 / self.update_rate  # 更新间隔（秒）
        self.channel_data = [0.0] * 8  # 存储通道1-8的数据
        self.valid_channels = False  # 标记通道数据是否有效
        
        # 输出初始化信息
        if not self.headless:
            self.get_logger().info("RC Channels Monitor 启动")
            self.get_logger().info(f"参数配置: headless={self.headless}, update_rate={self.update_rate}Hz")
            self.get_logger().info("正在等待遥控器通道数据...")

    def rc_channels_callback(self, rc_channels):
        """RcChannels话题订阅者的回调函数。"""
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_update_time).nanoseconds / 1e9
        
        # 按照指定的更新频率输出数据
        if time_diff >= self.update_interval:
            # 更新通道数据
            self.channel_data[0] = rc_channels.channel[0] if len(rc_channels.channel) > 0 else 0.0
            self.channel_data[1] = rc_channels.channel[1] if len(rc_channels.channel) > 1 else 0.0
            self.channel_data[2] = rc_channels.channel[2] if len(rc_channels.channel) > 2 else 0.0
            self.channel_data[3] = rc_channels.channel[3] if len(rc_channels.channel) > 3 else 0.0
            self.channel_data[4] = rc_channels.channel[4] if len(rc_channels.channel) > 4 else 0.0
            self.channel_data[5] = rc_channels.channel[5] if len(rc_channels.channel) > 5 else 0.0
            self.channel_data[6] = rc_channels.channel[6] if len(rc_channels.channel) > 6 else 0.0
            self.channel_data[7] = rc_channels.channel[7] if len(rc_channels.channel) > 7 else 0.0
            
            self.valid_channels = rc_channels.timestamp != 0
            
            # 输出通道数据
            self.print_rc_channels()
            
            # 更新最后更新时间
            self.last_update_time = current_time

    def print_rc_channels(self):
        """输出遥控器通道数据。"""
        if not self.headless:
            if self.valid_channels:
                self.get_logger().info("===========================================")
                self.get_logger().info("遥控器通道数据:")
                self.get_logger().info("===========================================")
                self.get_logger().info(f"通道1: {self.channel_data[0]:.2f}")
                self.get_logger().info(f"通道2: {self.channel_data[1]:.2f}")
                self.get_logger().info(f"通道3: {self.channel_data[2]:.2f}")
                self.get_logger().info(f"通道4: {self.channel_data[3]:.2f}")
                self.get_logger().info(f"通道5: {self.channel_data[4]:.2f}")
                self.get_logger().info(f"通道6: {self.channel_data[5]:.2f}")
                self.get_logger().info(f"通道7: {self.channel_data[6]:.2f}")
                self.get_logger().info(f"通道8: {self.channel_data[7]:.2f}")
                self.get_logger().info("===========================================")
            else:
                self.get_logger().warn("未接收到有效的遥控器通道数据")

    def destroy_node(self):
        """节点销毁时的清理工作。"""
        if not self.headless:
            self.get_logger().info("RC Channels Monitor 关闭")
        super().destroy_node()


def main(args=None) -> None:
    print('启动RC Channels Monitor节点...')
    rclpy.init(args=args)
    rc_channels_monitor = RcChannelsMonitor()
    rclpy.spin(rc_channels_monitor)
    rc_channels_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"错误: {e}")