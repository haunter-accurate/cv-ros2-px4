#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import ManualControlSetpoint


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
        self.manual_control_subscriber = self.create_subscription(
            ManualControlSetpoint, '/fmu/out/manual_control_setpoint', self.manual_control_callback, qos_profile)

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

    def manual_control_callback(self, manual_control):
        """ManualControlSetpoint话题订阅者的回调函数。"""
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_update_time).nanoseconds / 1e9
        
        # 按照指定的更新频率输出数据
        if time_diff >= self.update_interval:
            # 更新通道数据
            # ManualControlSetpoint 直接提供 -1..1 范围的值
            # 映射通道：
            # 1: roll
            # 2: pitch
            # 3: yaw
            # 4: throttle
            # 5: flaps
            # 6: aux1
            # 7: aux2
            # 8: aux3
            
            # 检查值是否有效（不是NaN）
            import math
            
            # 通道1: roll
            self.channel_data[0] = manual_control.roll if not math.isnan(manual_control.roll) else 0.0
            
            # 通道2: pitch
            self.channel_data[1] = manual_control.pitch if not math.isnan(manual_control.pitch) else 0.0
            
            # 通道3: yaw
            self.channel_data[2] = manual_control.yaw if not math.isnan(manual_control.yaw) else 0.0
            
            # 通道4: throttle
            self.channel_data[3] = manual_control.throttle if not math.isnan(manual_control.throttle) else 0.0
            
            # 通道5: flaps
            self.channel_data[4] = manual_control.flaps if not math.isnan(manual_control.flaps) else 0.0
            
            # 通道6: aux1
            self.channel_data[5] = manual_control.aux1 if not math.isnan(manual_control.aux1) else 0.0
            
            # 通道7: aux2
            self.channel_data[6] = manual_control.aux2 if not math.isnan(manual_control.aux2) else 0.0
            
            # 通道8: aux3
            self.channel_data[7] = manual_control.aux3 if not math.isnan(manual_control.aux3) else 0.0
            
            self.valid_channels = manual_control.timestamp != 0 and manual_control.valid
            
            # 输出通道数据
            self.print_rc_channels(manual_control)
            
            # 更新最后更新时间
            self.last_update_time = current_time

    def print_rc_channels(self, manual_control):
        """输出遥控器通道数据。"""
        if not self.headless:
            if self.valid_channels:
                self.get_logger().info("===========================================")
                self.get_logger().info("遥控器通道数据:")
                self.get_logger().info("===========================================")
                self.get_logger().info(f"通道1 (Roll): {self.channel_data[0]:.2f}")
                self.get_logger().info(f"通道2 (Pitch): {self.channel_data[1]:.2f}")
                self.get_logger().info(f"通道3 (Yaw): {self.channel_data[2]:.2f}")
                self.get_logger().info(f"通道4 (Throttle): {self.channel_data[3]:.2f}")
                self.get_logger().info(f"通道5 (Flaps): {self.channel_data[4]:.2f}")
                self.get_logger().info(f"通道6 (Aux1): {self.channel_data[5]:.2f}")
                self.get_logger().info(f"通道7 (Aux2): {self.channel_data[6]:.2f}")
                self.get_logger().info(f"通道8 (Aux3): {self.channel_data[7]:.2f}")
                
                # 显示额外的遥控器状态信息
                self.get_logger().info("-------------------------------------------")
                self.get_logger().info("遥控器状态:")
                
                # 显示数据源
                data_source_map = {
                    0: "未知",
                    1: "RC (遥控器)",
                    2: "MAVLink 实例 0",
                    3: "MAVLink 实例 1",
                    4: "MAVLink 实例 2",
                    5: "MAVLink 实例 3",
                    6: "MAVLink 实例 4",
                    7: "MAVLink 实例 5"
                }
                data_source = data_source_map.get(manual_control.data_source, f"未知 ({manual_control.data_source})")
                self.get_logger().info(f"  数据源: {data_source}")
                
                # 显示其他状态信息
                self.get_logger().info(f"  摇杆移动中: {manual_control.sticks_moving}")
                self.get_logger().info(f"  按钮状态: {manual_control.buttons}")
                
                # 显示额外的辅助通道
                import math
                if not math.isnan(manual_control.aux4):
                    self.get_logger().info(f"  额外通道: Aux4={manual_control.aux4:.2f}")
                if not math.isnan(manual_control.aux5):
                    self.get_logger().info(f"  额外通道: Aux5={manual_control.aux5:.2f}")
                if not math.isnan(manual_control.aux6):
                    self.get_logger().info(f"  额外通道: Aux6={manual_control.aux6:.2f}")
                
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