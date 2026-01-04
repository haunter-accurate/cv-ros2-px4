#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus


class FlightModeMonitor(Node):
    """监控无人机飞行模式的节点。"""

    def __init__(self) -> None:
        super().__init__('flight_mode_monitor')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        self.vehicle_status = VehicleStatus()

        self.timer = self.create_timer(1.0, self.timer_callback)

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def timer_callback(self) -> None:
        if hasattr(self.vehicle_status, 'nav_state'):
            nav_state = self.vehicle_status.nav_state
            self.get_logger().info(f"当前飞行模式: {nav_state}")
        else:
            self.get_logger().info("等待飞行状态数据...")


def main(args=None) -> None:
    print('启动飞行模式监控节点...')
    rclpy.init(args=args)
    flight_mode_monitor = FlightModeMonitor()
    rclpy.spin(flight_mode_monitor)
    flight_mode_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
