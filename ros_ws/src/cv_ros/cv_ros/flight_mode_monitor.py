#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleControlMode, VehicleLocalPosition


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

        self.vehicle_control_mode_subscriber = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)
        
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

        self.vehicle_control_mode = VehicleControlMode()
        self.vehicle_local_position = VehicleLocalPosition()

        self.timer = self.create_timer(1.0, self.timer_callback)

    def vehicle_control_mode_callback(self, vehicle_control_mode):
        self.vehicle_control_mode = vehicle_control_mode

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position

    def timer_callback(self) -> None:
        if hasattr(self.vehicle_control_mode, 'flag_control_offboard_enabled'):
            is_offboard = self.vehicle_control_mode.flag_control_offboard_enabled
            is_armed = self.vehicle_control_mode.flag_armed
            
            mode_info = []
            if is_armed:
                mode_info.append("已解锁")
            else:
                mode_info.append("未解锁")
            
            if is_offboard:
                mode_info.append("OFFBOARD模式")
            else:
                mode_info.append("非OFFBOARD模式")
            
            self.get_logger().info(f"当前状态: {', '.join(mode_info)}")
        else:
            self.get_logger().info("等待飞行控制模式数据...")



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
