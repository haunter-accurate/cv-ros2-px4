#!/usr/bin/env python3

import rclpy
import math
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

    def quaternion_to_euler(self, q):
        """将四元数转换为欧拉角（滚转、俯仰、偏航）"""
        w, x, y, z = q
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return roll, pitch, yaw

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
            # 输出姿态角信息
            if hasattr(self.vehicle_local_position, 'x_ang'):
                roll = self.vehicle_local_position.x_ang
                pitch = self.vehicle_local_position.y_ang
                yaw = self.vehicle_local_position.z_ang
                self.get_logger().info(f"姿态角: 滚转={roll:.2f}, 俯仰={pitch:.2f}, 偏航={yaw:.2f}")
            elif hasattr(self.vehicle_local_position, 'q'):
                # 从四元数转换为欧拉角
                q = self.vehicle_local_position.q
                roll, pitch, yaw = self.quaternion_to_euler(q)
                self.get_logger().info(f"姿态角 (从四元数转换): 滚转={roll:.2f}, 俯仰={pitch:.2f}, 偏航={yaw:.2f}")
            else:
                # 检查是否有其他姿态相关属性
                if hasattr(self.vehicle_local_position, 'heading'):
                    self.get_logger().info(f"偏航角: {self.vehicle_local_position.heading:.2f}")
                else:
                    self.get_logger().info("未找到姿态角数据")
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
