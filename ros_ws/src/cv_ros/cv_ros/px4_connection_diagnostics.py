#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleCommandAck, VehicleAttitude


class PX4ConnectionDiagnostics(Node):
    """PX4连接诊断节点。"""

    def __init__(self) -> None:
        super().__init__('px4_connection_diagnostics')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_attitude = VehicleAttitude()
        
        self.status_received = False
        self.position_received = False
        self.attitude_received = False
        
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.start_time = self.get_clock().now()
        
        self.get_logger().info("PX4连接诊断节点已启动，正在检查通信状态...")

    def vehicle_status_callback(self, vehicle_status):
        self.status_received = True
        self.vehicle_status = vehicle_status
        
        if self.status_received and not hasattr(self, 'first_status_received'):
            self.first_status_received = True
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.get_logger().info(f"✓ VehicleStatus话题已连接 (耗时: {elapsed:.2f}秒)")

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.position_received = True
        self.vehicle_local_position = vehicle_local_position
        
        if self.position_received and not hasattr(self, 'first_position_received'):
            self.first_position_received = True
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.get_logger().info(f"✓ VehicleLocalPosition话题已连接 (耗时: {elapsed:.2f}秒)")

    def vehicle_attitude_callback(self, vehicle_attitude):
        self.attitude_received = True
        self.vehicle_attitude = vehicle_attitude
        
        if self.attitude_received and not hasattr(self, 'first_attitude_received'):
            self.first_attitude_received = True
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.get_logger().info(f"✓ VehicleAttitude话题已连接 (耗时: {elapsed:.2f}秒)")

    def quaternion_to_euler(self, q):
        """将四元数转换为欧拉角（滚转、俯仰、偏航）"""
        w, x, y, z = q
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return roll, pitch, yaw

    def timer_callback(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"诊断报告 (运行时间: {elapsed:.1f}秒)")
        self.get_logger().info("=" * 60)
        
        # 检查话题连接状态
        self.get_logger().info(f"VehicleStatus话题: {'✓ 已连接' if self.status_received else '✗ 未连接'}")
        self.get_logger().info(f"VehicleLocalPosition话题: {'✓ 已连接' if self.position_received else '✗ 未连接'}")
        self.get_logger().info(f"VehicleAttitude话题: {'✓ 已连接' if self.attitude_received else '✗ 未连接'}")
        
        # 显示详细状态信息
        if self.status_received:
            self.get_logger().info(f"导航状态 (nav_state): {self.vehicle_status.nav_state}")
            self.get_logger().info(f"系统状态 (system_status): {self.vehicle_status.system_status}")
            self.get_logger().info(f"arming状态 (arming_state): {self.vehicle_status.arming_state}")
            self.get_logger().info(f"飞行模式 (flight_mode): {self.vehicle_status.flight_mode}")
            
            # 解释nav_state的值
            nav_state = self.vehicle_status.nav_state
            if nav_state == 0:
                self.get_logger().warn("  ⚠ nav_state=0 表示未定义或未初始化状态")
            elif nav_state == 1:
                self.get_logger().info("  ✓ nav_state=1 表示手动模式")
            elif nav_state == 2:
                self.get_logger().info("  ✓ nav_state=2 表示高度控制模式")
            elif nav_state == 3:
                self.get_logger().info("  ✓ nav_state=3 表示位置控制模式")
            elif nav_state == 4:
                self.get_logger().info("  ✓ nav_state=4 表示任务模式")
            elif nav_state == 6:
                self.get_logger().info("  ✓ nav_state=6 表示OFFBOARD模式")
            else:
                self.get_logger().warn(f"  ? nav_state={nav_state} (未知状态)")
        
        if self.position_received:
            self.get_logger().info(f"位置: X={self.vehicle_local_position.x:.2f}, Y={self.vehicle_local_position.y:.2f}, Z={self.vehicle_local_position.z:.2f}")
        
        # 显示姿态角信息
        attitude_available = False
        if self.attitude_received and hasattr(self.vehicle_attitude, 'q'):
            q = self.vehicle_attitude.q
            # 检查四元数是否全为0
            if not all(abs(val) < 1e-6 for val in q):
                roll, pitch, yaw = self.quaternion_to_euler(q)
                self.get_logger().info(f"姿态角 (从VehicleAttitude): 滚转={roll:.2f}, 俯仰={pitch:.2f}, 偏航={yaw:.2f}")
                attitude_available = True
            else:
                self.get_logger().warn("  ⚠ VehicleAttitude四元数全为0")
        
        if not attitude_available and self.position_received:
            if hasattr(self.vehicle_local_position, 'x_ang'):
                self.get_logger().info(f"姿态角 (从VehicleLocalPosition): 滚转={self.vehicle_local_position.x_ang:.2f}, 俯仰={self.vehicle_local_position.y_ang:.2f}, 偏航={self.vehicle_local_position.z_ang:.2f}")
            elif hasattr(self.vehicle_local_position, 'heading'):
                self.get_logger().info(f"偏航角 (从VehicleLocalPosition): {self.vehicle_local_position.heading:.2f}")
        
        # 诊断建议
        self.get_logger().info("=" * 60)
        self.get_logger().info("诊断建议:")
        
        if not self.status_received and not self.position_received and not self.attitude_received:
            self.get_logger().error("✗ 所有话题都未连接！")
            self.get_logger().error("  请检查:")
            self.get_logger().error("  1. Micro XRCE-DDS Agent是否正在运行")
            self.get_logger().error("  2. PX4飞控是否已上电")
            self.get_logger().error("  3. 串口连接是否正确 (/dev/ttyAMA0)")
            self.get_logger().error("  4. 波特率设置是否正确 (921600)")
        elif not self.status_received:
            self.get_logger().warn("⚠ VehicleStatus话题未连接")
            self.get_logger().warn("  可能原因: PX4飞控未正确启动或通信配置错误")
        elif not self.attitude_received:
            self.get_logger().warn("⚠ VehicleAttitude话题未连接")
            self.get_logger().warn("  可能原因: PX4飞控姿态估计系统未初始化或通信问题")
        elif self.vehicle_status.nav_state == 0:
            self.get_logger().warn("⚠ 导航状态为0，可能原因:")
            self.get_logger().warn("  1. PX4飞控尚未完成初始化")
            self.get_logger().warn("  2. GPS未锁定（如果需要GPS）")
            self.get_logger().warn("  3. 传感器校准未完成")
            self.get_logger().warn("  4. 使用遥控器切换飞行模式")
        
        self.get_logger().info("=" * 60)


def main(args=None) -> None:
    print('启动PX4连接诊断节点...')
    rclpy.init(args=args)
    diagnostics = PX4ConnectionDiagnostics()
    rclpy.spin(diagnostics)
    diagnostics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
