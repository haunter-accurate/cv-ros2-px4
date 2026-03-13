#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleControlMode
from geometry_msgs.msg import Vector3, Bool


class VelocityControlTest(Node):
    """速度控制测试节点，按照时间序列发布速度命令"""

    def __init__(self) -> None:
        super().__init__('velocity_control_test')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vehicle_control_mode_subscriber = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)

        self.velocity_command_publisher = self.create_publisher(
            Vector3, '/control/velocity_command', qos_profile)
        self.velocity_mode_publisher = self.create_publisher(
            Bool, '/control/velocity_mode_enabled', qos_profile)

        self.vehicle_control_mode = VehicleControlMode()
        self.offboard_entry_time = None
        self.test_started = False
        self.test_phase = 0
        self.phase_start_time = None

        self.get_logger().info("速度控制测试节点已启动")
        self.get_logger().info("测试序列：")
        self.get_logger().info("1. 进入OFFBOARD模式后等待2秒")
        self.get_logger().info("2. 启用速度控制模式，设置速度 (0.5, 0, 0)")
        self.get_logger().info("3. 2秒后设置速度 (0, 0.5, 0)")
        self.get_logger().info("4. 2秒后设置速度 (0, 0, 0)")
        self.get_logger().info("5. 1秒后禁用速度控制模式")

        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_control_mode_callback(self, vehicle_control_mode):
        self.vehicle_control_mode = vehicle_control_mode

    def publish_velocity_command(self, x: float, y: float, z: float):
        msg = Vector3()
        msg.x = x
        msg.y = y
        msg.z = z
        self.velocity_command_publisher.publish(msg)

    def publish_velocity_mode(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.velocity_mode_publisher.publish(msg)

    def timer_callback(self) -> None:
        is_offboard = hasattr(self.vehicle_control_mode, 'flag_control_offboard_enabled') and \
                     self.vehicle_control_mode.flag_control_offboard_enabled

        if not self.test_started:
            if is_offboard:
                if self.offboard_entry_time is None:
                    self.offboard_entry_time = self.get_clock().now()
                    self.get_logger().info("检测到OFFBOARD模式，开始2秒倒计时...")
                else:
                    elapsed_time = (self.get_clock().now() - self.offboard_entry_time).nanoseconds / 1e9
                    if elapsed_time >= 2.0:
                        self.test_started = True
                        self.test_phase = 1
                        self.phase_start_time = self.get_clock().now()
                        self.get_logger().info("2秒已到，开始测试阶段1")
        else:
            elapsed_time = (self.get_clock().now() - self.phase_start_time).nanoseconds / 1e9

            if self.test_phase == 1:
                if elapsed_time >= 2.0:
                    self.test_phase = 2
                    self.phase_start_time = self.get_clock().now()
                    self.get_logger().info("2秒已到，进入测试阶段2")
                else:
                    self.publish_velocity_command(0.5, 0.0, 0.0)
                    self.publish_velocity_mode(True)
                    if int(elapsed_time * 10) % 10 == 0:
                        self.get_logger().info(f"阶段1 ({elapsed_time:.1f}s): 速度=(0.5, 0, 0)")

            elif self.test_phase == 2:
                if elapsed_time >= 2.0:
                    self.test_phase = 3
                    self.phase_start_time = self.get_clock().now()
                    self.get_logger().info("2秒已到，进入测试阶段3")
                else:
                    self.publish_velocity_command(0.0, 0.5, 0.0)
                    self.publish_velocity_mode(True)
                    if int(elapsed_time * 10) % 10 == 0:
                        self.get_logger().info(f"阶段2 ({elapsed_time:.1f}s): 速度=(0, 0.5, 0)")

            elif self.test_phase == 3:
                if elapsed_time >= 2.0:
                    self.test_phase = 4
                    self.phase_start_time = self.get_clock().now()
                    self.get_logger().info("2秒已到，进入测试阶段4")
                else:
                    self.publish_velocity_command(0.0, 0.0, 0.0)
                    self.publish_velocity_mode(True)
                    if int(elapsed_time * 10) % 10 == 0:
                        self.get_logger().info(f"阶段3 ({elapsed_time:.1f}s): 速度=(0, 0, 0)")

            elif self.test_phase == 4:
                if elapsed_time >= 1.0:
                    self.test_phase = 5
                    self.get_logger().info("1秒已到，测试完成，禁用速度控制模式")
                else:
                    self.publish_velocity_command(0.0, 0.0, 0.0)
                    self.publish_velocity_mode(True)
                    if int(elapsed_time * 10) % 10 == 0:
                        self.get_logger().info(f"阶段4 ({elapsed_time:.1f}s): 速度=(0, 0, 0)")

            elif self.test_phase == 5:
                self.publish_velocity_command(0.0, 0.0, 0.0)
                self.publish_velocity_mode(False)
                self.get_logger().info("测试完成，速度控制模式已禁用")

    def destroy_node(self):
        self.publish_velocity_command(0.0, 0.0, 0.0)
        self.publish_velocity_mode(False)
        super().destroy_node()


def main(args=None) -> None:
    print('启动速度控制测试节点...')
    rclpy.init(args=args)
    velocity_control_test = VelocityControlTest()
    rclpy.spin(velocity_control_test)
    velocity_control_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
