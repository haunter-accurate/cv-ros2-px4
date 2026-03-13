#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleControlMode, VehicleStatus
from geometry_msgs.msg import Vector3, Bool
import math


class SimpleVelocityControl(Node):
    """简单速度控制节点，在offboard模式下发布速度控制命令"""

    def __init__(self) -> None:
        super().__init__('simple_velocity_control')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        self.vehicle_control_mode_subscriber = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        self.velocity_command_subscriber = self.create_subscription(
            Vector3, '/control/velocity_command', self.velocity_command_callback, qos_profile)
        self.control_mode_subscriber = self.create_subscription(
            Bool, '/control/velocity_mode_enabled', self.control_mode_callback, qos_profile)

        self.declare_parameter('max_horizontal_speed', 1.0)
        self.declare_parameter('max_vertical_speed', 0.5)

        self.max_horizontal_speed = self.get_parameter('max_horizontal_speed').value
        self.max_vertical_speed = self.get_parameter('max_vertical_speed').value

        self.vehicle_control_mode = VehicleControlMode()
        self.vehicle_status = VehicleStatus()

        self.velocity_command = Vector3()
        self.velocity_command.x = 0.0
        self.velocity_command.y = 0.0
        self.velocity_command.z = 0.0

        self.velocity_mode_enabled = False
        self.current_yaw = 0.0

        self.offboard_setpoint_counter = 0

        self.get_logger().info(f"速度控制节点已启动")
        self.get_logger().info(f"速度限制: 水平={self.max_horizontal_speed}m/s, 垂直={self.max_vertical_speed}m/s")

        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_control_mode_callback(self, vehicle_control_mode):
        self.vehicle_control_mode = vehicle_control_mode

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def velocity_command_callback(self, velocity_command):
        self.velocity_command = velocity_command

    def control_mode_callback(self, control_mode):
        self.velocity_mode_enabled = control_mode.data

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float, yaw: float):
        msg = TrajectorySetpoint()
        msg.velocity = [vx, vy, vz]
        msg.acceleration = [0.0, 0.0, 0.0]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def limit_velocity(self, vx: float, vy: float, vz: float):
        """限制速度到合理范围内"""
        horizontal_speed = (vx**2 + vy**2)**0.5
        
        if horizontal_speed > self.max_horizontal_speed:
            scale = self.max_horizontal_speed / horizontal_speed
            vx *= scale
            vy *= scale
        
        if abs(vz) > self.max_vertical_speed:
            vz = math.copysign(self.max_vertical_speed, vz)
        
        return vx, vy, vz

    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()

        is_offboard = hasattr(self.vehicle_control_mode, 'flag_control_offboard_enabled') and \
                     self.vehicle_control_mode.flag_control_offboard_enabled
        is_armed = hasattr(self.vehicle_status, 'arming_state') and \
                   self.vehicle_status.arming_state == self.vehicle_status.ARMING_STATE_ARMED

        if self.offboard_setpoint_counter % 10 == 0:
            self.get_logger().info(
                f"OFFBOARD: {is_offboard}, ARMED: {is_armed}, "
                f"速度模式: {self.velocity_mode_enabled}, "
                f"输入速度: ({self.velocity_command.x:.2f}, {self.velocity_command.y:.2f}, {self.velocity_command.z:.2f})m/s"
            )

        if is_offboard and self.velocity_mode_enabled:
            vx, vy, vz = self.limit_velocity(
                self.velocity_command.x,
                self.velocity_command.y,
                self.velocity_command.z
            )

            self.publish_velocity_setpoint(vx, vy, vz, self.current_yaw)

            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(
                    f"发布速度命令: VX={vx:.2f}, VY={vy:.2f}, VZ={vz:.2f}m/s, Yaw={math.degrees(self.current_yaw):.1f}°"
                )
        else:
            if self.offboard_setpoint_counter % 10 == 0:
                if not is_offboard:
                    self.get_logger().info("未处于OFFBOARD模式，不发布速度命令")
                elif not self.velocity_mode_enabled:
                    self.get_logger().info("速度模式未启用，不发布速度命令")

        self.offboard_setpoint_counter += 1
        if self.offboard_setpoint_counter >= 100:
            self.offboard_setpoint_counter = 0
            
    def destroy_node(self):
        super().destroy_node()


def main(args=None) -> None:
    print('启动简单速度控制节点...')
    rclpy.init(args=args)
    simple_velocity_control = SimpleVelocityControl()
    rclpy.spin(simple_velocity_control)
    simple_velocity_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
