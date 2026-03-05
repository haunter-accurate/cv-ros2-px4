#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleControlMode, VehicleStatus, VehicleAttitude
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Bool
import math


class ArucoTrackingOffboard(Node):
    """在offboard模式下控制无人机跟踪ArUco码的节点"""

    def __init__(self) -> None:
        super().__init__('aruco_tracking_offboard')

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
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_control_mode_subscriber = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)

        self.aruco_position_subscriber = self.create_subscription(
            Point, '/detection/aruco_position', self.aruco_position_callback, qos_profile)
        self.aruco_x_axis_yaw_subscriber = self.create_subscription(
            Float32, '/detection/aruco_x_axis_yaw', self.aruco_x_axis_yaw_callback, qos_profile)
        self.aruco_detected_subscriber = self.create_subscription(
            Bool, '/detection/aruco_detected', self.aruco_detected_callback, qos_profile)

        self.declare_parameter('offboard_maintain_time', 2.0)
        self.declare_parameter('target_offset_x', 1.0)
        self.declare_parameter('target_offset_y', 0.0)
        self.declare_parameter('target_offset_z', 3.0)
        self.declare_parameter('position_tolerance', 0.1)
        self.declare_parameter('max_horizontal_speed', 0.5)
        self.declare_parameter('max_vertical_speed', 0.3)
        self.declare_parameter('max_yaw_rate', 0.5)
        self.declare_parameter('pd_kp_x', 0.5)
        self.declare_parameter('pd_kp_y', 0.5)
        self.declare_parameter('pd_kp_z', 0.3)
        self.declare_parameter('pd_kd_x', 0.2)
        self.declare_parameter('pd_kd_y', 0.2)
        self.declare_parameter('pd_kd_z', 0.1)

        self.offboard_maintain_time = self.get_parameter('offboard_maintain_time').value
        self.target_offset_x = self.get_parameter('target_offset_x').value
        self.target_offset_y = self.get_parameter('target_offset_y').value
        self.target_offset_z = self.get_parameter('target_offset_z').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.max_horizontal_speed = self.get_parameter('max_horizontal_speed').value
        self.max_vertical_speed = self.get_parameter('max_vertical_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.pd_kp_x = self.get_parameter('pd_kp_x').value
        self.pd_kp_y = self.get_parameter('pd_kp_y').value
        self.pd_kp_z = self.get_parameter('pd_kp_z').value
        self.pd_kd_x = self.get_parameter('pd_kd_x').value
        self.pd_kd_y = self.get_parameter('pd_kd_y').value
        self.pd_kd_z = self.get_parameter('pd_kd_z').value

        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_control_mode = VehicleControlMode()
        self.vehicle_status = VehicleStatus()
        self.vehicle_attitude = VehicleAttitude()

        self.aruco_position = Point()
        self.aruco_x_axis_yaw = 0.0
        self.aruco_detected = False

        self.offboard_entry_time = None
        self.offboard_mode_maintained = False
        self.target_reached = False

        self.last_position_error = [0.0, 0.0, 0.0]
        self.last_position_error_time = None
        self.current_yaw = 0.0
        self.target_yaw = 0.0

        self.get_logger().info(f"参数配置: offboard_maintain_time={self.offboard_maintain_time}s")
        self.get_logger().info(f"目标偏移: X={self.target_offset_x}m, Y={self.target_offset_y}m, Z={self.target_offset_z}m")
        self.get_logger().info(f"PD参数: Kp=[{self.pd_kp_x}, {self.pd_kp_y}, {self.pd_kp_z}], Kd=[{self.pd_kd_x}, {self.pd_kd_y}, {self.pd_kd_z}]")

        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position

    def vehicle_control_mode_callback(self, vehicle_control_mode):
        self.vehicle_control_mode = vehicle_control_mode

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def vehicle_attitude_callback(self, vehicle_attitude):
        self.vehicle_attitude = vehicle_attitude
        self.current_yaw = self.quaternion_to_yaw(
            vehicle_attitude.q[0], vehicle_attitude.q[1],
            vehicle_attitude.q[2], vehicle_attitude.q[3]
        )

    def aruco_position_callback(self, aruco_position):
        self.aruco_position = aruco_position

    def aruco_x_axis_yaw_callback(self, aruco_x_axis_yaw):
        self.aruco_x_axis_yaw = aruco_x_axis_yaw.data

    def aruco_detected_callback(self, aruco_detected):
        self.aruco_detected = aruco_detected.data

    def quaternion_to_yaw(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('解锁命令已发送')

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('锁定命令已发送')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("正在切换到offboard模式")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("正在切换到降落模式")

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def calculate_pd_control(self, target_x, target_y, target_z):
        current_time = self.get_clock().now()
        
        if self.last_position_error_time is None:
            self.last_position_error_time = current_time
            dt = 0.1
        else:
            dt = (current_time - self.last_position_error_time).nanoseconds / 1e9
            if dt <= 0:
                dt = 0.1
        
        error_x = target_x - self.vehicle_local_position.x
        error_y = target_y - self.vehicle_local_position.y
        error_z = target_z - self.vehicle_local_position.z
        
        error_x = self.normalize_angle(error_x) if abs(error_x) > math.pi else error_x
        
        derivative_x = (error_x - self.last_position_error[0]) / dt
        derivative_y = (error_y - self.last_position_error[1]) / dt
        derivative_z = (error_z - self.last_position_error[2]) / dt
        
        output_x = self.pd_kp_x * error_x + self.pd_kd_x * derivative_x
        output_y = self.pd_kp_y * error_y + self.pd_kd_y * derivative_y
        output_z = self.pd_kp_z * error_z + self.pd_kd_z * derivative_z
        
        self.last_position_error = [error_x, error_y, error_z]
        self.last_position_error_time = current_time
        
        return output_x, output_y, output_z

    def apply_speed_limits(self, vx, vy, vz, dyaw):
        max_v = (vx**2 + vy**2)**0.5
        if max_v > self.max_horizontal_speed:
            scale = self.max_horizontal_speed / max_v
            vx *= scale
            vy *= scale
        
        if abs(vz) > self.max_vertical_speed:
            vz = math.copysign(self.max_vertical_speed, vz)
        
        if abs(dyaw) > self.max_yaw_rate:
            dyaw = math.copysign(self.max_yaw_rate, dyaw)
        
        return vx, vy, vz, dyaw

    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter % 10 == 0:
            is_offboard = self.vehicle_control_mode.flag_control_offboard_enabled if hasattr(self.vehicle_control_mode, 'flag_control_offboard_enabled') else False
            is_armed = self.vehicle_status.arming_state == self.vehicle_status.ARMING_STATE_ARMED if hasattr(self.vehicle_status, 'arming_state') else False
            altitude = self.vehicle_local_position.z if hasattr(self.vehicle_local_position, 'z') else "未知"
            self.get_logger().info(f"OFFBOARD: {is_offboard}, ARMED: {is_armed}, 高度: {altitude}, 检测到ArUco: {self.aruco_detected}")

        is_offboard = hasattr(self.vehicle_control_mode, 'flag_control_offboard_enabled') and \
                     self.vehicle_control_mode.flag_control_offboard_enabled
        
        is_armed = hasattr(self.vehicle_status, 'arming_state') and \
                   self.vehicle_status.arming_state == self.vehicle_status.ARMING_STATE_ARMED

        if is_offboard:
            if self.offboard_entry_time is None:
                self.offboard_entry_time = self.get_clock().now()
                self.get_logger().info("已进入offboard模式，开始计时")
            else:
                current_time = self.get_clock().now()
                elapsed_time = (current_time - self.offboard_entry_time).nanoseconds / 1e9
                
                if elapsed_time >= self.offboard_maintain_time and not self.offboard_mode_maintained:
                    self.offboard_mode_maintained = True
                    self.get_logger().info(f"offboard模式已维持 {self.offboard_maintain_time} 秒，开始ArUco跟踪控制")
        else:
            if self.offboard_entry_time is not None:
                self.offboard_entry_time = None
                self.offboard_mode_maintained = False
                self.get_logger().info("已退出offboard模式，重置计时")

        if self.offboard_mode_maintained and is_armed:
            if self.aruco_detected:
                aruco_x = self.aruco_position.x
                aruco_y = self.aruco_position.y
                aruco_z = self.aruco_position.z
                
                target_x = self.vehicle_local_position.x + aruco_x - self.target_offset_x
                target_y = self.vehicle_local_position.y + aruco_y - self.target_offset_y
                target_z = self.vehicle_local_position.z + aruco_z - self.target_offset_z
                
                self.target_yaw = self.aruco_x_axis_yaw
                
                vx, vy, vz = self.calculate_pd_control(target_x, target_y, target_z)
                
                yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
                dyaw = yaw_error * 0.5
                
                vx, vy, vz, dyaw = self.apply_speed_limits(vx, vy, vz, dyaw)
                
                new_x = self.vehicle_local_position.x
                new_y = self.vehicle_local_position.y
                new_z = self.vehicle_local_position.z
                new_yaw = self.normalize_angle(self.current_yaw + dyaw * 0.1)
                
                self.publish_position_setpoint(new_x, new_y, new_z, new_yaw)
                
                distance_to_target = ((self.vehicle_local_position.x - target_x) ** 2 +
                                     (self.vehicle_local_position.y - target_y) ** 2 +
                                     (self.vehicle_local_position.z - target_z) ** 2) ** 0.5
                
                if self.offboard_setpoint_counter % 10 == 0:
                    self.get_logger().info(f"目标: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})m, "
                                         f"当前: ({self.vehicle_local_position.x:.2f}, {self.vehicle_local_position.y:.2f}, {self.vehicle_local_position.z:.2f})m, "
                                         f"误差: {distance_to_target:.3f}m, Yaw: {math.degrees(new_yaw):.1f}°")
            else:
                self.get_logger().warn("未检测到ArUco码，保持当前位置和朝向")
                self.publish_position_setpoint(
                    self.vehicle_local_position.x,
                    self.vehicle_local_position.y,
                    self.vehicle_local_position.z,
                    self.current_yaw
                )
        elif is_offboard:
            self.get_logger().info("处于OFFBOARD模式，但飞机尚未解锁")
        else:
            self.get_logger().info("未处于OFFBOARD模式，等待遥控器切换")

        if self.offboard_setpoint_counter < 100:
            self.offboard_setpoint_counter += 1

    def destroy_node(self):
        super().destroy_node()


def main(args=None) -> None:
    print('启动ArUco跟踪offboard控制节点...')
    rclpy.init(args=args)
    aruco_tracking_offboard = ArucoTrackingOffboard()
    rclpy.spin(aruco_tracking_offboard)
    aruco_tracking_offboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
