#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleControlMode
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool
import math


class ArucoCBTracking(Node):
    """
    ArUco码追踪节点，基于匿名科创的色块跟踪控制逻辑
    实现旋转解耦、地面偏差计算和PD速度控制
    """

    def __init__(self) -> None:
        super().__init__('aruco_cbtracking')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.velocity_command_publisher = self.create_publisher(
            Vector3, '/control/velocity_command', qos_profile)
        self.velocity_mode_publisher = self.create_publisher(
            Bool, '/control/velocity_mode_enabled', qos_profile)

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)

        self.aruco_position_subscriber = self.create_subscription(
            Point, '/detection/aruco_position', self.aruco_position_callback, qos_profile)
        self.aruco_detected_subscriber = self.create_subscription(
            Bool, '/detection/aruco_detected', self.aruco_detected_callback, qos_profile)
        self.vehicle_control_mode_subscriber = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, qos_profile)

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_control_mode = VehicleControlMode()

        self.aruco_position = Point()
        self.aruco_detected = False

        self.ground_height_offset = None

        self.declare_parameter('enable_control', True)
        self.enable_control = self.get_parameter('enable_control').value

        self.offboard_entry_time = None
        self.control_enabled = False

        self.get_logger().info("ArUco追踪节点已启动（基于匿名科创CBTracking控制逻辑）")
        self.get_logger().info(f"控制使能: {self.enable_control}")

        self.timer = self.create_timer(0.02, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position
        
        if self.ground_height_offset is None:
            self.ground_height_offset = vehicle_local_position.z
            self.get_logger().info(f"地面高度已记录: {self.ground_height_offset:.2f}m")

    def vehicle_attitude_callback(self, vehicle_attitude):
        self.vehicle_attitude = vehicle_attitude

    def vehicle_control_mode_callback(self, vehicle_control_mode):
        self.vehicle_control_mode = vehicle_control_mode

    def aruco_position_callback(self, aruco_position):
        self.aruco_position = aruco_position

    def aruco_detected_callback(self, aruco_detected):
        self.aruco_detected = aruco_detected.data

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def limit(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))

    def lpf_1(self, alpha, dt, input_val, output_val):
        """一阶低通滤波器"""
        return output_val + alpha * (input_val - output_val)

    def timer_callback(self) -> None:
        dt = 0.02

        if not hasattr(self, 'cbtracking_state'):
            self.cbtracking_state = {
                'target_loss_hold_time': 0,
                'target_loss': False,
                'decou_pos_pixel_lpf': [[0.0, 0.0], [0.0, 0.0]],
                'ground_pos_err_old': [0.0, 0.0],
                'ground_pos_err_d': [0.0, 0.0],
                'target_gnd_velocity': [0.0, 0.0],
                'exp_velocity': [0.0, 0.0]
            }

        state = self.cbtracking_state

        if hasattr(self.vehicle_attitude, 'q'):
            roll, pitch, yaw = self.quaternion_to_euler(
                self.vehicle_attitude.q[0],
                self.vehicle_attitude.q[1],
                self.vehicle_attitude.q[2],
                self.vehicle_attitude.q[3]
            )
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
        else:
            roll_deg = 0.0
            pitch_deg = 0.0

        if self.ground_height_offset is None:
            return

        relative_height_cm = (self.vehicle_local_position.z - self.ground_height_offset) * 100

        if relative_height_cm < 0:
            relative_height_cm = 0

        is_offboard = hasattr(self.vehicle_control_mode, 'flag_control_offboard_enabled') and \
                     self.vehicle_control_mode.flag_control_offboard_enabled

        if is_offboard:
            if self.offboard_entry_time is None:
                self.offboard_entry_time = self.get_clock().now()
                self.get_logger().info("已进入OFFBOARD模式，开始2秒倒计时...")
            else:
                elapsed_time = (self.get_clock().now() - self.offboard_entry_time).nanoseconds / 1e9
                
                if elapsed_time >= 2.0 and not self.control_enabled:
                    self.control_enabled = True
                    self.get_logger().info("OFFBOARD模式已维持2秒，启用速度控制")
        else:
            if self.offboard_entry_time is not None:
                self.offboard_entry_time = None
                self.control_enabled = False
                self.get_logger().info("已退出OFFBOARD模式，禁用速度控制")

        if self.aruco_detected:
            state['target_loss'] = False
            state['target_loss_hold_time'] = 0
        else:
            if state['target_loss_hold_time'] < 1000:
                state['target_loss_hold_time'] += dt * 1000
            else:
                state['target_loss'] = True

        opmv_pos_x = self.aruco_position.x
        opmv_pos_y = self.aruco_position.y

        if self.aruco_detected:
            rp2pixel_val_x = -2.4 * pitch_deg
            rp2pixel_val_y = -2.4 * roll_deg
            rp2pixel_val_x = self.limit(rp2pixel_val_x, -60, 60)
            rp2pixel_val_y = self.limit(rp2pixel_val_y, -80, 80)

            ref_carrier_velocity_x = self.vehicle_local_position.vx * 100
            ref_carrier_velocity_y = self.vehicle_local_position.vy * 100
        else:
            ref_carrier_velocity_x = 0.0
            ref_carrier_velocity_y = 0.0

        if not state['target_loss']:
            decou_pos_x = opmv_pos_y - rp2pixel_val_x
            decou_pos_y = -opmv_pos_x - rp2pixel_val_y

            state['decou_pos_pixel_lpf'][0][0] = self.lpf_1(0.2, dt, decou_pos_x, state['decou_pos_pixel_lpf'][0][0])
            state['decou_pos_pixel_lpf'][0][1] = self.lpf_1(0.2, dt, decou_pos_y, state['decou_pos_pixel_lpf'][0][1])

            state['decou_pos_pixel_lpf'][1][0] = self.lpf_1(0.2, dt, state['decou_pos_pixel_lpf'][0][0], state['decou_pos_pixel_lpf'][1][0])
            state['decou_pos_pixel_lpf'][1][1] = self.lpf_1(0.2, dt, state['decou_pos_pixel_lpf'][0][1], state['decou_pos_pixel_lpf'][1][1])

            decou_pos_pixel_x = state['decou_pos_pixel_lpf'][1][0]
            decou_pos_pixel_y = state['decou_pos_pixel_lpf'][1][1]
        else:
            state['decou_pos_pixel_lpf'][1][0] = self.lpf_1(0.2, dt, 0.0, state['decou_pos_pixel_lpf'][1][0])
            state['decou_pos_pixel_lpf'][1][1] = self.lpf_1(0.2, dt, 0.0, state['decou_pos_pixel_lpf'][1][1])

            decou_pos_pixel_x = state['decou_pos_pixel_lpf'][1][0]
            decou_pos_pixel_y = state['decou_pos_pixel_lpf'][1][1]

        ground_pos_err_x = 0.01 * relative_height_cm * decou_pos_pixel_x
        ground_pos_err_y = 0.01 * relative_height_cm * decou_pos_pixel_y

        gped_tmp_x = (ground_pos_err_x - state['ground_pos_err_old'][0]) * (1000 / (dt * 1000))
        gped_tmp_y = (ground_pos_err_y - state['ground_pos_err_old'][1]) * (1000 / (dt * 1000))

        state['ground_pos_err_d'][0] = self.lpf_1(0.2, dt, gped_tmp_x, state['ground_pos_err_d'][0])
        state['ground_pos_err_d'][1] = self.lpf_1(0.2, dt, gped_tmp_y, state['ground_pos_err_d'][1])

        state['target_gnd_velocity'][0] = state['ground_pos_err_d'][0] + ref_carrier_velocity_x
        state['target_gnd_velocity'][1] = state['ground_pos_err_d'][1] + ref_carrier_velocity_y

        CBT_KF = 0.2
        CBT_KP = 0.8
        CBT_KD = 0.2

        state['exp_velocity'][0] = CBT_KF * state['target_gnd_velocity'][0] + CBT_KP * ground_pos_err_x + CBT_KD * state['ground_pos_err_d'][0]
        state['exp_velocity'][1] = CBT_KF * state['target_gnd_velocity'][1] + CBT_KP * ground_pos_err_y + CBT_KD * state['ground_pos_err_d'][1]

        if self.control_enabled and self.enable_control:
            if state['target_loss']:
                velocity_msg = Vector3()
                velocity_msg.x = 0.0
                velocity_msg.y = 0.0
                velocity_msg.z = 0.0
                self.velocity_command_publisher.publish(velocity_msg)
                
                mode_msg = Bool()
                mode_msg.data = True
                self.velocity_mode_publisher.publish(mode_msg)
            else:
                velocity_msg = Vector3()
                velocity_msg.x = state['exp_velocity'][0] / 100
                velocity_msg.y = state['exp_velocity'][1] / 100
                velocity_msg.z = 0.0
                self.velocity_command_publisher.publish(velocity_msg)

                mode_msg = Bool()
                mode_msg.data = True
                self.velocity_mode_publisher.publish(mode_msg)
        else:
            if self.control_enabled and not self.enable_control:
                self.get_logger().info(f"期望速度: VX={state['exp_velocity'][0]/100:.3f}, VY={state['exp_velocity'][1]/100:.3f} m/s (测试模式，未发布)")
            
            mode_msg = Bool()
            mode_msg.data = False
            self.velocity_mode_publisher.publish(mode_msg)

        state['ground_pos_err_old'][0] = ground_pos_err_x
        state['ground_pos_err_old'][1] = ground_pos_err_y

    def destroy_node(self):
        mode_msg = Bool()
        mode_msg.data = False
        self.velocity_mode_publisher.publish(mode_msg)
        super().destroy_node()


def main(args=None) -> None:
    print('启动ArUco追踪节点（基于匿名科创CBTracking控制逻辑）...')
    rclpy.init(args=args)
    aruco_cbtracking = ArucoCBTracking()
    rclpy.spin(aruco_cbtracking)
    aruco_cbtracking.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
