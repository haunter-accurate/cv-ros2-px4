#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude


class ColorTrackingOffboard(Node):
    """在offboard模式下控制无人机跟踪彩色物体的节点。"""

    def __init__(self) -> None:
        super().__init__('color_tracking_offboard')

        # 配置发布和订阅的QoS配置文件
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 创建发布者
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # 创建订阅者
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        
        # 订阅红色物体检测话题
        self.color_detection_subscriber = self.create_subscription(
            Point, '/detection/red_center', self.color_detection_callback, qos_profile)

        # 初始化变量
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_attitude = VehicleAttitude()  # 无人机姿态信息
        self.color_center = Point()  # 最新检测到的颜色中心（像素坐标）
        
        # 图像参数（假设的摄像头参数）
        self.image_width = 640.0  # 像素
        self.image_height = 480.0  # 像素
        
        # 控制参数
        self.target_altitude = -5.0  # 目标高度（米，负数因为PX4使用NED坐标系）
        self.control_gain = 0.01  # 将像素误差转换为位置偏移的增益
        self.max_offset = 1.0  # 最大允许的位置偏移（米）
        
        # 解耦参数（参考匿名飞控）
        self.pixel_per_degree_x = 2.4  # 每度对应的像素数（X轴）
        self.pixel_per_degree_y = 2.4  # 每度对应的像素数（Y轴）
        self.cmp_pixel_x = 0.01  # 每像素对应的地面距离（米）
        self.cmp_pixel_y = 0.01  # 每像素对应的地面距离（米）
        self.lpf_alpha = 0.2  # 低通滤波系数
        
        # 解耦变量
        self.decou_pos_pixel = [0.0, 0.0]  # 解耦后的像素偏移
        self.decou_pos_pixel_lpf = [[0.0, 0.0], [0.0, 0.0]]  # 两级低通滤波
        self.target_loss = 0  # 目标丢失标志
        self.target_loss_hold_time = 0  # 目标丢失保持时间
        self.tlh_time = 100  # 目标丢失超时时间（10秒，因为定时器间隔0.1秒）
        
        # 创建定时器来发布控制命令
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """vehicle_local_position话题订阅者的回调函数。"""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """vehicle_status话题订阅者的回调函数。"""
        self.vehicle_status = vehicle_status
        
    def vehicle_attitude_callback(self, vehicle_attitude):
        """vehicle_attitude话题订阅者的回调函数。"""
        self.vehicle_attitude = vehicle_attitude
        
    def color_detection_callback(self, color_center):
        """颜色检测话题订阅者的回调函数。"""
        self.color_center = color_center
        self.get_logger().info(f"接收到颜色中心: ({color_center.x}, {color_center.y})")

    def arm(self):
        """向无人机发送解锁命令。"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('解锁命令已发送')

    def disarm(self):
        """向无人机发送锁定命令。"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('锁定命令已发送')

    def engage_offboard_mode(self):
        """切换到offboard模式。"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("正在切换到offboard模式")

    def land(self):
        """切换到降落模式。"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("正在切换到降落模式")

    def publish_offboard_control_heartbeat_signal(self):
        """发布offboard控制模式。"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """发布轨迹设定点。"""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90度)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"正在发布位置设定点 {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """发布无人机命令。"""
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

    def quaternion_to_euler(self, x, y, z, w):
        """将四元数转换为欧拉角（滚转、俯仰、偏航）"""
        import math
        
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
        
    def decouple_vision_data(self):
        """实现解耦算法，参考匿名飞控的ANO_CBTracking_Decoupling函数"""
        import math
        
        # 检查是否检测到目标
        if hasattr(self.color_center, 'x') and hasattr(self.color_center, 'y'):
            self.target_loss = 0
            self.target_loss_hold_time = 0
        else:
            # 目标丢失，增加保持时间
            if self.target_loss_hold_time < self.tlh_time:
                self.target_loss_hold_time += 1
            else:
                self.target_loss = 1
        
        # 获取图像像素误差
        error_x = self.color_center.x - (self.image_width / 2)
        error_y = self.color_center.y - (self.image_height / 2)
        
        # 摄像头朝向地面，方向对应关系
        # 图像X轴（水平）对应无人机Y轴（左右方向）
        # 图像Y轴（垂直）对应无人机X轴（前后方向）
        opmv_pos_x = -error_y  # 图像Y轴误差对应无人机X轴（需要反转）
        opmv_pos_y = error_x   # 图像X轴误差对应无人机Y轴
        
        # 获取无人机姿态（四元数转换为欧拉角）
        if hasattr(self.vehicle_attitude, 'q'):
            roll, pitch, yaw = self.quaternion_to_euler(
                self.vehicle_attitude.q[0], 
                self.vehicle_attitude.q[1], 
                self.vehicle_attitude.q[2], 
                self.vehicle_attitude.q[3]
            )
            
            # 将弧度转换为角度
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            
            # 计算姿态引起的像素偏移
            rp2pixel_val_x = -self.pixel_per_degree_x * pitch_deg
            rp2pixel_val_y = -self.pixel_per_degree_y * roll_deg
        else:
            rp2pixel_val_x = 0.0
            rp2pixel_val_y = 0.0
        
        # 应用姿态补偿
        corrected_pos_x = opmv_pos_x - rp2pixel_val_x
        corrected_pos_y = opmv_pos_y - rp2pixel_val_y
        
        if not self.target_loss:
            # 应用两级低通滤波
            self.decou_pos_pixel_lpf[0][0] += self.lpf_alpha * (corrected_pos_x - self.decou_pos_pixel_lpf[0][0])
            self.decou_pos_pixel_lpf[0][1] += self.lpf_alpha * (corrected_pos_y - self.decou_pos_pixel_lpf[0][1])
            
            self.decou_pos_pixel_lpf[1][0] += self.lpf_alpha * (self.decou_pos_pixel_lpf[0][0] - self.decou_pos_pixel_lpf[1][0])
            self.decou_pos_pixel_lpf[1][1] += self.lpf_alpha * (self.decou_pos_pixel_lpf[0][1] - self.decou_pos_pixel_lpf[1][1])
            
            self.decou_pos_pixel[0] = self.decou_pos_pixel_lpf[1][0]
            self.decou_pos_pixel[1] = self.decou_pos_pixel_lpf[1][1]
        else:
            # 目标丢失，应用低通滤波归零
            self.decou_pos_pixel[0] += self.lpf_alpha * (0.0 - self.decou_pos_pixel[0])
            self.decou_pos_pixel[1] += self.lpf_alpha * (0.0 - self.decou_pos_pixel[1])
        
    def calculate_position_offset(self):
        """根据解耦后的像素偏移计算位置偏移。"""
        # 先执行解耦算法
        self.decouple_vision_data()
        
        # 获取当前高度（绝对值，米）
        current_height = abs(self.vehicle_local_position.z) if hasattr(self.vehicle_local_position, 'z') else 1.0
        
        # 根据高度调整每像素对应的地面距离
        cmp_pixel_x = self.cmp_pixel_x * current_height
        cmp_pixel_y = self.cmp_pixel_y * current_height
        
        # 将解耦后的像素偏移转换为位置偏移
        offset_x = self.decou_pos_pixel[0] * cmp_pixel_x
        offset_y = self.decou_pos_pixel[1] * cmp_pixel_y
        
        # 将偏移限制在最大允许值内
        offset_x = max(min(offset_x, self.max_offset), -self.max_offset)
        offset_y = max(min(offset_y, self.max_offset), -self.max_offset)
        
        return offset_x, offset_y

    def timer_callback(self) -> None:
        """定时器的回调函数。"""
        # 发布offboard控制模式心跳信号（无论是否在OFFBOARD模式）
        self.publish_offboard_control_heartbeat_signal()

        # 记录当前导航状态和高度
        if self.offboard_setpoint_counter % 10 == 0:
            nav_state = self.vehicle_status.nav_state if hasattr(self.vehicle_status, 'nav_state') else "未知"
            altitude = self.vehicle_local_position.z if hasattr(self.vehicle_local_position, 'z') else "未知"
            self.get_logger().info(f"导航状态: {nav_state}, 高度: {altitude}")

        # 检查是否处于OFFBOARD模式
        is_offboard = hasattr(self.vehicle_status, 'nav_state') and \
                     self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        
        # 检查是否已经起飞（高度低于-0.5米，因为PX4使用NED坐标系）
        is_flying = self.vehicle_local_position.z < -0.5
        
        # 只有在OFFBOARD模式且已经起飞的情况下才发送目标位置
        if is_offboard and is_flying:
            # 计算位置偏移
            offset_x, offset_y = self.calculate_position_offset()
            
            # 限制速度（通过限制每次位置更新的最大偏移量实现）
            max_speed = 0.5  # 最大速度（米/秒）
            update_interval = 0.1  # 定时器间隔（秒）
            max_offset_per_update = max_speed * update_interval  # 每次更新的最大偏移量
            
            offset_x = max(min(offset_x, max_offset_per_update), -max_offset_per_update)
            offset_y = max(min(offset_y, max_offset_per_update), -max_offset_per_update)
            
            # 计算目标位置
            target_x = self.vehicle_local_position.x + offset_x
            target_y = self.vehicle_local_position.y + offset_y
            target_z = self.target_altitude
            
            # 发布目标位置
            self.publish_position_setpoint(target_x, target_y, target_z)
        elif is_offboard:
            self.get_logger().info("处于OFFBOARD模式，但飞机尚未起飞")
        else:
            self.get_logger().info("未处于OFFBOARD模式，等待遥控器切换")

        # 更新计数器
        if self.offboard_setpoint_counter < 100:
            self.offboard_setpoint_counter += 1

    def destroy_node(self):
        """节点销毁时的清理工作。"""
        # 不再自动锁定，由用户通过遥控器控制
        super().destroy_node()


def main(args=None) -> None:
    print('启动颜色跟踪offboard控制节点...')
    rclpy.init(args=args)
    color_tracking_offboard = ColorTrackingOffboard()
    rclpy.spin(color_tracking_offboard)
    color_tracking_offboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
