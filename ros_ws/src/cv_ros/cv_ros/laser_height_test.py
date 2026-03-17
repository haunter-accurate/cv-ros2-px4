#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
import time


class LaserHeightTest(Node):
    """激光高度测试节点，实时获取并显示激光高度"""

    def __init__(self) -> None:
        super().__init__('laser_height_test')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, qos_profile)

        self.odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, qos_profile)

        self.current_height_local = 0.0
        self.current_height_odom = 0.0
        self.last_height_local = 0.0
        self.last_height_odom = 0.0
        self.height_received_local = False
        self.height_received_odom = False
        self.msg_count_local = 0
        self.msg_count_odom = 0

        self.get_logger().info("激光高度测试节点已启动")
        self.get_logger().info("订阅话题: /fmu/out/vehicle_local_position")
        self.get_logger().info("订阅话题: /fmu/out/vehicle_odometry")
        self.get_logger().info("QoS设置: BEST_EFFORT, VOLATILE, KEEP_LAST(10)")

        self.timer = self.create_timer(1.0, self.timer_callback)

    def local_position_callback(self, msg):
        """本地位置回调函数"""
        self.current_height_local = msg.z
        self.height_received_local = True
        self.msg_count_local += 1
        
        if self.msg_count_local == 1:
            self.get_logger().info(f"首次收到本地位置数据")
            self.get_logger().info(f"位置: X={msg.x:.3f}m, Y={msg.y:.3f}m, Z={msg.z:.3f}m")
            self.get_logger().info(f"速度: VX={msg.vx:.3f}m/s, VY={msg.vy:.3f}m/s, VZ={msg.vz:.3f}m/s")
            self.get_logger().info(f"状态: xy_valid={msg.xy_valid}, z_valid={msg.z_valid}, v_xy_valid={msg.v_xy_valid}, v_z_valid={msg.v_z_valid}")

    def odometry_callback(self, msg):
        """里程计回调函数"""
        self.current_height_odom = msg.position[2]
        self.height_received_odom = True
        self.msg_count_odom += 1
        
        if self.msg_count_odom == 1:
            pose_frame_str = "NED" if msg.pose_frame == VehicleOdometry.POSE_FRAME_NED else "FRD" if msg.pose_frame == VehicleOdometry.POSE_FRAME_FRD else "UNKNOWN"
            velocity_frame_str = "NED" if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED else "FRD" if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_FRD else "BODY_FRD" if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD else "UNKNOWN"
            
            self.get_logger().info(f"首次收到里程计数据")
            self.get_logger().info(f"位置框架: {pose_frame_str}, 速度框架: {velocity_frame_str}")
            self.get_logger().info(f"位置: X={msg.position[0]:.3f}m, Y={msg.position[1]:.3f}m, Z={msg.position[2]:.3f}m")
            self.get_logger().info(f"速度: VX={msg.velocity[0]:.3f}m/s, VY={msg.velocity[1]:.3f}m/s, VZ={msg.velocity[2]:.3f}m/s")
            self.get_logger().info(f"姿态: qw={msg.q[0]:.3f}, qx={msg.q[1]:.3f}, qy={msg.q[2]:.3f}, qz={msg.q[3]:.3f}")

    def timer_callback(self):
        """定时器回调，每秒输出一次高度信息"""
        if self.height_received_local or self.height_received_odom:
            local_changed = self.current_height_local != self.last_height_local
            odom_changed = self.current_height_odom != self.last_height_odom
            
            if local_changed or odom_changed:
                local_str = f"{self.current_height_local:.3f}m" if self.height_received_local else "N/A"
                odom_str = f"{self.current_height_odom:.3f}m" if self.height_received_odom else "N/A"
                diff_str = f"{abs(self.current_height_local - self.current_height_odom):.3f}m" if (self.height_received_local and self.height_received_odom) else "N/A"
                
                self.get_logger().info(f"高度对比 - LocalPosition(Z): {local_str}, Odometry(Z): {odom_str}, 差值: {diff_str}")
            else:
                if int(time.time()) % 10 == 0:
                    local_str = f"{self.current_height_local:.3f}m" if self.height_received_local else "N/A"
                    odom_str = f"{self.current_height_odom:.3f}m" if self.height_received_odom else "N/A"
                    diff_str = f"{abs(self.current_height_local - self.current_height_odom):.3f}m" if (self.height_received_local and self.height_received_odom) else "N/A"
                    
                    self.get_logger().info(f"高度对比(稳定) - LocalPosition(Z): {local_str}, Odometry(Z): {odom_str}, 差值: {diff_str}")
            
            self.last_height_local = self.current_height_local
            self.last_height_odom = self.current_height_odom
        else:
            self.get_logger().warn(f"未收到任何高度数据，请检查:")
            self.get_logger().warn("1. PX4是否正常运行")
            self.get_logger().warn("2. 话题名称是否正确")
            self.get_logger().warn("3. 使用 'ros2 topic list' 查看可用话题")
            self.get_logger().warn("4. 使用 'ros2 topic echo /fmu/out/vehicle_local_position' 查看话题内容")
            self.get_logger().warn("5. 使用 'ros2 topic echo /fmu/out/vehicle_odometry' 查看话题内容")


def main(args=None) -> None:
    print('启动激光高度测试节点...')
    rclpy.init(args=args)
    laser_height_test = LaserHeightTest()
    rclpy.spin(laser_height_test)
    laser_height_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
