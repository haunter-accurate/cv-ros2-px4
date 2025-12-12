#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from std_msgs.msg import Header
import time
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class GPSPositionController(Node):
    def __init__(self):
        super().__init__('gps_position_controller')
        
        # 参数配置
        self.declare_parameter('operation_mode', 'track')  # 'track'或'hold'
        self.declare_parameter('target_altitude', 5.0)  # 目标高度（米）
        self.declare_parameter('approach_speed', 0.5)  # 接近速度（米/秒）
        self.declare_parameter('position_tolerance', 0.5)  # 位置容差（米）
        self.declare_parameter('max_distance', 10.0)  # 最大跟踪距离（米）
        self.declare_parameter('hold_radius', 2.0)  # 悬停半径（米）
        
        # 状态变量
        self.current_state = State()
        self.current_position = None
        self.current_pose = None
        self.target_position_world = None
        self.last_target_time = 0
        self.target_lost = True
        
        # 订阅器
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        # 订阅无人机当前位置
        self.position_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.position_callback,
            10
        )
        
        # 订阅世界坐标系中的目标位置
        self.target_sub = self.create_subscription(
            PointStamped,
            '/detection/target_world',
            self.target_callback,
            10
        )
        
        # 发布器 - 发布目标位置给PX4
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        # 服务客户端
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # 控制定时器
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        # 状态变量
        self.offboard_setpoint_count = 0
        self.current_target = None
        self.target_history = []
        self.max_history = 10
        
        # 操作模式
        self.operation_mode = self.get_parameter('operation_mode').value
        
        self.get_logger().info(f'GPS位置控制器已启动，模式: {self.operation_mode}')

    def state_callback(self, msg):
        self.current_state = msg

    def position_callback(self, msg):
        self.current_pose = msg.pose
        self.current_position = msg.pose.position

    def target_callback(self, msg):
        self.target_position_world = msg.point
        self.last_target_time = time.time()
        self.target_lost = False
        
        # 添加到历史记录
        self.target_history.append((self.target_position_world.x, self.target_position_world.y))
        if len(self.target_history) > self.max_history:
            self.target_history.pop(0)

    def control_loop(self):
        # 确保已经连接到飞控
        if not self.current_state.connected:
            self.get_logger().warn('未连接到飞控')
            return
        
        # 初始阶段：发布设定点以进入Offboard模式
        if self.offboard_setpoint_count <= 50:
            self.publish_hover_setpoint()
            self.offboard_setpoint_count += 1
            
            if self.offboard_setpoint_count == 30:
                self.set_mode("OFFBOARD")
                self.arm_drone()
            return
        
        # 检查目标是否丢失
        current_time = time.time()
        if current_time - self.last_target_time > 2.0:  # 2秒未检测到目标
            self.target_lost = True
        
        # 根据操作模式执行相应控制
        if self.operation_mode == 'track':
            self.track_mode()
        else:
            self.hold_mode()

    def track_mode(self):
        """跟踪模式：飞向目标位置"""
        if self.target_lost or self.target_position_world is None:
            # 目标丢失，悬停
            self.hover()
            self.get_logger().warn('目标丢失，悬停中...', throttle_duration_sec=1.0)
            return
        
        if self.current_position is None:
            self.get_logger().warn('未获取到当前位置')
            return
        
        # 计算到目标的距离
        target_x = self.target_position_world.x
        target_y = self.target_position_world.y
        current_x = self.current_position.x
        current_y = self.current_position.y
        
        distance = np.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        # 检查是否在最大距离内
        max_distance = self.get_parameter('max_distance').value
        if distance > max_distance:
            self.get_logger().warn(f'目标距离({distance:.1f}m)超过最大跟踪距离({max_distance}m)')
            self.hover()
            return
        
        # 检查是否已达到目标
        tolerance = self.get_parameter('position_tolerance').value
        if distance < tolerance:
            # 在目标上方悬停
            self.publish_position_setpoint(target_x, target_y, self.current_position.z)
            self.get_logger().info('已在目标上方悬停', throttle_duration_sec=1.0)
            return
        
        # 计算目标位置（保持当前高度）
        target_z = self.current_position.z
        
        # 使用历史位置进行平滑
        if len(self.target_history) > 1:
            # 计算历史位置的平均值
            avg_x = np.mean([p[0] for p in self.target_history])
            avg_y = np.mean([p[1] for p in self.target_history])
            
            # 添加一些预测（简单线性预测）
            if len(self.target_history) >= 3:
                dx = avg_x - current_x
                dy = avg_y - current_y
                
                # 计算目标速度（简化）
                if len(self.target_history) >= 4:
                    vx = (self.target_history[-1][0] - self.target_history[-4][0]) / 0.3
                    vy = (self.target_history[-1][1] - self.target_history[-4][1]) / 0.3
                    
                    # 预测未来位置（0.5秒后）
                    predict_time = 0.5
                    predict_x = target_x + vx * predict_time
                    predict_y = target_y + vy * predict_time
                    
                    # 混合预测和当前位置
                    alpha = 0.3  # 预测权重
                    target_x = target_x * (1-alpha) + predict_x * alpha
                    target_y = target_y * (1-alpha) + predict_y * alpha
        
        # 发布目标位置
        self.publish_position_setpoint(target_x, target_y, target_z)
        
        self.get_logger().info(
            f'跟踪目标: 距离={distance:.1f}m, 目标位置=({target_x:.1f}, {target_y:.1f})',
            throttle_duration_sec=0.5
        )

    def hold_mode(self):
        """保持模式：在目标周围悬停"""
        if self.target_lost or self.target_position_world is None:
            self.hover()
            return
        
        if self.current_position is None:
            return
        
        # 计算到目标的距离
        target_x = self.target_position_world.x
        target_y = self.target_position_world.y
        current_x = self.current_position.x
        current_y = self.current_position.y
        
        distance = np.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        hold_radius = self.get_parameter('hold_radius').value
        
        if distance > hold_radius:
            # 飞向保持点
            # 计算从当前位置到目标方向上的保持点
            direction_x = target_x - current_x
            direction_y = target_y - current_y
            direction_len = np.sqrt(direction_x**2 + direction_y**2)
            
            if direction_len > 0:
                direction_x /= direction_len
                direction_y /= direction_len
                
                # 保持点位置
                hold_x = target_x - direction_x * hold_radius
                hold_y = target_y - direction_y * hold_radius
                
                self.publish_position_setpoint(hold_x, hold_y, self.current_position.z)
                
                self.get_logger().info(
                    f'飞向保持点: 距离={distance:.1f}m',
                    throttle_duration_sec=1.0
                )
        else:
            # 已在保持半径内，悬停
            self.hover()
            self.get_logger().info(
                f'已在保持半径内: 距离={distance:.1f}m',
                throttle_duration_sec=2.0
            )

    def hover(self):
        """悬停在当前位置"""
        if self.current_position is None:
            return
        
        self.publish_position_setpoint(
            self.current_position.x,
            self.current_position.y,
            self.current_position.z
        )

    def publish_hover_setpoint(self):
        """发布悬停设定点"""
        if self.current_position is not None:
            target_z = self.current_position.z
        else:
            target_z = self.get_parameter('target_altitude').value
        
        self.publish_position_setpoint(0, 0, target_z)

    def publish_position_setpoint(self, x, y, z):
        """发布位置设定点"""
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        
        # 保持当前朝向（或朝向目标）
        if self.current_pose:
            pose_msg.pose.orientation = self.current_pose.orientation
        else:
            pose_msg.pose.orientation.w = 1.0
        
        self.setpoint_pub.publish(pose_msg)

    def set_mode(self, mode):
        """设置飞行模式"""
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待set_mode服务...')
        
        req = SetMode.Request()
        req.custom_mode = mode
        
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.set_mode_callback)

    def set_mode_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f'成功设置模式为: OFFBOARD')
        except Exception as e:
            self.get_logger().error(f'设置模式失败: {e}')

    def arm_drone(self):
        """解锁无人机"""
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待arming服务...')
        
        req = CommandBool.Request()
        req.value = True
        
        future = self.arming_client.call_async(req)
        future.add_done_callback(self.arm_callback)

    def arm_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('无人机已解锁')
        except Exception as e:
            self.get_logger().error(f'解锁失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    controller = GPSPositionController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()