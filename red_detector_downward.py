#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class RedDetectorDownward(Node):
    def __init__(self):
        super().__init__('red_detector_downward')
        
        # 参数配置
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('min_area', 300)
        
        # HSV颜色范围（红色）- 需要根据实际环境调整
        self.declare_parameter('lower_red1_h', 0)
        self.declare_parameter('lower_red1_s', 120)
        self.declare_parameter('lower_red1_v', 70)
        self.declare_parameter('upper_red1_h', 10)
        self.declare_parameter('upper_red1_s', 255)
        self.declare_parameter('upper_red1_v', 255)
        
        self.declare_parameter('lower_red2_h', 170)
        self.declare_parameter('lower_red2_s', 120)
        self.declare_parameter('lower_red2_v', 70)
        self.declare_parameter('upper_red2_h', 180)
        self.declare_parameter('upper_red2_s', 255)
        self.declare_parameter('upper_red2_v', 255)
        
        # 相机安装参数（摄像头朝下）
        self.declare_parameter('camera_angle', 0.0)  # 相机倾斜角度（度），0表示垂直向下
        self.declare_parameter('camera_height_m', 5.0)  # 相机离地面高度（米）
        
        # 初始化参数
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.min_area = self.get_parameter('min_area').value
        self.camera_angle = np.radians(self.get_parameter('camera_angle').value)
        self.camera_height_m = self.get_parameter('camera_height_m').value
        
        # HSV颜色范围
        self.lower_red1 = np.array([
            self.get_parameter('lower_red1_h').value,
            self.get_parameter('lower_red1_s').value,
            self.get_parameter('lower_red1_v').value
        ])
        self.upper_red1 = np.array([
            self.get_parameter('upper_red1_h').value,
            self.get_parameter('upper_red1_s').value,
            self.get_parameter('upper_red1_v').value
        ])
        
        self.lower_red2 = np.array([
            self.get_parameter('lower_red2_h').value,
            self.get_parameter('lower_red2_s').value,
            self.get_parameter('lower_red2_v').value
        ])
        self.upper_red2 = np.array([
            self.get_parameter('upper_red2_h').value,
            self.get_parameter('upper_red2_s').value,
            self.get_parameter('upper_red2_v').value
        ])
        
        # 相机内参（需要根据实际相机标定）！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        self.fov_h = 60  # 水平视场角（度）
        self.fov_v = 45  # 垂直视场角（度）
        self.fx = self.camera_width / (2 * np.tan(np.radians(self.fov_h / 2)))
        self.fy = self.camera_height / (2 * np.tan(np.radians(self.fov_v / 2)))
        self.cx = self.camera_width / 2
        self.cy = self.camera_height / 2
        
        # 创建发布者和订阅者
        camera_topic = self.get_parameter('camera_topic').value
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            qos_profile_sensor_data
        )
        
        # 发布检测结果
        # 发布图像坐标系中的位置（归一化）
        self.target_norm_pub = self.create_publisher(Point, '/detection/target_normalized', 10)
        # 发布机体坐标系中的位置（米）
        self.target_body_pub = self.create_publisher(Point, '/detection/target_body', 10)
        # 发布世界坐标系中的位置（需要GPS位置）
        self.target_world_pub = self.create_publisher(PointStamped, '/detection/target_world', 10)
        # 可视化图像
        self.visualization_pub = self.create_publisher(Image, '/detection/visualization', 10)
        
        self.bridge = CvBridge()
        
        # 当前无人机位置（由GPS提供）
        self.current_position = None
        self.current_yaw = 0.0
        
        # 订阅无人机位置
        self.position_sub = self.create_subscription(
            PointStamped,
            '/mavros/local_position/pose',  # 或使用GPS话题
            self.position_callback,
            10
        )
        
        self.get_logger().info('向下摄像头红色检测器已启动')

    def position_callback(self, msg):
        """更新无人机当前位置"""
        self.current_position = msg.point
        
        # 如果需要，也可以订阅姿态获取偏航角
        # self.current_yaw = ...

    def image_callback(self, msg):
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 处理图像
            result, target_info = self.process_image(cv_image)
            
            # 发布可视化结果
            vis_msg = self.bridge.cv2_to_imgmsg(result, 'bgr8')
            self.visualization_pub.publish(vis_msg)
            
            if target_info['detected']:
                # 发布归一化坐标
                norm_msg = Point()
                norm_msg.x = target_info['norm_x']
                norm_msg.y = target_info['norm_y']
                norm_msg.z = target_info['size']
                self.target_norm_pub.publish(norm_msg)
                
                # 计算并发布机体坐标系中的位置
                body_position = self.image_to_body_coords(
                    target_info['pixel_x'], 
                    target_info['pixel_y'],
                    self.camera_height_m
                )
                
                body_msg = Point()
                body_msg.x = body_position[0]
                body_msg.y = body_position[1]
                body_msg.z = 0.0  # 地面目标
                self.target_body_pub.publish(body_msg)
                
                # 如果已知无人机位置，计算世界坐标
                if self.current_position:
                    world_position = self.body_to_world_coords(
                        body_position[0], 
                        body_position[1],
                        self.current_position.x,
                        self.current_position.y,
                        self.current_yaw
                    )
                    
                    world_msg = PointStamped()
                    world_msg.header = Header()
                    world_msg.header.stamp = self.get_clock().now().to_msg()
                    world_msg.header.frame_id = 'map'
                    world_msg.point.x = world_position[0]
                    world_msg.point.y = world_position[1]
                    world_msg.point.z = 0.0  # 地面目标
                    self.target_world_pub.publish(world_msg)
                    
                    self.get_logger().info(
                        f'目标位置: 机体=({body_position[0]:.2f}, {body_position[1]:.2f}), '
                        f'世界=({world_position[0]:.2f}, {world_position[1]:.2f})',
                        throttle_duration_sec=0.5
                    )
            
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {e}')

    def process_image(self, cv_image):
        """处理图像，检测红色物体"""
        result = cv_image.copy()
        target_info = {
            'detected': False,
            'pixel_x': 0,
            'pixel_y': 0,
            'norm_x': 0,
            'norm_y': 0,
            'size': 0
        }
        
        # 转换到HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 创建红色掩码
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # 形态学操作
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # 寻找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # 找到最大轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > self.min_area:
                # 计算边界框和中心
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x = x + w // 2
                center_y = y + h // 2
                
                # 归一化坐标 [-1, 1]
                norm_x = (center_x / self.camera_width) * 2 - 1
                norm_y = (center_y / self.camera_height) * 2 - 1
                
                # 更新目标信息
                target_info.update({
                    'detected': True,
                    'pixel_x': center_x,
                    'pixel_y': center_y,
                    'norm_x': norm_x,
                    'norm_y': norm_y,
                    'size': np.sqrt(area)
                })
                
                # 绘制检测结果
                cv2.rectangle(result, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(result, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(result, f'Target', (x, y-10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # 绘制中心十字线
                cv2.line(result, (self.camera_width//2, 0), 
                        (self.camera_width//2, self.camera_height), (255, 255, 255), 1)
                cv2.line(result, (0, self.camera_height//2), 
                        (self.camera_width, self.camera_height//2), (255, 255, 255), 1)
        
        return result, target_info

    def image_to_body_coords(self, pixel_x, pixel_y, altitude):
        """
        将图像像素坐标转换为机体坐标系坐标（米）
        假设摄像头垂直向下，无倾斜
        """
        # 计算相对于图像中心的像素偏移
        dx_pixel = pixel_x - self.cx
        dy_pixel = pixel_y - self.cy
        
        # 转换为角度（弧度）
        theta_x = np.arctan2(dx_pixel, self.fx)
        theta_y = np.arctan2(dy_pixel, self.fy)
        
        # 转换为地面距离（米）
        # 对于垂直向下的摄像头：地面距离 = 高度 * tan(角度)
        ground_x = altitude * np.tan(theta_x)
        ground_y = altitude * np.tan(theta_y)
        
        # 注意：图像坐标系中y向下为正，机体坐标系中向前为正
        # 所以需要反转y轴
        body_x = ground_x  # 机体右侧为正
        body_y = -ground_y  # 机体前方为正
        
        return body_x, body_y

    def body_to_world_coords(self, body_x, body_y, drone_x, drone_y, yaw):
        """
        将机体坐标系坐标转换为世界坐标系坐标
        """
        # 旋转矩阵
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        # 坐标变换
        world_x = drone_x + body_x * cos_yaw - body_y * sin_yaw
        world_y = drone_y + body_x * sin_yaw + body_y * cos_yaw
        
        return world_x, world_y

def main(args=None):
    rclpy.init(args=args)
    detector = RedDetectorDownward()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()