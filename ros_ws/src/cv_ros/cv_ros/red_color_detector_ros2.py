#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class RedColorDetectorROS2(Node):
    def __init__(self):
        super().__init__('red_color_detector_ros2')
        
        # 参数配置
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('publish_rate', 10.0)  # 最大发布频率（Hz）
        self.declare_parameter('min_area', 500)  # 最小色块面积阈值
        
        # 获取参数
        camera_id = self.get_parameter('camera_id').value
        publish_rate = self.get_parameter('publish_rate').value
        self.min_area = self.get_parameter('min_area').value
        
        # 打开摄像头
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'无法打开摄像头 {camera_id}')
            return
        
        # 读取摄像头参数
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # 创建发布者
        self.center_pub = self.create_publisher(Point, '/detection/red_center', 10)
        
        # 创建定时器（控制发布频率）
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 初始化中心点坐标
        self.last_center = None
        
        self.get_logger().info(f'红色色块检测ROS2节点已启动 (发布频率: {publish_rate}Hz)')
        self.get_logger().info('按 "q" 键退出程序')

    def timer_callback(self):
        # 读取一帧视频
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('无法获取视频帧')
            return
        
        # 将BGR转换为HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 定义红色的HSV范围（考虑红色在HSV中的两个区间）
        # 低红色范围
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        # 高红色范围
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        
        # 创建红色掩码
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # 对掩码进行形态学操作，去除噪声
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 初始化最大面积和最大轮廓
        max_area = 0
        max_contour = None
        
        # 遍历所有轮廓，找出最大的那个
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour
        
        # 如果找到最大轮廓且面积大于阈值，则标记它
        current_center = None
        if max_contour is not None and max_area > self.min_area:
            # 计算最小外接矩形
            x, y, w, h = cv2.boundingRect(max_contour)
            # 计算中心点
            center_x = x + w // 2
            center_y = y + h // 2
            current_center = (center_x, center_y)
            
            # 创建YOLOv5风格的叠加效果
            # 1. 绘制带透明度的边界框
            overlay = frame.copy()
            cv2.rectangle(overlay, (x, y), (x+w, y+h), (0, 255, 0), 2)  # 绿色边框
            
            # 2. 计算标签背景高度
            label = f"Red: {int(max_area)} area"
            label_height = 25
            
            # 3. 绘制标签背景（带透明度）
            cv2.rectangle(overlay, (x, y - label_height), (x + len(label) * 10 + 10, y), (0, 255, 0), -1)  # 绿色背景
            
            # 4. 合并叠加层与原图像
            frame = cv2.addWeighted(overlay, 0.8, frame, 0.2, 0)
            
            # 5. 绘制标签文本
            cv2.putText(frame, label, (x + 5, y - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)  # 黑色文本
            
            # 6. 绘制中心点（红色圆点）
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
        
        # 在画面中心添加十字分割线
        frame_center_x = self.frame_width // 2
        frame_center_y = self.frame_height // 2
        
        # 绘制水平和垂直分割线
        cv2.line(frame, (0, frame_center_y), (self.frame_width, frame_center_y), (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(frame, (frame_center_x, 0), (frame_center_x, self.frame_height), (255, 255, 255), 1, cv2.LINE_AA)
        
        # 显示结果
        cv2.imshow('Red Color Detection', frame)
        #cv2.imshow('Mask', mask)
        
        # 发布中心点坐标
        if current_center is not None:
            center_msg = Point()
            center_msg.x = float(current_center[0])
            center_msg.y = float(current_center[1])
            center_msg.z = 0.0  # 2D坐标，z设为0
            self.center_pub.publish(center_msg)
            
            # 更新最后检测到的中心点
            self.last_center = current_center
            
            # 在控制台输出中心点坐标
            self.get_logger().info(f'最大红色色块中心点坐标: ({current_center[0]}, {current_center[1]})', throttle_duration_sec=1.0)
        
        # 检查退出键
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

    def __del__(self):
        # 释放摄像头和关闭窗口
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    detector = RedColorDetectorROS2()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()