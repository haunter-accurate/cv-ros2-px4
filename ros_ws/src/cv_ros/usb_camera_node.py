#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class USBCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        
        # 参数配置
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        
        # 初始化相机
        camera_id = self.get_parameter('camera_id').value
        self.cap = cv2.VideoCapture(camera_id)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'无法打开相机 {camera_id}')
            return
        
        # 设置相机参数
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        # 发布者
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # 定时器
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'USB相机节点已启动 (ID: {camera_id}, {width}x{height} @ {fps}fps)')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # 可选：旋转图像（如果相机安装方向需要）
            # frame = cv2.rotate(frame, cv2.ROTATE_180)  # 旋转180度
            
            # 发布图像
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            self.image_pub.publish(msg)
        else:
            self.get_logger().warn('无法从相机读取帧')

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = USBCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()