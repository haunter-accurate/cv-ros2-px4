#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Float32, Bool
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
import cv2
import cv2.aruco as aruco
import numpy as np
import math


class ArucoDetectorROS2(Node):
    """ArUco码检测ROS2节点，发布解耦后的坐标和朝向信息"""

    def __init__(self):
        super().__init__('aruco_detector_ros2')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.aruco_position_publisher = self.create_publisher(
            Point, '/detection/aruco_position', qos_profile)
        self.aruco_x_axis_yaw_publisher = self.create_publisher(
            Float32, '/detection/aruco_x_axis_yaw', qos_profile)
        self.aruco_detected_publisher = self.create_publisher(
            Bool, '/detection/aruco_detected', qos_profile)

        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', 
            self.vehicle_attitude_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)

        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_local_position = VehicleLocalPosition()

        self.script_dir = os.path.dirname(os.path.abspath(__file__))

        self.aruco_length = 0.05
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.parameters = aruco.DetectorParameters()

        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.adaptiveThreshConstant = 7
        self.parameters.minMarkerPerimeterRate = 0.02
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.parameters.polygonalApproxAccuracyRate = 0.05
        self.parameters.minCornerDistanceRate = 0.05
        self.parameters.minDistanceToBorder = 2
        self.parameters.minMarkerDistanceRate = 0.05
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 3
        self.parameters.cornerRefinementMaxIterations = 20
        self.parameters.cornerRefinementMinAccuracy = 0.1
        self.parameters.markerBorderBits = 1
        self.parameters.perspectiveRemovePixelPerCell = 4
        self.parameters.perspectiveRemoveIgnoredMarginPerCell = 0.13
        self.parameters.maxErroneousBitsInBorderRate = 0.4
        self.parameters.minOtsuStdDev = 3.0
        self.parameters.errorCorrectionRate = 0.6

        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None
        self.mapx = None
        self.mapy = None

        self._init_camera_parameters()
        self._init_camera()

        self.declare_parameter('target_aruco_id', 0)
        self.target_aruco_id = self.get_parameter('target_aruco_id').value

        self.pixel_per_degree_x = 2.4
        self.pixel_per_degree_y = 2.4
        self.cmp_pixel_x = 0.01
        self.cmp_pixel_y = 0.01
        self.lpf_alpha = 0.2

        self.decou_pos_pixel = [0.0, 0.0]
        self.decou_pos_pixel_lpf = [[0.0, 0.0], [0.0, 0.0]]
        self.target_loss = 0
        self.target_loss_hold_time = 0
        self.tlh_time = 100

        self.output_lpf_alpha = 0.3
        self.last_position = [0.0, 0.0, 0.0]
        self.last_yaw = 0.0
        self.filtered_position = [0.0, 0.0, 0.0]
        self.filtered_yaw = 0.0
        self.last_valid_data = None
        self.consecutive_loss_count = 0
        self.max_consecutive_loss = 2

        self.last_log_time = None
        self.log_interval = 1.0

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(f"目标ArUco ID: {self.target_aruco_id}")

    def _init_camera_parameters(self):
        camera_calib_path = os.path.join(self.script_dir, 'camera_calibration_data.npz')
        distance_calib_path = os.path.join(self.script_dir, 'distance_calibration_data.npz')
        
        try:
            with np.load(camera_calib_path) as data:
                self.camera_matrix = data['camera_matrix']
                self.dist_coeffs = data['dist_coeffs']
            self.get_logger().info("成功加载自定义相机标定参数")
        except FileNotFoundError:
            self.get_logger().info("未找到自定义相机标定参数，使用默认参数")
            fx = 406.932130
            fy = 402.678201
            cx = 316.629381
            cy = 242.533947
            k1 = 0.039106
            k2 = -0.056494
            p1 = -0.000824
            p2 = 0.092161
            k3 = 0.0
            self.camera_matrix = np.array([[fx, 0, cx],
                                           [0, fy, cy],
                                           [0, 0, 1]], dtype=np.float64)
            self.dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

        self.distance_calibration = None
        try:
            with np.load(distance_calib_path) as data:
                a = data['a']
                b = data['b']
                self.distance_calibration = (a, b)
            self.get_logger().info(f"成功加载距离标定参数：a={a:.6f}, b={b:.6f}")
        except FileNotFoundError:
            self.get_logger().info("未找到距离标定参数，使用原始计算值")

    def _init_camera(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开相机")
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("无法读取帧")
            return

        h, w = frame.shape[:2]
        self.image_width = w
        self.image_height = h

        self.new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(
            self.camera_matrix, self.dist_coeffs, None, 
            self.new_camera_matrix, (w, h), 5)

        self.get_logger().info(f"相机已初始化，分辨率: {w}x{h}")

    def vehicle_attitude_callback(self, vehicle_attitude):
        self.vehicle_attitude = vehicle_attitude

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position

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

    def rotation_matrix_to_euler(self, R):
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return x, y, z

    def decouple_aruco_position(self, tvec, rvec):
        """
        解耦ArUco码位置，消除无人机姿态和相对距离的影响
        参考Anonymous飞控的CBTracking解耦算法
        """
        aruco_x = tvec.flatten()[0]
        aruco_y = tvec.flatten()[1]
        aruco_z = tvec.flatten()[2]

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

        rp_offset_x = -self.pixel_per_degree_x * pitch_deg * (aruco_z / 1000.0)
        rp_offset_y = -self.pixel_per_degree_y * roll_deg * (aruco_z / 1000.0)

        corrected_x = aruco_x - rp_offset_x
        corrected_y = aruco_y - rp_offset_y

        if not self.target_loss:
            self.decou_pos_pixel_lpf[0][0] += self.lpf_alpha * (corrected_x - self.decou_pos_pixel_lpf[0][0])
            self.decou_pos_pixel_lpf[0][1] += self.lpf_alpha * (corrected_y - self.decou_pos_pixel_lpf[0][1])

            self.decou_pos_pixel_lpf[1][0] += self.lpf_alpha * (self.decou_pos_pixel_lpf[0][0] - self.decou_pos_pixel_lpf[1][0])
            self.decou_pos_pixel_lpf[1][1] += self.lpf_alpha * (self.decou_pos_pixel_lpf[0][1] - self.decou_pos_pixel_lpf[1][1])

            decoupled_x = self.decou_pos_pixel_lpf[1][0]
            decoupled_y = self.decou_pos_pixel_lpf[1][1]
        else:
            decoupled_x = aruco_x
            decoupled_y = aruco_y

        return (aruco_x, aruco_y, aruco_z), (decoupled_x, decoupled_y, aruco_z)

    def calculate_aruco_x_axis_yaw(self, rvec):
        """
        计算ArUco码X轴在相机坐标系中的朝向（yaw角）
        """
        R, _ = cv2.Rodrigues(rvec)

        aruco_x_axis_in_camera = R[:, 0]

        yaw = math.atan2(aruco_x_axis_in_camera[1], aruco_x_axis_in_camera[0])

        return yaw

    def timer_callback(self):
        if not self.cap.isOpened():
            self.get_logger().warn("相机未打开")
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("无法读取帧")
            return

        undistorted_frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
        gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)

        detector = aruco.ArucoDetector(self.dictionary, self.parameters)
        marker_corners, marker_ids, rejected_candidates = detector.detectMarkers(gray)

        current_position = None
        current_yaw = None

        if marker_ids is not None and len(marker_ids) > 0:
            self.target_loss = 0
            self.target_loss_hold_time = 0
            self.consecutive_loss_count = 0

            for i in range(len(marker_ids)):
                marker_id = marker_ids[i][0] if marker_ids[i].ndim > 0 else marker_ids[i]

                if marker_id != self.target_aruco_id:
                    self.get_logger().debug(f"跳过非目标标记 ID: {marker_id}")
                    continue

                success, rvec, tvec = cv2.solvePnP(
                    np.array([[-self.aruco_length/2, self.aruco_length/2, 0],
                              [self.aruco_length/2, self.aruco_length/2, 0],
                              [self.aruco_length/2, -self.aruco_length/2, 0],
                              [-self.aruco_length/2, -self.aruco_length/2, 0]], dtype=np.float32),
                    marker_corners[i],
                    self.new_camera_matrix,
                    np.zeros(5),
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if success:
                    distance = np.linalg.norm(tvec)
                    calibrated_distance = distance
                    calibrated_tvec = tvec.flatten().copy()

                    if self.distance_calibration is not None:
                        a, b = self.distance_calibration
                        calibrated_distance = a * distance + b
                        if distance > 0:
                            scale_factor = calibrated_distance / distance
                            calibrated_tvec = tvec.flatten() * scale_factor

                    (original_x, original_y, original_z), (decoupled_x, decoupled_y, decoupled_z) = self.decouple_aruco_position(
                        calibrated_tvec.reshape(3, 1), rvec)

                    aruco_x_yaw = self.calculate_aruco_x_axis_yaw(rvec)

                    current_position = [decoupled_x, decoupled_y, decoupled_z]
                    current_yaw = aruco_x_yaw
                    self.last_valid_data = (current_position, current_yaw)

                    if hasattr(self.vehicle_attitude, 'q'):
                        roll, pitch, yaw = self.quaternion_to_euler(
                            self.vehicle_attitude.q[0],
                            self.vehicle_attitude.q[1],
                            self.vehicle_attitude.q[2],
                            self.vehicle_attitude.q[3]
                        )
                        roll_deg = math.degrees(roll)
                        pitch_deg = math.degrees(pitch)
                        yaw_deg = math.degrees(yaw)
                    else:
                        roll_deg = 0.0
                        pitch_deg = 0.0
                        yaw_deg = 0.0

                    current_time = self.get_clock().now()
                    if self.last_log_time is None or (current_time - self.last_log_time).nanoseconds / 1e9 >= self.log_interval:
                        self.last_log_time = current_time
                        self.get_logger().info(
                            f"Marker ID: {marker_id}, "
                            f"解耦前: ({original_x:.3f}, {original_y:.3f}, {original_z:.3f}) m, "
                            f"解耦后: ({decoupled_x:.3f}, {decoupled_y:.3f}, {decoupled_z:.3f}) m, "
                            f"X-axis Yaw: {math.degrees(aruco_x_yaw):.1f} deg, "
                            f"无人机姿态: Roll={roll_deg:.1f}°, Pitch={pitch_deg:.1f}°, Yaw={yaw_deg:.1f}°"
                        )
                    break
                else:
                    self.get_logger().warn(f"Marker ID: {marker_id} 位姿估计失败")
        else:
            if self.target_loss_hold_time < self.tlh_time:
                self.target_loss_hold_time += 1
            else:
                self.target_loss = 1
            self.get_logger().debug("未检测到ArUco标记")

        if current_position is not None and current_yaw is not None:
            for j in range(3):
                self.filtered_position[j] += self.output_lpf_alpha * (current_position[j] - self.filtered_position[j])
            self.filtered_yaw += self.output_lpf_alpha * (current_yaw - self.filtered_yaw)

            position_msg = Point()
            position_msg.x = self.filtered_position[0]
            position_msg.y = self.filtered_position[1]
            position_msg.z = self.filtered_position[2]
            self.aruco_position_publisher.publish(position_msg)

            yaw_msg = Float32()
            yaw_msg.data = self.filtered_yaw
            self.aruco_x_axis_yaw_publisher.publish(yaw_msg)

            detected_msg = Bool()
            detected_msg.data = True
            self.aruco_detected_publisher.publish(detected_msg)
        elif self.last_valid_data is not None and self.consecutive_loss_count < self.max_consecutive_loss:
            self.consecutive_loss_count += 1
            self.get_logger().info(f"使用上一帧数据（连续丢失帧数: {self.consecutive_loss_count}）")

            position_msg = Point()
            position_msg.x = self.filtered_position[0]
            position_msg.y = self.filtered_position[1]
            position_msg.z = self.filtered_position[2]
            self.aruco_position_publisher.publish(position_msg)

            yaw_msg = Float32()
            yaw_msg.data = self.filtered_yaw
            self.aruco_x_axis_yaw_publisher.publish(yaw_msg)

            detected_msg = Bool()
            detected_msg.data = True
            self.aruco_detected_publisher.publish(detected_msg)
        else:
            if self.last_valid_data is not None:
                self.get_logger().warn(f"连续丢失帧数超过阈值（{self.max_consecutive_loss}），停止输出")
            self.last_valid_data = None

            detected_msg = Bool()
            detected_msg.data = False
            self.aruco_detected_publisher.publish(detected_msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    print('启动ArUco检测ROS2节点...')
    rclpy.init(args=args)
    aruco_detector = ArucoDetectorROS2()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
