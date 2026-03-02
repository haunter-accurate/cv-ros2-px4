import cv2
import cv2.aruco as aruco
import numpy as np
import argparse

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='ArUco Detection for Raspberry Pi')
    parser.add_argument('--headless', action='store_true', help='Run in headless mode (no GUI)')
    args = parser.parse_args()
    
    # 获取无头模式状态
    headless_mode = args.headless
    print(f"Running in {'headless' if headless_mode else 'normal'} mode")
    # 定义ArUco字典和参数
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
    parameters = aruco.DetectorParameters()
    
    # 优化检测参数，提高在树莓派上的检测率
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    parameters.adaptiveThreshWinSizeStep = 10
    parameters.adaptiveThreshConstant = 7
    parameters.minMarkerPerimeterRate = 0.02  # 减小阈值，提高小标记的检测率
    parameters.maxMarkerPerimeterRate = 4.0
    parameters.polygonalApproxAccuracyRate = 0.05
    parameters.minCornerDistanceRate = 0.05
    parameters.minDistanceToBorder = 2  # 减小边界距离要求
    parameters.minMarkerDistanceRate = 0.05
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    parameters.cornerRefinementWinSize = 3  # 减小窗口大小，提高在低分辨率图像上的性能
    parameters.cornerRefinementMaxIterations = 20  # 减少迭代次数，提高速度
    parameters.cornerRefinementMinAccuracy = 0.1
    parameters.markerBorderBits = 1
    parameters.perspectiveRemovePixelPerCell = 4
    parameters.perspectiveRemoveIgnoredMarginPerCell = 0.13
    parameters.maxErroneousBitsInBorderRate = 0.4  # 增加容错率
    parameters.minOtsuStdDev = 3.0  # 减小阈值，提高在不同光照下的检测率
    parameters.errorCorrectionRate = 0.6
    
    # 尝试加载自定义相机标定参数
    cameraMatrix = None
    distCoeffs = None
    
    try:
        # 加载标定数据
        with np.load('camera_calibration_data.npz') as data:
            cameraMatrix = data['camera_matrix']
            distCoeffs = data['dist_coeffs']
        print("成功加载自定义相机标定参数")
    except FileNotFoundError:
        print("未找到自定义相机标定参数，使用默认参数")
        # 使用默认相机标定参数
        fx = 406.932130
        fy = 402.678201
        cx = 316.629381
        cy = 242.533947
        
        k1 = 0.039106
        k2 = -0.056494
        p1 = -0.000824
        p2 = 0.092161
        k3 = 0.0
        
        cameraMatrix = np.array([[fx, 0, cx],
                                 [0, fy, cy],
                                 [0, 0, 1]], dtype=np.float64)
        
        distCoeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
    
    # 尝试加载距离标定参数
    distance_calibration = None
    try:
        with np.load('distance_calibration_data.npz') as data:
            a = data['a']
            b = data['b']
            distance_calibration = (a, b)
        print(f"成功加载距离标定参数：a={a:.6f}, b={b:.6f}")
    except FileNotFoundError:
        print("未找到距离标定参数，使用原始计算值")
    
    # 打开相机
    cap = cv2.VideoCapture(0)  # 树莓派通常使用相机索引 0
    if not cap.isOpened():
        print("无法打开相机")
        return -1
    
    # 获取相机图像尺寸并计算矫正映射
    ret, frame = cap.read()
    if not ret:
        print("无法读取帧")
        return -1
    
    h, w = frame.shape[:2]
    
    # 计算矫正映射
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, (w, h), 1, (w, h))
    mapx, mapy = cv2.initUndistortRectifyMap(cameraMatrix, distCoeffs, None, newCameraMatrix, (w, h), 5)
    
    print("相机已打开，开始检测 ArUco 标记...")
    print("按下 'q' 键退出")
    
    while True:
        # 读取相机帧
        ret, frame = cap.read()
        if not ret:
            print("无法读取帧")
            break
        
        # 应用畸变矫正
        undistorted_frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
        
        # 转换为灰度图像
        gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
        
        # 检测ArUco二维码
        detector = aruco.ArucoDetector(dictionary, parameters)
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray)
        
        # 绘制检测结果
        if markerIds is not None and len(markerIds) > 0:
            print(f"检测到 {len(markerIds)} 个标记")
            aruco.drawDetectedMarkers(undistorted_frame, markerCorners, markerIds)
            
            # 估计相机姿态
            arucoLength = 0.05  # aruco二维码边长
            for i in range(len(markerIds)):
                # 估计单个标记的位姿
                success, rvec, tvec = cv2.solvePnP(
                    np.array([[-arucoLength/2, arucoLength/2, 0],
                              [arucoLength/2, arucoLength/2, 0],
                              [arucoLength/2, -arucoLength/2, 0],
                              [-arucoLength/2, -arucoLength/2, 0]], dtype=np.float32),
                    markerCorners[i],
                    newCameraMatrix,  # 使用新的相机矩阵
                    np.zeros(5),      # 畸变已矫正，使用零畸变系数
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                
                if success:
                    marker_id = markerIds[i][0] if markerIds[i].ndim > 0 else markerIds[i]
                    
                    # 计算相机到标记中心的距离（使用tvec的范数）
                    distance = np.linalg.norm(tvec)
                    
                    # 如果有距离标定参数，使用标定参数校正距离和坐标
                    calibrated_distance = distance
                    calibrated_aruco_in_camera = tvec.flatten()
                    calibrated_camera_in_aruco = None
                    
                    if distance_calibration is not None:
                        a, b = distance_calibration
                        calibrated_distance = a * distance + b
                        print(f"Marker ID: {marker_id}, 原始距离: {distance:.3f} 米, 校正后距离: {calibrated_distance:.3f} 米")
                        
                        # 计算缩放因子
                        if distance > 0:
                            scale_factor = calibrated_distance / distance
                        else:
                            scale_factor = 1.0
                        
                        # 校正aruco码在摄像机坐标系中的坐标
                        calibrated_aruco_in_camera = (tvec.flatten() * scale_factor)
                        print(f"Marker ID: {marker_id}, 原始摄像机坐标系坐标: ({tvec.flatten()[0]:.3f}, {tvec.flatten()[1]:.3f}, {tvec.flatten()[2]:.3f}) 米")
                        print(f"Marker ID: {marker_id}, 校正后摄像机坐标系坐标: ({calibrated_aruco_in_camera[0]:.3f}, {calibrated_aruco_in_camera[1]:.3f}, {calibrated_aruco_in_camera[2]:.3f}) 米")
                        
                        # 计算并校正摄像机在aruco码坐标系中的坐标
                        # 将旋转向量转换为旋转矩阵
                        R, _ = cv2.Rodrigues(rvec)
                        # 计算旋转矩阵的转置
                        R_T = R.T
                        # 计算摄像机在aruco码坐标系中的坐标
                        camera_in_aruco = -R_T @ tvec
                        camera_in_aruco = camera_in_aruco.flatten()
                        # 校正摄像机在aruco码坐标系中的坐标
                        calibrated_camera_in_aruco = camera_in_aruco * scale_factor
                        print(f"Marker ID: {marker_id}, 原始aruco码坐标系坐标: ({camera_in_aruco[0]:.3f}, {camera_in_aruco[1]:.3f}, {camera_in_aruco[2]:.3f}) 米")
                        print(f"Marker ID: {marker_id}, 校正后aruco码坐标系坐标: ({calibrated_camera_in_aruco[0]:.3f}, {calibrated_camera_in_aruco[1]:.3f}, {calibrated_camera_in_aruco[2]:.3f}) 米")
                    else:
                        print(f"Marker ID: {marker_id}, 距离: {distance:.3f} 米")
                        
                        # aruco码在摄像机坐标系中的坐标（就是tvec）
                        aruco_in_camera = tvec.flatten()
                        print(f"Marker ID: {marker_id}, 在摄像机坐标系中的坐标: ({aruco_in_camera[0]:.3f}, {aruco_in_camera[1]:.3f}, {aruco_in_camera[2]:.3f}) 米")
                        
                        # 计算摄像机在aruco码坐标系中的坐标
                        # 将旋转向量转换为旋转矩阵
                        R, _ = cv2.Rodrigues(rvec)
                        # 计算旋转矩阵的转置
                        R_T = R.T
                        # 计算摄像机在aruco码坐标系中的坐标
                        camera_in_aruco = -R_T @ tvec
                        camera_in_aruco = camera_in_aruco.flatten()
                        print(f"Marker ID: {marker_id}, 摄像机在aruco码坐标系中的坐标: ({camera_in_aruco[0]:.3f}, {camera_in_aruco[1]:.3f}, {camera_in_aruco[2]:.3f}) 米")
                    
                    # 非无头模式下绘制相对位姿和文本
                    if not headless_mode:
                        # 绘制相对位姿，缩小轴的大小以避免超出帧范围
                        cv2.drawFrameAxes(undistorted_frame, newCameraMatrix, np.zeros(5), rvec, tvec, 0.05)
                        
                        # 在图像上绘制校正后的坐标信息
                        # 获取标记的左上角坐标作为文本位置
                        if markerCorners[i].size > 0:
                            corner_x = int(markerCorners[i][0][0][0])
                            corner_y = int(markerCorners[i][0][0][1])
                            
                            # 准备要显示的文本
                            if distance_calibration is not None:
                                text = f"ID:{marker_id}\nX:{calibrated_aruco_in_camera[0]:.2f}\nY:{calibrated_aruco_in_camera[1]:.2f}\nZ:{calibrated_aruco_in_camera[2]:.2f}"
                            else:
                                text = f"ID:{marker_id}\nX:{tvec.flatten()[0]:.2f}\nY:{tvec.flatten()[1]:.2f}\nZ:{tvec.flatten()[2]:.2f}"
                            
                            # 绘制文本
                            font = cv2.FONT_HERSHEY_SIMPLEX
                            font_scale = 0.4
                            font_thickness = 1
                            text_color = (0, 255, 0)  # 绿色
                            line_type = cv2.LINE_AA
                            
                            # 逐行绘制文本
                            lines = text.split('\n')
                            for j, line in enumerate(lines):
                                text_size = cv2.getTextSize(line, font, font_scale, font_thickness)[0]
                                text_x = corner_x + 10
                                text_y = corner_y + (j + 1) * (text_size[1] + 5)
                                cv2.putText(undistorted_frame, line, (text_x, text_y), font, font_scale, text_color, font_thickness, line_type)
                else:
                    print(f"Marker ID: {markerIds[i][0]} 位姿估计失败")
        else:
            # 非无头模式下绘制被拒绝的候选标记
            if not headless_mode:
                # 绘制被拒绝的候选标记（用于调试）
                aruco.drawDetectedMarkers(undistorted_frame, rejectedCandidates, None, (100, 0, 255))
            print("未检测到标记，显示被拒绝的候选标记")
        
        # 非无头模式下显示图像和处理按键
        if not headless_mode:
            # 显示图像
            cv2.imshow("ArUco Detection (Undistorted)", undistorted_frame)
            
            # 按下 'q' 键退出循环
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            # 无头模式下，每100毫秒检查一次是否退出
            import time
            time.sleep(0.1)
            # 这里可以添加其他退出条件，比如检测到特定标记或时间限制
    
    # 释放资源
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
