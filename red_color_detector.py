#!/usr/bin/env python3
import cv2
import numpy as np


def detect_red_color():
    # 打开摄像头（参数0表示默认摄像头）
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    
    print("红色色块检测程序已启动...")
    print("按 'q' 键退出程序")
    
    while True:
        # 读取一帧视频
        ret, frame = cap.read()
        
        if not ret:
            print("无法获取视频帧")
            break
        
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
        if max_contour is not None and max_area > 500:  # 可根据实际情况调整阈值
            # 计算最小外接矩形
            x, y, w, h = cv2.boundingRect(max_contour)
            # 计算中心点
            center_x = x + w // 2
            center_y = y + h // 2
            
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
            
            # 7. 在控制台输出中心点坐标
            print(f'最大红色色块中心点坐标: ({center_x}, {center_y})')
        
        # 在画面中心添加十字分割线
        frame_height, frame_width = frame.shape[:2]
        center_x = frame_width // 2
        center_y = frame_height // 2
        
        # 绘制水平和垂直分割线
        cv2.line(frame, (0, center_y), (frame_width, center_y), (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(frame, (center_x, 0), (center_x, frame_height), (255, 255, 255), 1, cv2.LINE_AA)
        
        # 显示结果
        cv2.imshow('Red Color Detection', frame)
        #cv2.imshow('Mask', mask)
        
        # 按'q'键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # 释放摄像头和关闭窗口
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    detect_red_color()