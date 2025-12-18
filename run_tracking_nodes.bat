@echo off
REM 启动红色色块检测和颜色跟踪节点的脚本（Windows系统）

REM 颜色检测节点参数
set CAMERA_ID=1
set PUBLISH_RATE=30
set HEADLESS=true

REM 1. 启动红色色块检测节点
start "Red Color Detector" cmd.exe /k "cd /d e:\cv-ros2-px4\cv-ros2-px4\ros_ws && ros2 run cv_ros red_color_detector_ros2 --ros-args -p camera_id:=%CAMERA_ID% -p publish_rate:=%PUBLISH_RATE% -p headless:=%HEADLESS%"

REM 2. 等待2秒，确保第一个节点完全启动
timeout /t 2 /nobreak

REM 3. 启动颜色跟踪offboard控制节点
start "Color Tracking Offboard" cmd.exe /k "cd /d e:\cv-ros2-px4\cv-ros2-px4\ros_ws && ros2 run cv_ros color_tracking_offboard"

echo 两个节点已启动！
echo 按 Ctrl+C 退出脚本
echo.
echo 红色色块检测节点参数：
echo   - 摄像头ID: %CAMERA_ID%
echo   - 发布速率: %PUBLISH_RATE%Hz
echo   - Headless模式: %HEADLESS%
echo.
echo 查看节点输出，请在各自的命令窗口中查看。
