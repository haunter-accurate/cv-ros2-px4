# cv_ros包重构总结

## 完成的工作

1. **创建废弃代码文件夹**：
   - 在 `cv_ros` 包下创建了 `deprecated` 文件夹，用于存放弃用的代码

2. **移动废弃代码**：
   - 将以下文件移动到 `deprecated` 文件夹中：
     - `gps_position_controller.py`
     - `red_detector_downward.py`
     - `usb_camera_node.py`

3. **更新配置文件**：
   - **setup.py**：移除了已废弃程序的入口点配置，只保留正在验证的两个程序：
     - `red_color_detector_ros2`
     - `color_tracking_offboard`
   
   - **CMakeLists.txt**：移除了已废弃程序的安装配置，只保留正在验证的两个程序

## 当前包结构

```
cv_ros/
├── deprecated/                  # 废弃代码文件夹
│   ├── gps_position_controller.py
│   ├── red_detector_downward.py
│   └── usb_camera_node.py
├── resource/                   # ROS2资源文件夹
│   └── cv_ros
├── red_color_detector_ros2.py  # 正在验证的程序1
├── color_tracking_offboard.py  # 正在验证的程序2
├── setup.py                    # 已更新的包配置
├── CMakeLists.txt              # 已更新的构建配置
├── package.xml                 # 包定义文件
├── requirements.txt            # 依赖配置
└── __init__.py                 # Python包初始化文件
```

## 在树莓派上的后续步骤

### 1. 同步更新到树莓派

将本地的更改同步到树莓派上（假设您使用git进行版本控制）：

```bash
# 在本地提交更改
git add .
git commit -m "Refactor: Move deprecated code to deprecated folder and update config files"

# 在树莓派上拉取更改
cd ~/cv-ros2-px4
git pull
```

如果您使用其他方式同步文件，请确保将以下文件同步到树莓派：
- 更新后的 `setup.py`
- 更新后的 `CMakeLists.txt`
- `deprecated/` 文件夹及其内容

### 2. 重新构建工作空间

```bash
cd ~/cv-ros2-px4/ros_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### 3. 验证更新结果

```bash
# 检查可用的节点（应该只有两个）
ros2 pkg executables cv_ros
```

**预期输出**：
```
cv_ros color_tracking_offboard
cv_ros red_color_detector_ros2
```

### 4. 运行验证的节点

```bash
# 运行红色色块检测节点
ros2 run cv_ros red_color_detector_ros2

# 在新终端中运行颜色跟踪控制节点
ros2 run cv_ros color_tracking_offboard
```

## 注意事项

1. 废弃文件夹中的代码不会被构建为ROS2节点
2. 如果需要恢复使用废弃的代码，需要将文件移回cv_ros根目录并更新setup.py和CMakeLists.txt
3. 建议定期清理不再需要的废弃代码

## 常见问题排查

1. **节点未找到**：确保工作空间已正确构建并加载环境
2. **构建失败**：运行 `rosdep install --from-paths src --ignore-src -r -y` 安装缺少的依赖
3. **权限问题**：确保Python脚本具有执行权限

祝您验证顺利！