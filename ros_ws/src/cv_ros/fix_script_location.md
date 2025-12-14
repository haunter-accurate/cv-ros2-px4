# 解决cv_ros包脚本文件位置错误的问题

## 问题现象

执行 `colcon build --symlink-install` 命令时，`px4_msgs` 包编译成功，但 `cv_ros` 包出现以下错误：

```bash
--- stderr: cv_ros                                                                
 CMake Error at ament_cmake_symlink_install/ament_cmake_symlink_install.cmake:204 (message): 
   ament_cmake_symlink_install_programs() can't find 
   '/home/hanfei/cv-ros2-px4/ros_ws/src/cv_ros/cv_ros/red_color_detector_ros2.py' 
 Call Stack (most recent call first): 
   ament_cmake_symlink_install/ament_cmake_symlink_install.cmake:326 (ament_cmake_symlink_install_programs) 
   cmake_install.cmake:46 (include) 
```

## 问题原因

ROS2的Python包要求所有的Python模块文件都应该放在与包同名的子目录中。在这个案例中：

1. CMakeLists.txt文件配置为从 `cv_ros/cv_ros/` 目录安装脚本：
   ```cmake
   install(PROGRAMS
     cv_ros/red_color_detector_ros2.py
     cv_ros/color_tracking_offboard.py
     DESTINATION lib/${PROJECT_NAME}
   )
   ```

2. setup.py文件也配置为从 `cv_ros` 模块中导入这些脚本：
   ```python
   entry_points={
       'console_scripts': [
           'red_color_detector_ros2 = cv_ros.red_color_detector_ros2:main',
           'color_tracking_offboard = cv_ros.color_tracking_offboard:main',
       ],
   },
   ```

但实际上，这些脚本文件被放在了 `cv_ros` 包的根目录下，而不是 `cv_ros/cv_ros/` 子目录中，导致构建系统找不到它们。

## 解决方案

### 步骤1：将脚本文件移动到正确的位置

将脚本文件从包根目录移动到与包同名的子目录中：

```bash
# 在树莓派上执行
cd ~/cv-ros2-px4/ros_ws/src/cv_ros/

# 创建子目录（如果不存在）
mkdir -p cv_ros

# 移动脚本文件到子目录
mv red_color_detector_ros2.py cv_ros/
mv color_tracking_offboard.py cv_ros/
```

### 步骤2：检查文件是否已经正确移动

```bash
# 在树莓派上执行
ls -la ~/cv-ros2-px4/ros_ws/src/cv_ros/cv_ros/
```

预期输出应该包括：
- `__init__.py`
- `red_color_detector_ros2.py`
- `color_tracking_offboard.py`

### 步骤3：重新尝试构建

移动完文件后，重新尝试构建工作空间：

```bash
# 在树莓派上执行
cd ~/cv-ros2-px4/ros_ws

# 重新构建整个工作空间
colcon build --symlink-install
```

## 总结

这个错误是由于ROS2的Python包结构要求导致的。ROS2期望Python包具有以下结构：

```
package_name/
├── package.xml
├── setup.py
├── CMakeLists.txt
└── package_name/
    ├── __init__.py
    ├── script1.py
    └── script2.py
```

通过将脚本文件移动到正确的位置，您应该能够解决这个构建错误。