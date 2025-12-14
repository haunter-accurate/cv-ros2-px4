# 构建包含PX4依赖的ROS2工作空间

## 问题分析

构建时出现以下错误：
```
CMake Error at CMakeLists.txt:24 (find_package):
  By not providing "Findpx4_msgs.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "px4_msgs",
  but CMake did not find one.
```

原因：**构建顺序问题**。`px4_ros_com` 依赖 `px4_msgs`，但 `colcon build` 默认会并行构建所有包，导致 `px4_ros_com` 在 `px4_msgs` 构建完成前就开始构建。

## 解决方案

### 方法1：按依赖顺序单独构建

```bash
# 进入工作空间目录
cd ~/cv-ros2-px4/ros_ws

# 清理之前的构建产物
rm -rf build install log

# 1. 首先只构建px4_msgs包
colcon build --symlink-install --packages-select px4_msgs

# 2. 然后构建px4_ros_com包
colcon build --symlink-install --packages-select px4_ros_com

# 3. 最后构建cv_ros包
colcon build --symlink-install --packages-select cv_ros

# 加载工作空间环境
source install/setup.bash
```

### 方法2：使用依赖分析构建

```bash
# 进入工作空间目录
cd ~/cv-ros2-px4/ros_ws

# 清理之前的构建产物
rm -rf build install log

# 使用--packages-up-to参数，自动按依赖顺序构建
colcon build --symlink-install --packages-up-to cv_ros

# 加载工作空间环境
source install/setup.bash
```

### 方法3：禁用并行构建

```bash
# 进入工作空间目录
cd ~/cv-ros2-px4/ros_ws

# 清理之前的构建产物
rm -rf build install log

# 使用--parallel-workers 1参数，禁用并行构建
colcon build --symlink-install --parallel-workers 1

# 加载工作空间环境
source install/setup.bash
```

## 验证构建结果

```bash
# 检查所有包是否都已构建成功
ros2 pkg list | grep -E "px4_msgs|px4_ros_com|cv_ros"

# 检查可用的节点
ros2 pkg executables cv_ros
```

**预期输出**：
```
px4_msgs
px4_ros_com
cv_ros
cv_ros color_tracking_offboard
cv_ros red_color_detector_ros2
```

## 运行节点

```bash
# 运行红色色块检测节点
ros2 run cv_ros red_color_detector_ros2

# 在新终端中运行颜色跟踪控制节点
ros2 run cv_ros color_tracking_offboard
```

## 常见问题排查

### 1. 仍然找不到px4_msgs包

```bash
# 检查px4_msgs是否真的构建成功
find ~/cv-ros2-px4/ros_ws/install -name "px4_msgsConfig.cmake"

# 预期输出（类似）：
# /home/hanfei/cv-ros2-px4/ros_ws/install/px4_msgs/share/px4_msgs/cmake/px4_msgsConfig.cmake

# 如果找不到，尝试重新构建px4_msgs
colcon build --symlink-install --packages-select px4_msgs --cmake-force-configure
```

### 2. 其他依赖问题

```bash
# 安装所有缺失的依赖
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 子模块问题

```bash
# 确保子模块已更新
cd ~/cv-ros2-px4
git submodule update --init --recursive
```

## 注意事项

1. 每次修改代码后，只需要重新构建修改的包即可，不需要重新构建所有包
2. 使用 `--symlink-install` 参数可以避免每次修改Python代码都需要重新构建
3. 确保每次打开新终端都执行 `source install/setup.bash` 加载工作空间环境
4. 如果使用VSCode开发，建议在终端中先加载环境再启动VSCode，这样VSCode中的终端也会自动加载环境