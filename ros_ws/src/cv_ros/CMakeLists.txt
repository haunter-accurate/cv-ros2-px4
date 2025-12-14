cmake_minimum_required(VERSION 3.8)
project(cv_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 安装Python模块
ament_python_install_package(${PROJECT_NAME})

# 安装Python脚本作为节点
install(PROGRAMS
  cv_ros/red_color_detector_ros2.py
  cv_ros/color_tracking_offboard.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件（如果有）
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# 安装配置文件（如果有）
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
