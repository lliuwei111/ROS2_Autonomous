cmake_minimum_required(VERSION 3.5)
project(ROS2_DEMO)

# 添加所有子模块
# add_subdirectory(components/common)
add_subdirectory(components/camera)
add_subdirectory(components/perception)
add_subdirectory(components/planning)
add_subdirectory(components/control)

# 其他配置...
