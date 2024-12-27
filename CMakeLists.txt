cmake_minimum_required(VERSION 3.5)
project(ROS2_DEMO)

# 添加所有子模块
add_subdirectory(components/camera)
add_subdirectory(components/perception)

# 其他配置...
