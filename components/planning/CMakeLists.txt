cmake_minimum_required(VERSION 3.8)
project(planning)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)

#! Composable nodes must be compiled as shared libraries.
#! Pay attention to the following, new directives, to compile shared libraries,
#! then to those necessary to register the new plugins.

# planning component
add_library(planning_sdk SHARED src/planning.cpp)
target_compile_definitions(planning_sdk PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(planning_sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/opencv4)
ament_target_dependencies(
  planning_sdk
  rclcpp
  rclcpp_components
  std_msgs)
rclcpp_components_register_nodes(planning_sdk "planning_component")

# planning
add_executable(planning src/planning_main.cpp)
target_include_directories(planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/opencv4)
target_link_libraries(planning 
  planning_sdk
  ${OpenCV_LIBS})
ament_target_dependencies(
  planning
  rclcpp
  cv_bridge
  sensor_msgs)

# Install components
install(TARGETS planning_sdk
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS planning
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
