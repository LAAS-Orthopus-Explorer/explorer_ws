cmake_minimum_required(VERSION 3.5.0)
project(ros2_control_wheelchair)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop 
  rclcpp
  geometry_msgs
)

install(
  DIRECTORY description src
  DESTINATION share/ros2_control_wheelchair
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_wheelchair
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

## Install
install(
  TARGETS
  teleop
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()