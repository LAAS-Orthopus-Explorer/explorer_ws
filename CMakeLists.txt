cmake_minimum_required(VERSION 3.16)
project(ros2_control_explorer LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_executable(send_trajectory reference_generator/send_trajectory.cpp)

ament_target_dependencies(
  send_trajectory PUBLIC
  ${REF_GEN_INCLUDE_DEPENDS}
)

add_library(
  ros2_control_explorer
  SHARED
  hardware/explorer_hardware.cpp
  controller/explorer_controller.cpp
)

target_compile_features(ros2_control_explorer PUBLIC cxx_std_17)
target_include_directories(ros2_control_explorer PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/ros2_control_explorer>
)
ament_target_dependencies(
  ros2_control_explorer PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_EXPLORER_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_explorer.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface ros2_control_explorer.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_explorer
)

install(
  DIRECTORY description reference_generator
  DESTINATION share/ros2_control_explorer
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_explorer
)
install(
    TARGETS send_trajectory
    RUNTIME DESTINATION lib/ros2_control_explorer
)

install(TARGETS ros2_control_explorer
  EXPORT export_ros2_control_explorer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(explorer_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_explorer_launch test/test_view_robot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_explorer HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()
