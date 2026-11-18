# -----------------------------------------------------------------------------
# qrb_ros_adapter_stream_probe: end-to-end test streams through the installed
# HCM IPC Adapter and ROS IPC Bridge.
#   src/probe.hpp        probe format, statistics and parameters of both sides
#   src/hcm_probe_*      HcmStreamProbeNode (a QCNode) and its host
#                        qrb_ros_adapter_stream_probe, with terminal_ui
#   src/ros_probe_*      RosStreamProbeNode (a component) and ros_stream_probe_node
#   config/              the default parameter file and Adapter static configs
#   scripts/             qrb_ros_adapter_stream_test.sh, one side of a run per terminal
# The HCM side links no ROS, the ROS side no QCNode.
# -----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(qrb_ros_adapter_stream_probe VERSION 0.2.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(qrb_ros_transport_point_cloud2_type REQUIRED)
find_package(qrb_ros_adapter REQUIRED)
find_package(qrb_hcm_ipc_adapter 0.2.0 REQUIRED)
find_package(nlohmann_json 3 REQUIRED)
find_package(Threads REQUIRED)

# --- HCM side --------------------------------------------------------------------

add_library(qrb_ros_adapter_hcm_stream_probe_node SHARED src/hcm_probe_node.cpp)
target_include_directories(qrb_ros_adapter_hcm_stream_probe_node PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(qrb_ros_adapter_hcm_stream_probe_node PUBLIC
  nlohmann_json::nlohmann_json
  qrb_ros_adapter::hcm_ipc_adapter)

add_executable(qrb_ros_adapter_stream_probe src/hcm_probe_main.cpp src/terminal_ui.cpp)
target_link_libraries(qrb_ros_adapter_stream_probe
  qrb_ros_adapter_hcm_stream_probe_node
  Threads::Threads)

# --- ROS side --------------------------------------------------------------------

add_library(qrb_ros_adapter_ros_stream_probe_node SHARED src/ros_probe_node.cpp)
target_include_directories(qrb_ros_adapter_ros_stream_probe_node PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(qrb_ros_adapter_ros_stream_probe_node PUBLIC
  nlohmann_json::nlohmann_json
  qrb_ros_adapter::ros_flow_bridge)
ament_target_dependencies(qrb_ros_adapter_ros_stream_probe_node PUBLIC
  ament_index_cpp rclcpp rclcpp_components geometry_msgs sensor_msgs
  qrb_ros_transport_point_cloud2_type)
rclcpp_components_register_nodes(qrb_ros_adapter_ros_stream_probe_node
  "qrb::adapter::stream_probe::RosStreamProbeNode")

# Its own main rather than a generated one, for the exit code of the probe.
add_executable(ros_stream_probe_node src/ros_probe_main.cpp)
target_link_libraries(ros_stream_probe_node qrb_ros_adapter_ros_stream_probe_node)

# --- Install -------------------------------------------------------------------

install(TARGETS qrb_ros_adapter_hcm_stream_probe_node qrb_ros_adapter_ros_stream_probe_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# The HCM host has no ROS dependency, so it goes on PATH rather than behind `ros2 run`.
install(TARGETS qrb_ros_adapter_stream_probe RUNTIME DESTINATION bin)
install(PROGRAMS scripts/qrb_ros_adapter_stream_test.sh DESTINATION bin)
install(TARGETS ros_stream_probe_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/probe_configs)

ament_package()
