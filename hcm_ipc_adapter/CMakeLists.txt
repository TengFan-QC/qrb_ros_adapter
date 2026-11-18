# -----------------------------------------------------------------------------
# qrb_hcm_ipc_adapter (HCM IPC Adapter)
#
# The HCM IPC Adapter has two cohesive submodules:
#   IPC Adapter Node  Direct QCNodeIfs implementation and configuration.
#   HCM Flow Client   IPC client state, synchronous requests and conversion.
#
# Deliberately does NOT depend on rclcpp: this code runs inside the HCM
# process, and the two runtimes stay isolated.
#
# Exported target: qrb_ros_adapter::hcm_ipc_adapter.
# -----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(qrb_hcm_ipc_adapter VERSION 0.2.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Defines BUILD_TESTING (ON by default) and writes the ctest dashboard config
# that `colcon test` expects.
include(CTest)

find_package(qrb_ros_adapter_runtime ${PROJECT_VERSION} REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(qrb_contracts REQUIRED CONFIG)
include(cmake/QrbQcNode.cmake)

# --- HCM IPC Adapter library -------------------------------------------------

file(GLOB _hcm_contract_converter_sources CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/source/flow_client/*_hcm_converter.cpp")

# Unversioned: hosts load the Adapter Node library by file name.
add_library(qrb_ros_adapter_hcm_ipc_adapter SHARED
  source/flow_client/flow_context.cpp
  source/flow_client/hcm_flow_client.cpp
  source/ipc_adapter_node/hcm_ipc_adapter_config.cpp
  source/ipc_adapter_node/hcm_ipc_adapter_node.cpp
  source/flow_client/hcm_converter_registry.cpp
  ${_hcm_contract_converter_sources})
add_library(qrb_ros_adapter::hcm_ipc_adapter ALIAS qrb_ros_adapter_hcm_ipc_adapter)

target_include_directories(qrb_ros_adapter_hcm_ipc_adapter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

target_compile_features(qrb_ros_adapter_hcm_ipc_adapter PUBLIC cxx_std_17)

target_link_libraries(qrb_ros_adapter_hcm_ipc_adapter PUBLIC
  qrb_ros_adapter::runtime       # also brings qrb_contracts_proto::qrb_contracts_proto
  qrb_contracts::qrb_contracts   # QCNode buffer descriptors, one per Contract
  nlohmann_json::nlohmann_json
  QCNode::QCNode)

target_compile_definitions(qrb_ros_adapter_hcm_ipc_adapter PUBLIC
  QC_TARGET_SOC=${QRB_QCNODE_TARGET_SOC})

set_target_properties(qrb_ros_adapter_hcm_ipc_adapter PROPERTIES
  EXPORT_NAME hcm_ipc_adapter)

# --- Install and CMake package -----------------------------------------------

include(CMakePackageConfigHelpers)
set(_config_install_dir lib/cmake/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS qrb_ros_adapter_hcm_ipc_adapter
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(EXPORT ${PROJECT_NAME}Targets
  NAMESPACE qrb_ros_adapter::
  DESTINATION ${_config_install_dir})

configure_package_config_file(cmake/${PROJECT_NAME}Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${_config_install_dir})
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  COMPATIBILITY SameMinorVersion)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    cmake/QrbQcNode.cmake
  DESTINATION ${_config_install_dir})

# --- Tests -------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(GTest)
  if(TARGET GTest::gtest_main)
    add_subdirectory(test)
  else()
    message(STATUS "GTest not found, skipping HCM IPC Adapter tests")
  endif()
endif()
