// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

#include "qrb_ros_adapter/ros_ipc_bridge/manager/ros_bridge_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto manager = std::make_shared<qrb::adapter::RosBridgeManager>(executor);
  // A fixed name: `ros2 component load /qrb_ros_adapter_container ...` puts a
  // node in this process, e.g. the stream probe, to share DMA-BUFs with Flows.
  auto componentManager =
      std::make_shared<rclcpp_components::ComponentManager>(executor, "qrb_ros_adapter_container");
  executor->add_node(manager);
  executor->add_node(componentManager);
  manager->Start();
  RCLCPP_INFO(manager->get_logger(), "component container: /qrb_ros_adapter_container");
  executor->spin();
  manager->Stop();
  executor->remove_node(manager);
  executor->remove_node(componentManager);
  rclcpp::shutdown();
  return 0;
}
