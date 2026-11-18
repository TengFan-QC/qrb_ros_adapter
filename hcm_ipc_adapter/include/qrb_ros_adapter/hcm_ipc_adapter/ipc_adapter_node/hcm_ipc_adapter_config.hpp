// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_IPC_ADAPTER_NODE_HCM_IPC_ADAPTER_CONFIG_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_IPC_ADAPTER_NODE_HCM_IPC_ADAPTER_CONFIG_HPP

#include <cstdint>
#include <string>

#include "qrb_ros_adapter/ipc_runtime/flow_config.hpp"
#include "qrb_ros_adapter/ipc_runtime/status.hpp"

namespace qrb::adapter
{

/**
 * @brief Default buffer index of the Flow's descriptor in the frame passed to
 * ProcessFrameDescriptor().
 *
 * The node uses exactly one buffer. Without a `globalBufferIdMap` it is at
 * index 0, the identity map other QCNodes default to.
 */
constexpr std::uint32_t kHcmDefaultGlobalBufferId = 0;

/** @brief HCM-only configuration layered on top of the shared FlowConfig. */
struct HcmIpcAdapterConfig
{
  /** `flowConfig.flowId` is the QCNode `name`. */
  FlowConfig flowConfig;
  std::string nodeName;
  std::uint32_t nodeId{0};
  /** Index of the descriptor in the host frame, from `globalBufferIdMap`. */
  std::uint32_t globalBufferId{kHcmDefaultGlobalBufferId};
};

/** QCNode configuration contract generated from the parser's canonical field lists. */
const std::string & HcmIpcAdapterConfigOptions();

/**
 * @brief Parse the complete QCNode configuration.
 *
 * The QCNode config string, which is JSON, is the only Flow configuration of
 * the HCM IPC Adapter; the ROS IPC Bridge receives it over REGISTER_FLOW.
 * Parsing is kept separate from the QCNode interface methods so it can be
 * unit tested independently.
 *
 * Expected shape -- the `"static"` object is the QCNode static configuration block.
 * Only the first four fields are required:
 *
 *   {
 *     "static": {
 *       "name": "pose_to_hcm",
 *       "contract_type": "qrb.contract.geometry.v1.PoseWithCovarianceStamped",
 *       "direction": 1,
 *       "ros_topic": "/localization/pose",
 *       "id": 0,
 *       "qos_profile": {
 *         "reliability": "reliable",
 *         "durability": "volatile",
 *         "depth": 10
 *       },
 *       "globalBufferIdMap": [ { "name": "Output", "id": 0 } ]
 *     }
 *   }
 *
 * `name` also identifies the Flow to the ROS Bridge Manager, so it must be
 * unique among the Flows registered with one Manager. The Manager assigns the
 * per-Flow data socket. The bootstrap socket path and the timeouts are fixed
 * (FlowConfig defaults), not configured. `direction` is 0 (HCM to ROS: the
 * node publishes the descriptor) or 1 (ROS to HCM: the node fills it in).
 * `qos_profile` sets the ROS reliability (`reliable`, `best_effort`),
 * durability (`volatile`, `transient_local`) and KEEP_LAST depth (1-32); each
 * member is optional and defaults to `rclcpp::QoS( 10 )` (reliable, volatile,
 * depth 10). Payload class and ROS endpoint strategy are
 * resolved from the Contract registries. `globalBufferIdMap` follows the
 * QCNode convention for a frame shared by several nodes: it has exactly one
 * entry, whose `id` is the index of the native qrb_contracts descriptor in the
 * frame (default kHcmDefaultGlobalBufferId) and whose non-empty `name` is
 * informational. Unknown fields are rejected.
 *
 * @param diagnostic Filled in with a human-readable reason on failure.
 * @return kOk, kUnsupportedContract, or kInvalidArgument if the JSON is
 *         malformed or a field is missing, unknown, of the wrong type, or has
 *         an unrecognized value.
 */
Status ParseHcmIpcAdapterConfigJson(const std::string & json,
    HcmIpcAdapterConfig & out,
    std::string & diagnostic);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_IPC_ADAPTER_NODE_HCM_IPC_ADAPTER_CONFIG_HPP
