// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/flow_config.hpp"

namespace qrb::adapter
{

namespace
{

/** @brief ROS topic names must be non-empty and absolute-or-relative, but
 * never contain whitespace; a typo here otherwise surfaces much later as an
 * obscure rclcpp exception during entity creation. */
bool IsPlausibleTopicName(const std::string & topic)
{
  if (topic.empty())
    return false;
  for (char c : topic) {
    if (c == ' ' || c == '\t' || c == '\n')
      return false;
  }
  return true;
}

Status ValidateContractCapabilities(const ContractDescriptor & descriptor,
    Direction direction,
    std::string & diagnostic)
{
  if (descriptor.typeName.empty()) {
    diagnostic += "registry contains an incomplete Contract Descriptor; ";
    return Status::kInternalError;
  }

  if (direction == Direction::kUnspecified) {
    diagnostic += "direction unspecified; ";
    return Status::kUnsupportedContract;
  }

  return Status::kOk;
}

}  // namespace

Status ValidateFlowConfig(const FlowConfig & config,
    const ContractDescriptor & descriptor,
    std::string & diagnostic)
{
  if (config.flowId.empty()) {
    diagnostic += "flow id (QCNode name) must not be empty; ";
    return Status::kInvalidArgument;
  }

  // A generated typed Flow Bridge implements exactly one Contract. Pointing
  // an instance at another type is a deployment error, not something to
  // reconcile at runtime.
  if (config.contractTypeName != descriptor.typeName) {
    diagnostic += "contract_type_name '" + config.contractTypeName +
                  "' does not match this Flow Bridge's descriptor '" +
                  std::string(descriptor.typeName) + "'; ";
    return Status::kInvalidArgument;
  }

  if (!IsPlausibleTopicName(config.rosTopic)) {
    diagnostic += "ros_topic '" + config.rosTopic + "' is empty or contains whitespace; ";
    return Status::kInvalidArgument;
  }

  if (config.timeoutMs < kMinTimeoutMs || config.timeoutMs > kMaxTimeoutMs) {
    diagnostic += "timeout_ms " + std::to_string(config.timeoutMs) + " outside [" +
                  std::to_string(kMinTimeoutMs) + ", " + std::to_string(kMaxTimeoutMs) + "]; ";
    return Status::kInvalidArgument;
  }

  if (config.startupTimeoutMs < kMinStartupTimeoutMs ||
      config.startupTimeoutMs > kMaxStartupTimeoutMs) {
    diagnostic += "startup_timeout_ms " + std::to_string(config.startupTimeoutMs) + " outside [" +
                  std::to_string(kMinStartupTimeoutMs) + ", " +
                  std::to_string(kMaxStartupTimeoutMs) + "]; ";
    return Status::kInvalidArgument;
  }

  const QosProfile & qos = config.qosProfile;
  if (qos.reliability != QosReliability::kReliable &&
      qos.reliability != QosReliability::kBestEffort) {
    diagnostic += "qos_profile.reliability is not supported; ";
    return Status::kInvalidArgument;
  }
  if (qos.durability != QosDurability::kVolatile &&
      qos.durability != QosDurability::kTransientLocal) {
    diagnostic += "qos_profile.durability is not supported; ";
    return Status::kInvalidArgument;
  }
  if (qos.depth < kMinQosDepth || qos.depth > kMaxQosDepth) {
    diagnostic += "qos_profile.depth " + std::to_string(qos.depth) + " outside [" +
                  std::to_string(kMinQosDepth) + ", " + std::to_string(kMaxQosDepth) + "]; ";
    return Status::kInvalidArgument;
  }

  return ValidateContractCapabilities(descriptor, config.direction, diagnostic);
}

}  // namespace qrb::adapter
