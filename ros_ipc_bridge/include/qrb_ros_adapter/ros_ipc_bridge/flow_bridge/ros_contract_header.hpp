// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_CONTRACT_HEADER_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_CONTRACT_HEADER_HPP

#include <std_msgs/msg/header.hpp>

#include "qrb/contract/common/v1/header.pb.h"

namespace qrb::adapter
{

/**
 * @brief Field-for-field copy between a ROS header and the Contract `Header`.
 *
 * Both stamps are int32 `sec` plus uint32 `nanosec`, so the copy cannot fail;
 * the Contract validation checks `frame_id` and the `nanosec` range.
 */
void HeaderToContract(const std_msgs::msg::Header & source,
    qrb::contract::common::v1::Header & target);

void HeaderToRos(const qrb::contract::common::v1::Header & source, std_msgs::msg::Header & target);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_FLOW_BRIDGE_ROS_CONTRACT_HEADER_HPP
