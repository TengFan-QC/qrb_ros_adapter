// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_header.hpp"

namespace qrb::adapter
{

void HeaderToContract(const std_msgs::msg::Header & source,
    qrb::contract::common::v1::Header & target)
{
  target.mutable_stamp()->set_sec(source.stamp.sec);
  target.mutable_stamp()->set_nanosec(source.stamp.nanosec);
  target.set_frame_id(source.frame_id);
}

void HeaderToRos(const qrb::contract::common::v1::Header & source, std_msgs::msg::Header & target)
{
  target.stamp.sec = source.stamp().sec();
  target.stamp.nanosec = source.stamp().nanosec();
  target.frame_id = source.frame_id();
}

}  // namespace qrb::adapter
