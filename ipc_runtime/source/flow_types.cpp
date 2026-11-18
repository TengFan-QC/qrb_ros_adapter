// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/flow_types.hpp"

namespace qrb::adapter
{

const char * ToString(PayloadClass payloadClass)
{
  switch (payloadClass) {
    case PayloadClass::kInline:
      return "inline";
    case PayloadClass::kDmaBuf:
      return "dma_buf";
  }
  return "unknown";
}

const char * ToString(Direction direction)
{
  switch (direction) {
    case Direction::kRosToHcm:
      return "ros_to_hcm";
    case Direction::kHcmToRos:
      return "hcm_to_ros";
    case Direction::kUnspecified:
      return "unspecified";
  }
  return "unknown";
}

const char * ToString(QosReliability reliability)
{
  switch (reliability) {
    case QosReliability::kReliable:
      return "reliable";
    case QosReliability::kBestEffort:
      return "best_effort";
  }
  return "unknown";
}

const char * ToString(QosDurability durability)
{
  switch (durability) {
    case QosDurability::kVolatile:
      return "volatile";
    case QosDurability::kTransientLocal:
      return "transient_local";
  }
  return "unknown";
}

std::int64_t ToDirectionCode(Direction direction)
{
  switch (direction) {
    case Direction::kHcmToRos:
      return kDirectionCodeHcmToRos;
    case Direction::kRosToHcm:
      return kDirectionCodeRosToHcm;
    case Direction::kUnspecified:
      break;
  }
  return -1;
}

bool ParseDirectionCode(std::int64_t code, Direction & out)
{
  if (code == kDirectionCodeHcmToRos)
    out = Direction::kHcmToRos;
  else if (code == kDirectionCodeRosToHcm)
    out = Direction::kRosToHcm;
  else
    return false;
  return true;
}

bool ParseQosReliability(std::string_view text, QosReliability & out)
{
  if (text == "reliable")
    out = QosReliability::kReliable;
  else if (text == "best_effort")
    out = QosReliability::kBestEffort;
  else
    return false;
  return true;
}

bool ParseQosDurability(std::string_view text, QosDurability & out)
{
  if (text == "volatile")
    out = QosDurability::kVolatile;
  else if (text == "transient_local")
    out = QosDurability::kTransientLocal;
  else
    return false;
  return true;
}

}  // namespace qrb::adapter
