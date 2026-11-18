// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/ipc_adapter_node/hcm_ipc_adapter_config.hpp"

#include <algorithm>
#include <array>
#include <iterator>
#include <limits>
#include <nlohmann/json.hpp>
#include <string_view>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_converters.hpp"

namespace qrb::adapter
{

namespace
{

namespace field
{
constexpr char kStatic[] = "static";
constexpr char kName[] = "name";
constexpr char kId[] = "id";
constexpr char kContractType[] = "contract_type";
constexpr char kDirection[] = "direction";
constexpr char kRosTopic[] = "ros_topic";
constexpr char kQosProfile[] = "qos_profile";
constexpr char kReliability[] = "reliability";
constexpr char kDurability[] = "durability";
constexpr char kDepth[] = "depth";
constexpr char kGlobalBufferIdMap[] = "globalBufferIdMap";
}  // namespace field

constexpr std::array<const char *, 4> kRequiredOptionFields = {
    field::kName, field::kContractType, field::kDirection, field::kRosTopic};
constexpr std::array<const char *, 3> kOptionalOptionFields = {
    field::kId, field::kQosProfile, field::kGlobalBufferIdMap};
constexpr std::array<const char *, 3> kQosProfileFields = {
    field::kReliability, field::kDurability, field::kDepth};
constexpr std::array<const char *, 2> kBufferMapEntryFields = {field::kName, field::kId};

template <std::size_t N>
bool Contains(const std::array<const char *, N> & fields, std::string_view key)
{
  return std::any_of(
      fields.begin(), fields.end(), [key](const char * known) { return key == known; });
}

bool IsKnownField(std::string_view key)
{
  return Contains(kRequiredOptionFields, key) || Contains(kOptionalOptionFields, key);
}

/** `direction` is an integer: 0 HCM to ROS, 1 ROS to HCM. */
bool GetRequiredDirection(const nlohmann::json & node, Direction & out, std::string & diagnostic)
{
  auto it = node.find(field::kDirection);
  if (it == node.end()) {
    diagnostic += "missing field \"direction\"; ";
    return false;
  }
  if (it->is_number_integer() && ParseDirectionCode(it->get<std::int64_t>(), out)) {
    return true;
  }
  diagnostic += "\"direction\" must be 0 (HCM to ROS) or 1 (ROS to HCM), got " + it->dump() + "; ";
  return false;
}

bool GetRequiredString(const nlohmann::json & node,
    const char * key,
    std::string & out,
    std::string & diagnostic)
{
  auto it = node.find(key);
  if (it == node.end() || !it->is_string()) {
    diagnostic += std::string("missing or non-string field \"") + key + "\"; ";
    return false;
  }
  out = it->get<std::string>();
  return true;
}

bool GetOptionalString(const nlohmann::json & node,
    const char * key,
    std::string & out,
    std::string & diagnostic)
{
  auto it = node.find(key);
  if (it == node.end())
    return true;  // keep the default
  if (!it->is_string()) {
    diagnostic += std::string("\"") + key + "\" must be a string; ";
    return false;
  }
  out = it->get<std::string>();
  return true;
}

bool GetOptionalUint(const nlohmann::json & node,
    const char * key,
    std::uint32_t & out,
    std::string & diagnostic,
    std::uint32_t max = std::numeric_limits<std::uint32_t>::max())
{
  auto it = node.find(key);
  if (it == node.end())
    return true;  // keep the default
  if (!it->is_number_unsigned()) {
    diagnostic += std::string("\"") + key + "\" must be a non-negative integer; ";
    return false;
  }
  const std::uint64_t value = it->get<std::uint64_t>();
  if (value > max) {
    diagnostic += std::string("\"") + key + "\" must be in [0, " + std::to_string(max) + "]; ";
    return false;
  }
  out = static_cast<std::uint32_t>(value);
  return true;
}

/** Parse the optional `qos_profile` object; absent members keep their default. */
bool ParseQosProfileObject(const nlohmann::json & node, QosProfile & out, std::string & diagnostic)
{
  auto it = node.find(field::kQosProfile);
  if (it == node.end())
    return true;
  if (!it->is_object()) {
    diagnostic += "\"qos_profile\" must be an object; ";
    return false;
  }
  const nlohmann::json & qos = *it;

  bool ok = true;
  for (const auto & item : qos.items()) {
    if (!Contains(kQosProfileFields, item.key())) {
      diagnostic += "unknown field \"qos_profile." + item.key() + "\"; ";
      ok = false;
    }
  }

  std::string reliability = ToString(out.reliability);
  std::string durability = ToString(out.durability);
  ok &= GetOptionalString(qos, field::kReliability, reliability, diagnostic);
  ok &= GetOptionalString(qos, field::kDurability, durability, diagnostic);
  ok &= GetOptionalUint(qos, field::kDepth, out.depth, diagnostic);
  if (!ok)
    return false;

  if (!ParseQosReliability(reliability, out.reliability)) {
    diagnostic += "\"qos_profile.reliability\" must be \"reliable\" or \"best_effort\", got \"" +
                  reliability + "\"; ";
    return false;
  }
  if (!ParseQosDurability(durability, out.durability)) {
    diagnostic +=
        "\"qos_profile.durability\" must be \"volatile\" or \"transient_local\", "
        "got \"" +
        durability + "\"; ";
    return false;
  }
  if (out.depth < kMinQosDepth || out.depth > kMaxQosDepth) {
    diagnostic += "\"qos_profile.depth\" must be in [" + std::to_string(kMinQosDepth) + ", " +
                  std::to_string(kMaxQosDepth) + "]; ";
    return false;
  }
  return true;
}

/**
 * Parse the optional QCNode `globalBufferIdMap`. The node has one buffer, so
 * the map has one entry; like CL2DFlex, its `name` must be non-empty and its
 * `id` set (UINT32_MAX is QCNode's "unset").
 */
bool ParseGlobalBufferIdMap(const nlohmann::json & node,
    std::uint32_t & out,
    std::string & diagnostic)
{
  auto it = node.find(field::kGlobalBufferIdMap);
  if (it == node.end())
    return true;
  if (!it->is_array() || it->size() != 1 || !(*it)[0].is_object()) {
    diagnostic += "\"globalBufferIdMap\" must be an array of one {\"name\", \"id\"} entry; ";
    return false;
  }
  const nlohmann::json & entry = (*it)[0];

  bool ok = true;
  for (const auto & item : entry.items()) {
    if (!Contains(kBufferMapEntryFields, item.key())) {
      diagnostic += "unknown field \"globalBufferIdMap[0]." + item.key() + "\"; ";
      ok = false;
    }
  }
  std::string name;
  ok &= GetRequiredString(entry, field::kName, name, diagnostic);
  if (ok && name.empty()) {
    diagnostic += "\"globalBufferIdMap[0].name\" must not be empty; ";
    ok = false;
  }
  if (entry.find(field::kId) == entry.end()) {
    diagnostic += "missing field \"globalBufferIdMap[0].id\"; ";
    return false;
  }
  ok &= GetOptionalUint(
      entry, field::kId, out, diagnostic, std::numeric_limits<std::uint32_t>::max() - 1);
  return ok;
}

}  // namespace

const std::string & HcmIpcAdapterConfigOptions()
{
  static const std::string options = []() {
    nlohmann::json schema;
    for (const char * key : kRequiredOptionFields)
      schema["required"].push_back(std::string("static.") + key);
    schema["optional"].push_back(std::string("static.") + field::kId);
    for (const char * key : kQosProfileFields)
      schema["optional"].push_back(std::string("static.") + field::kQosProfile + "." + key);
    schema["optional"].push_back(std::string("static.") + field::kGlobalBufferIdMap);
    return schema.dump();
  }();
  return options;
}

Status ParseHcmIpcAdapterConfigJson(const std::string & json,
    HcmIpcAdapterConfig & out,
    std::string & diagnostic)
{
  nlohmann::json root;
  try {
    root = nlohmann::json::parse(json);
  } catch (const nlohmann::json::parse_error & e) {
    diagnostic += std::string("JSON parse error: ") + e.what() + "; ";
    return Status::kInvalidArgument;
  }

  auto staticIt = root.find(field::kStatic);
  if (staticIt == root.end() || !staticIt->is_object()) {
    diagnostic += "missing \"static\" object; ";
    return Status::kInvalidArgument;
  }
  const nlohmann::json & node = *staticIt;

  HcmIpcAdapterConfig config;
  FlowConfig & flow = config.flowConfig;
  bool ok = true;

  // A misspelled optional field would otherwise silently keep its default.
  for (const auto & item : node.items()) {
    if (!IsKnownField(item.key())) {
      diagnostic += "unknown field \"" + item.key() + "\"; ";
      ok = false;
    }
  }

  ok &= GetRequiredString(node, field::kName, config.nodeName, diagnostic);
  ok &= GetRequiredString(node, field::kContractType, flow.contractTypeName, diagnostic);
  ok &= GetRequiredDirection(node, flow.direction, diagnostic);
  ok &= GetRequiredString(node, field::kRosTopic, flow.rosTopic, diagnostic);

  ok &= GetOptionalUint(node, field::kId, config.nodeId, diagnostic, UINT8_MAX);
  ok &= ParseQosProfileObject(node, flow.qosProfile, diagnostic);
  ok &= ParseGlobalBufferIdMap(node, config.globalBufferId, diagnostic);

  if (!ok)
    return Status::kInvalidArgument;

  if (config.nodeName.empty()) {
    diagnostic += "\"name\" must not be empty; ";
    return Status::kInvalidArgument;
  }
  // The QCNode name is the Flow identity seen by the ROS Bridge Manager.
  flow.flowId = config.nodeName;

  const HcmContractBinding * binding = FindHcmContractBinding(flow.contractTypeName);
  if (binding == nullptr || binding->descriptor == nullptr) {
    diagnostic += "\"contract_type\" has no HCM implementation; ";
    return Status::kUnsupportedContract;
  }

  out = std::move(config);
  return Status::kOk;
}

}  // namespace qrb::adapter
