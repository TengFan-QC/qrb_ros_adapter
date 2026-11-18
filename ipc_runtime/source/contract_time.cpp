// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"

#include <limits>

namespace qrb::adapter
{

ConversionResult ContractTimeToNanos(const qrb::contract::common::v1::Time & time,
    std::uint64_t & outNanos)
{
  if (time.sec() < 0)
    return ConversionResult::Failure("stamp seconds must not be negative");
  if (time.nanosec() >= kNanosPerSecond)
    return ConversionResult::Failure("stamp nanoseconds must be below one second");

  // int32 seconds times 1e9 plus less than 1e9 always fits in uint64.
  outNanos = static_cast<std::uint64_t>(time.sec()) * kNanosPerSecond + time.nanosec();
  return ConversionResult::Success();
}

ConversionResult NanosToContractTime(std::uint64_t nanos, qrb::contract::common::v1::Time & outTime)
{
  const std::uint64_t seconds = nanos / kNanosPerSecond;
  if (seconds > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()))
    return ConversionResult::Failure("timestamp seconds exceed the int32 range of the stamp");

  outTime.set_sec(static_cast<std::int32_t>(seconds));
  outTime.set_nanosec(static_cast<std::uint32_t>(nanos % kNanosPerSecond));
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
