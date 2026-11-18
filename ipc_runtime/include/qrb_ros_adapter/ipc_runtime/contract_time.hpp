// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_TIME_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_TIME_HPP

#include <cstdint>

#include "qrb/contract/common/v1/time.pb.h"
#include "qrb_ros_adapter/ipc_runtime/conversion_result.hpp"

namespace qrb::adapter
{

/**
 * @brief Overflow-safe conversion between a Contract `Time` and the unsigned
 * nanosecond count of the HCM descriptors. Both directions can fail:
 *
 *   - `Time.sec` is *signed*. A negative stamp cannot be represented in an
 *     unsigned nanosecond count, and casting it would silently produce a
 *     timestamp roughly 292 years in the future.
 *   - The reverse conversion divides by 1e9 into a signed 32-bit `sec`, which
 *     saturates just past the year 2038.
 *
 * Both are reported rather than clamped: a conversion function must not
 * silently modify its input.
 */

constexpr std::uint64_t kNanosPerSecond = 1000000000ULL;

/**
 * @brief Contract `Time` -> nanoseconds.
 * @return Success, or a diagnostic if `sec` is negative or `nanosec` is not
 *         below one second.
 */
ConversionResult ContractTimeToNanos(const qrb::contract::common::v1::Time & time,
    std::uint64_t & outNanos);

/**
 * @brief Nanoseconds -> Contract `Time`.
 * @return Success, or a diagnostic if the seconds part exceeds int32.
 */
ConversionResult NanosToContractTime(std::uint64_t nanos,
    qrb::contract::common::v1::Time & outTime);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_CONTRACT_TIME_HPP
