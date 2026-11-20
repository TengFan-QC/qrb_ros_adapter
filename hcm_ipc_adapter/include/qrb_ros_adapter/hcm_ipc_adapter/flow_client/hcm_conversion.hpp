// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_HCM_CONVERSION_HPP
#define QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_HCM_CONVERSION_HPP

#include <cstddef>
#include <cstdint>
#include <string>

#include "QC/Common/QCString.hpp"
#include "qrb_ros_adapter/ipc_runtime/conversion_result.hpp"

namespace qrb::adapter
{

/** @brief Read-only mapping of a received DMA-BUF. */
struct DmaBufView
{
  const void * data{nullptr};
  std::size_t size{0};
};

/** @brief Copy a Contract string into a fixed-size descriptor string; fails if it does not fit. */
template <std::size_t N>
[[nodiscard]] bool AssignHcmString(const std::string & source, QC::QCString<N> & target)
{
  return source.size() <= N && target.Assign(source.data(),
                                   static_cast<std::uint32_t>(source.size())) == QC::QC_STATUS_OK;
}

/** @brief Read a fixed-size descriptor string back as a Contract string. */
template <std::size_t N>
std::string HcmStringToContract(const QC::QCString<N> & source)
{
  return std::string(source.Data(), source.Size());
}

/** @brief Convert a Contract and optional external payload to an HCM descriptor. */
template <typename ContractT, typename DescriptorT>
ConversionResult ConvertContractToHcm(const ContractT & source,
    const DmaBufView & payload,
    DescriptorT & target) = delete;

/** @brief Convert an HCM descriptor to its transport Contract. */
template <typename ContractT, typename DescriptorT>
ConversionResult ConvertHcmToContract(const DescriptorT & source, ContractT & target) = delete;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_HCM_IPC_ADAPTER_FLOW_CLIENT_HCM_CONVERSION_HPP
