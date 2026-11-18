// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <array>
#include <iterator>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_contract_registry.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_converters.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_descriptor_registry.hpp"

namespace qrb::adapter
{

namespace
{

// Each binding takes the buffer type its qrb_contracts descriptor sets for itself.
const std::array<HcmContractBinding, ContractDescriptorCount()> kHcmBindings{{
#define QRB_CONTRACT_MAPPING(Id, TypeName, Payload, ContractT, DescriptorT, ...)                   \
  MakeHcmContractBinding<ContractT, DescriptorT>(),
#include "qrb_ros_adapter/ipc_runtime/contract_mappings.def"
#undef QRB_CONTRACT_MAPPING
}};

}  // namespace

std::size_t HcmContractBindingCount()
{
  return std::size(kHcmBindings);
}

const HcmContractBinding * HcmContractBindingAt(std::size_t index)
{
  return index < std::size(kHcmBindings) ? &kHcmBindings[index] : nullptr;
}

const HcmContractBinding * FindHcmContractBinding(std::string_view contractTypeName)
{
  for (const HcmContractBinding & binding : kHcmBindings) {
    if (binding.descriptor != nullptr && binding.descriptor->typeName == contractTypeName) {
      return &binding;
    }
  }
  return nullptr;
}

QC::QCBufferType_e GetQrbBufferDescriptorType(std::string_view contractTypeName)
{
  const HcmContractBinding * binding = FindHcmContractBinding(contractTypeName);
  return binding == nullptr ? QC::QC_BUFFER_TYPE_MAX : binding->descriptorType;
}

QC::QCStatus_e CreateQrbBufferDescriptor(QC::QCBufferType_e type,
    std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> & outDescriptor)
{
  outDescriptor.reset();
  for (const HcmContractBinding & binding : kHcmBindings) {
    if (binding.descriptorType == type && binding.createDescriptor != nullptr) {
      outDescriptor = binding.createDescriptor();
      return outDescriptor == nullptr ? QC::QC_STATUS_NOMEM : QC::QC_STATUS_OK;
    }
  }
  return QC::QC_STATUS_UNSUPPORTED;
}

}  // namespace qrb::adapter
