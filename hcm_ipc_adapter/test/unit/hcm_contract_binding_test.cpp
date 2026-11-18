// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <gtest/gtest.h>

#include <memory>
#include <set>
#include <string>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_converters.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/point_cloud_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/pose_hcm_converter.hpp"

using namespace qrb::adapter;

// UT-HCM-02: every shared Contract has exactly one complete HCM binding, and the
// descriptor factory creates descriptors only that binding accepts.
TEST(HcmContractBinding, EveryContractHasOneCompleteBindingAndAMatchingFactory)
{
  ASSERT_EQ(HcmContractBindingCount(), ContractDescriptorCount());
  std::set<int> types;
  for (std::size_t i = 0; i < HcmContractBindingCount(); ++i) {
    const HcmContractBinding * binding = HcmContractBindingAt(i);
    ASSERT_NE(binding, nullptr);
    ASSERT_NE(binding->descriptor, nullptr);
    const std::string name(binding->descriptor->typeName);
    EXPECT_EQ(FindHcmContractBinding(name), binding) << name;
    EXPECT_NE(binding->acceptsDescriptor, nullptr) << name;
    EXPECT_NE(binding->contractToHcm, nullptr) << name;
    EXPECT_NE(binding->hcmToContract, nullptr) << name;
    EXPECT_NE(binding->createDescriptor, nullptr) << name;

    // Types come from the QCNode custom range and never collide.
    EXPECT_GE(binding->descriptorType, kHcmCustomDescriptorTypeBegin) << name;
    EXPECT_LT(binding->descriptorType, kHcmCustomDescriptorTypeEnd) << name;
    EXPECT_TRUE(types.insert(static_cast<int>(binding->descriptorType)).second) << name;
    EXPECT_EQ(GetQrbBufferDescriptorType(name), binding->descriptorType) << name;
  }
  EXPECT_EQ(HcmContractBindingAt(HcmContractBindingCount()), nullptr);
  EXPECT_EQ(FindHcmContractBinding("qrb.contract.unknown.v1.Missing"), nullptr);
  EXPECT_EQ(GetQrbBufferDescriptorType("qrb.contract.unknown.v1.Missing"), QC::QC_BUFFER_TYPE_MAX);

  for (std::size_t i = 0; i < HcmContractBindingCount(); ++i) {
    const HcmContractBinding & binding = *HcmContractBindingAt(i);
    std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> created;
    ASSERT_EQ(CreateQrbBufferDescriptor(binding.descriptorType, created), QC::QC_STATUS_OK);
    ASSERT_NE(created, nullptr);
    // The factory sets the registry-assigned type, overriding the header default.
    EXPECT_EQ(created->type, binding.descriptorType);
    EXPECT_TRUE(binding.acceptsDescriptor(*created));
  }

  std::unique_ptr<QC::Memory::QCBufferDescriptorBase_t> descriptor =
      std::make_unique<QC::Memory::QCBufferDescriptorBase_t>();
  EXPECT_EQ(CreateQrbBufferDescriptor(kHcmCustomDescriptorTypeEnd, descriptor),
      QC::QC_STATUS_UNSUPPORTED);
  EXPECT_EQ(descriptor, nullptr);

  // A binding never accepts or converts another Contract's descriptor.
  const HcmContractBinding & pose = *FindHcmContractBinding(kPoseWithCovarianceStampedContractType);
  PointCloudHcmDescriptor cloudDescriptor;
  PoseHcmContract poseContract;
  EXPECT_FALSE(pose.acceptsDescriptor(cloudDescriptor));
  EXPECT_FALSE(pose.contractToHcm(poseContract, DmaBufView{}, cloudDescriptor));
  EXPECT_FALSE(pose.hcmToContract(cloudDescriptor, poseContract));
}
