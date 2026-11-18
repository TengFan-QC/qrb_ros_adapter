// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/pose_hcm_converter.hpp"

#include "qrb/contract/geometry/v1/pose_with_covariance_stamped.pb.h"
#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter
{

namespace geometry_proto = ::qrb::contract::geometry::v1;

template <>
ConversionResult ConvertContractToHcm<PoseHcmContract, PoseHcmDescriptor>(
    const PoseHcmContract & pose,
    const DmaBufView & /*payload*/,
    PoseHcmDescriptor & descriptor)
{
  ConversionResult validation = ValidatePoseWithCovarianceStamped(pose);
  if (!validation)
    return validation;
  if (pose.header().frame_id().size() > QRB::Memory::QRB_FRAME_ID_MAX_LEN)
    return ConversionResult::Failure("frame_id is longer than the HCM Pose descriptor holds");

  std::uint64_t timestampNs = 0;
  ConversionResult time = ContractTimeToNanos(pose.header().stamp(), timestampNs);
  if (!time)
    return time;

  if (!AssignHcmString(pose.header().frame_id(), descriptor.frameId))
    return ConversionResult::Failure("frame_id does not fit the HCM Pose descriptor");
  descriptor.timestampNs = timestampNs;

  const auto & value = pose.pose();
  descriptor.poseWithCovariance.pose.position.x = value.pose().position().x();
  descriptor.poseWithCovariance.pose.position.y = value.pose().position().y();
  descriptor.poseWithCovariance.pose.position.z = value.pose().position().z();
  descriptor.poseWithCovariance.pose.orientation.x = value.pose().orientation().x();
  descriptor.poseWithCovariance.pose.orientation.y = value.pose().orientation().y();
  descriptor.poseWithCovariance.pose.orientation.z = value.pose().orientation().z();
  descriptor.poseWithCovariance.pose.orientation.w = value.pose().orientation().w();
  descriptor.poseWithCovariance.covariance.fill(0.0);
  for (int i = 0; i < value.covariance_size(); ++i) {
    descriptor.poseWithCovariance.covariance[static_cast<std::size_t>(i)] = value.covariance(i);
  }
  return ConversionResult::Success();
}

template <>
ConversionResult ConvertHcmToContract<PoseHcmContract, PoseHcmDescriptor>(
    const PoseHcmDescriptor & descriptor,
    PoseHcmContract & pose)
{
  pose.Clear();
  ConversionResult time =
      NanosToContractTime(descriptor.timestampNs, *pose.mutable_header()->mutable_stamp());
  if (!time)
    return time;
  pose.mutable_header()->set_frame_id(HcmStringToContract(descriptor.frameId));
  auto * value = pose.mutable_pose();
  auto * position = value->mutable_pose()->mutable_position();
  position->set_x(descriptor.poseWithCovariance.pose.position.x);
  position->set_y(descriptor.poseWithCovariance.pose.position.y);
  position->set_z(descriptor.poseWithCovariance.pose.position.z);
  auto * orientation = value->mutable_pose()->mutable_orientation();
  orientation->set_x(descriptor.poseWithCovariance.pose.orientation.x);
  orientation->set_y(descriptor.poseWithCovariance.pose.orientation.y);
  orientation->set_z(descriptor.poseWithCovariance.pose.orientation.z);
  orientation->set_w(descriptor.poseWithCovariance.pose.orientation.w);
  for (double covariance : descriptor.poseWithCovariance.covariance) {
    value->add_covariance(covariance);
  }

  return ValidatePoseWithCovarianceStamped(pose);
}

}  // namespace qrb::adapter
