// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/point_cloud_hcm_converter.hpp"

#include <cstring>
#include <limits>
#include <string>

#include "qrb/contract/sensor/v1/point_cloud2.pb.h"
#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

namespace qrb::adapter
{

namespace
{

namespace sensor_proto = ::qrb::contract::sensor::v1;

bool PayloadSize(const sensor_proto::PointCloud2 & cloud, std::uint64_t & size)
{
  if (cloud.height() != 0 &&
      cloud.row_step() > std::numeric_limits<std::uint64_t>::max() / cloud.height()) {
    return false;
  }
  size = static_cast<std::uint64_t>(cloud.row_step()) * cloud.height();
  return size != 0;
}

/** @brief The descriptor stores names and frame ids in fixed-size strings. */
ConversionResult CheckDescriptorCapacity(const sensor_proto::PointCloud2 & cloud)
{
  if (cloud.header().frame_id().size() > QRB::Memory::QRB_FRAME_ID_MAX_LEN) {
    return ConversionResult::Failure("frame_id is longer than the HCM PointCloud descriptor holds");
  }
  if (static_cast<std::uint32_t>(cloud.fields_size()) > QRB::Memory::QRB_POINT_CLOUD_MAX_FIELDS) {
    return ConversionResult::Failure(
        "point cloud has more fields than the HCM PointCloud descriptor holds");
  }
  for (const sensor_proto::PointField & field : cloud.fields()) {
    if (field.name().size() > QRB::Memory::QRB_POINT_FIELD_NAME_MAX_LEN) {
      return ConversionResult::Failure(
          "point field name is longer than the HCM PointCloud descriptor holds");
    }
  }
  return ConversionResult::Success();
}

}  // namespace

template <>
ConversionResult ConvertContractToHcm<PointCloudHcmContract, PointCloudHcmDescriptor>(
    const PointCloudHcmContract & cloud,
    const DmaBufView & payload,
    PointCloudHcmDescriptor & descriptor)
{
  std::uint64_t copySize = 0;
  if (!PayloadSize(cloud, copySize)) {
    return ConversionResult::Failure("point cloud payload size is zero or overflows");
  }
  ConversionResult validation = ValidatePointCloud2(cloud, payload.size);
  if (!validation)
    return validation;
  ConversionResult capacity = CheckDescriptorCapacity(cloud);
  if (!capacity)
    return capacity;
  std::uint64_t timestampNs = 0;
  ConversionResult time = ContractTimeToNanos(cloud.header().stamp(), timestampNs);
  if (!time)
    return time;
  if (payload.data == nullptr || copySize > payload.size) {
    return ConversionResult::Failure(
        "source DMA-BUF is missing or smaller than the Contract payload");
  }
  if (descriptor.pBuf == nullptr || descriptor.offset > descriptor.size ||
      copySize > descriptor.size - descriptor.offset) {
    return ConversionResult::Failure("HCM PointCloud descriptor buffer is missing or too small");
  }

  descriptor.timestampNs = timestampNs;
  if (!AssignHcmString(cloud.header().frame_id(), descriptor.frameId))
    return ConversionResult::Failure("frame_id does not fit the HCM PointCloud descriptor");
  descriptor.height = cloud.height();
  descriptor.width = cloud.width();
  descriptor.pointStep = cloud.point_step();
  descriptor.rowStep = cloud.row_step();
  descriptor.isBigEndian = cloud.is_bigendian();
  descriptor.isDense = cloud.is_dense();
  descriptor.numFields = static_cast<std::uint32_t>(cloud.fields_size());
  for (QRB::Memory::PointField_t & out : descriptor.fields) {
    out = QRB::Memory::PointField_t();
  }
  // CheckDescriptorCapacity bounds numFields by QRB_POINT_CLOUD_MAX_FIELDS.
  for (std::uint32_t i = 0; i < descriptor.numFields; ++i) {
    QRB::Memory::PointField_t & out = descriptor.fields[i];
    const sensor_proto::PointField & field = cloud.fields(static_cast<int>(i));
    if (!AssignHcmString(field.name(), out.name)) {
      return ConversionResult::Failure(
          "point field name does not fit the HCM PointCloud descriptor");
    }
    out.offset = field.offset();
    out.datatype = static_cast<QRB::Memory::PointFieldType_e>(field.datatype());
    out.count = field.count();
  }

  std::memcpy(descriptor.GetDataPtr(), payload.data, static_cast<std::size_t>(copySize));
  descriptor.validSize = static_cast<std::size_t>(copySize);
  return ConversionResult::Success();
}

template <>
ConversionResult ConvertHcmToContract<PointCloudHcmContract, PointCloudHcmDescriptor>(
    const PointCloudHcmDescriptor & descriptor,
    PointCloudHcmContract & cloud)
{
  if (descriptor.numFields > QRB::Memory::QRB_POINT_CLOUD_MAX_FIELDS) {
    return ConversionResult::Failure("HCM PointCloud numFields exceeds the descriptor capacity");
  }

  cloud.Clear();
  ConversionResult time =
      NanosToContractTime(descriptor.timestampNs, *cloud.mutable_header()->mutable_stamp());
  if (!time)
    return time;
  cloud.mutable_header()->set_frame_id(HcmStringToContract(descriptor.frameId));
  cloud.set_height(descriptor.height);
  cloud.set_width(descriptor.width);
  cloud.set_point_step(descriptor.pointStep);
  cloud.set_row_step(descriptor.rowStep);
  cloud.set_is_bigendian(descriptor.isBigEndian);
  cloud.set_is_dense(descriptor.isDense);

  for (std::uint32_t i = 0; i < descriptor.numFields; ++i) {
    const QRB::Memory::PointField_t & in = descriptor.fields[i];
    sensor_proto::PointField * field = cloud.add_fields();
    field->set_name(HcmStringToContract(in.name));
    field->set_offset(in.offset);
    field->set_datatype(static_cast<sensor_proto::PointFieldType>(in.datatype));
    field->set_count(in.count);
  }

  std::uint64_t expectedSize = 0;
  if (!PayloadSize(cloud, expectedSize)) {
    return ConversionResult::Failure("HCM PointCloud payload size is zero or overflows");
  }
  if (descriptor.pBuf == nullptr || descriptor.offset > descriptor.size ||
      descriptor.validSize < expectedSize ||
      descriptor.validSize > descriptor.size - descriptor.offset) {
    return ConversionResult::Failure(
        "HCM PointCloud payload storage is inconsistent with its metadata");
  }
  return ValidatePointCloud2(cloud, descriptor.validSize);
}

}  // namespace qrb::adapter
