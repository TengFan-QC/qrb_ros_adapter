// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

// Converters are stateless functions over message structures: no ROS Context,
// no node, no DMA heap. A memfd stands in for a DMA-BUF where only fd identity
// and size matter.

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdint>
#include <lib_mem_dmabuf/dmabuf.hpp>
#include <memory>
#include <sensor_msgs/msg/point_field.hpp>

#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/point_cloud_ros_converter.hpp"
#include "qrb_ros_adapter/ros_ipc_bridge/flow_bridge/ros_contract_time.hpp"

using namespace qrb::adapter;

namespace sensor_proto = ::qrb::contract::sensor::v1;

namespace
{

std::shared_ptr<lib_mem_dmabuf::DmaBuffer> MakeMemfdBuffer(std::size_t size)
{
  const int fd = ::memfd_create("qrb_ros_adapter_converter_test", MFD_CLOEXEC);
  if (fd < 0)
    return nullptr;
  if (::ftruncate(fd, static_cast<off_t>(size)) != 0) {
    ::close(fd);
    return nullptr;
  }
  return std::make_shared<lib_mem_dmabuf::DmaBuffer>(fd, size);
}

sensor_msgs::msg::PointField Field(const char * name, std::uint32_t offset, std::uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

constexpr std::uint32_t kPointStep = 32;
constexpr std::uint32_t kPoints = 4;
constexpr std::uint32_t kCloudBytes = kPointStep * kPoints;

/** @brief One point of every PointField datatype, packed into 32 bytes. */
PointCloudRosMessage MakeRosCloud(std::size_t bufferSize = kCloudBytes)
{
  using PF = sensor_msgs::msg::PointField;
  // The transport type leaves its scalars uninitialized; an indeterminate
  // bool serializes as a malformed varint.
  auto cloud = PointCloudRosMessage();
  cloud.header.stamp.sec = 100;
  cloud.header.stamp.nanosec = 500;
  cloud.header.frame_id = "lidar";
  cloud.height = 1;
  cloud.width = kPoints;
  cloud.point_step = kPointStep;
  cloud.row_step = kCloudBytes;
  cloud.is_dense = true;
  cloud.fields = {Field("i8", 0, PF::INT8), Field("u8", 1, PF::UINT8), Field("i16", 2, PF::INT16),
      Field("u16", 4, PF::UINT16), Field("i32", 8, PF::INT32), Field("u32", 12, PF::UINT32),
      Field("f32", 16, PF::FLOAT32), Field("f64", 24, PF::FLOAT64)};
  cloud.data_fd = MakeMemfdBuffer(bufferSize);
  return cloud;
}

}  // namespace

// UT-ROS-02: PointCloud2 metadata and every PointField datatype map 1:1 and the
// payload context binds the DMA-BUF instead of copying bytes; a layout the
// payload cannot back is rejected, and a payload context binds once within
// offset + length <= capacity.
TEST(PointCloudRosConverter, MapsMetadataBindsTheDmaBufAndRejectsWhatItCannotBack)
{
  const PointCloudRosMessage source = MakeRosCloud(kCloudBytes + 64);
  ASSERT_NE(source.data_fd, nullptr);

  PointCloudRosContract contract;
  RosToContractPayload outbound;
  ASSERT_TRUE(PointCloudRosConversion::RosToContract(source, contract, outbound));
  EXPECT_EQ(contract.timestamp_ns(), 100 * kNanosPerSecond + 500);
  EXPECT_EQ(contract.row_step(), kCloudBytes);
  ASSERT_EQ(contract.fields_size(), 8);
  EXPECT_EQ(contract.fields(7).datatype(), sensor_proto::POINT_FIELD_TYPE_FLOAT64);
  // The valid range is the layout size, not the buffer capacity.
  EXPECT_EQ(outbound.Buffer(), source.data_fd);
  EXPECT_EQ(outbound.Size(), kCloudBytes);
  EXPECT_EQ(outbound.Offset(), 0u);

  auto back = PointCloudRosMessage();
  ContractToRosPayload inbound(kCloudBytes);
  ASSERT_TRUE(PointCloudRosConversion::ContractToRos(contract, back, inbound));
  EXPECT_EQ(inbound.Size(), kCloudBytes);
  EXPECT_EQ(back.data_fd, nullptr);  // the caller supplies the ROS-owned buffer
  const auto destination = MakeMemfdBuffer(kCloudBytes);
  ASSERT_TRUE(inbound.Assign(destination));
  EXPECT_EQ(back.data_fd, destination);

  EXPECT_EQ(back.header, source.header);
  EXPECT_EQ(back.fields, source.fields);
  EXPECT_EQ(back.width, source.width);
  EXPECT_EQ(back.point_step, source.point_step);
  EXPECT_EQ(back.row_step, source.row_step);
  EXPECT_EQ(back.is_dense, source.is_dense);

  {
    PointCloudRosMessage invalid = MakeRosCloud();
    invalid.fields[0].datatype = 0;  // not a PointField datatype
    RosToContractPayload payload;
    EXPECT_FALSE(PointCloudRosConversion::RosToContract(invalid, contract, payload));
    EXPECT_FALSE(payload.IsBound());
  }
  {
    PointCloudRosMessage tooShort = MakeRosCloud(kCloudBytes - 1);
    RosToContractPayload payload;
    EXPECT_FALSE(PointCloudRosConversion::RosToContract(tooShort, contract, payload));
  }
  {
    PointCloudRosMessage withoutBuffer = MakeRosCloud();
    withoutBuffer.data_fd.reset();
    RosToContractPayload payload;
    EXPECT_FALSE(PointCloudRosConversion::RosToContract(withoutBuffer, contract, payload));
  }

  // HCM -> ROS: the received DMA-BUF range must cover the declared layout.
  RosToContractPayload valid;
  ASSERT_TRUE(PointCloudRosConversion::RosToContract(MakeRosCloud(), contract, valid));
  auto target = PointCloudRosMessage();
  ContractToRosPayload tooSmall(kCloudBytes - 1);
  EXPECT_FALSE(PointCloudRosConversion::ContractToRos(contract, target, tooSmall));
  EXPECT_FALSE(tooSmall.IsBound());

  // Source side: offset + length must fit the producer's buffer.
  const auto buffer = MakeMemfdBuffer(256);
  ASSERT_NE(buffer, nullptr);
  RosToContractPayload bounded;
  EXPECT_FALSE(bounded.Bind(nullptr, 16));
  EXPECT_FALSE(bounded.Bind(buffer, 129, 128));
  EXPECT_FALSE(bounded.IsBound());
  ASSERT_TRUE(bounded.Bind(buffer, 128, 128));
  EXPECT_EQ(bounded.Offset(), 128u);
  EXPECT_FALSE(bounded.Bind(buffer, 16));  // already bound
  EXPECT_EQ(bounded.Size(), 128u);

  // Destination side: the length must fit the received range.
  std::shared_ptr<lib_mem_dmabuf::DmaBuffer> unassigned;
  ContractToRosPayload received(64);
  EXPECT_FALSE(received.Assign(MakeMemfdBuffer(64)));  // nothing bound yet
  EXPECT_FALSE(received.Bind(unassigned, 65));
  ASSERT_TRUE(received.Bind(unassigned, 64));
  EXPECT_FALSE(received.Bind(unassigned, 32));  // already bound
  EXPECT_FALSE(received.Assign(nullptr));
  EXPECT_EQ(unassigned, nullptr);
}
