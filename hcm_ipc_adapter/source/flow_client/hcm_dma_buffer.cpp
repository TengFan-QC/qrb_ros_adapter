// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_dma_buffer.hpp"

#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <climits>
#include <cstdint>

#include "QC/Infras/Memory/BufferDescriptor.hpp"

// The QCNode build adds the kernel UAPI headers as a plain include directory, and they use
// GNU extensions that -Wpedantic reports.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wvariadic-macros"
#include <linux/dma-buf.h>
#pragma GCC diagnostic pop

namespace qrb::adapter
{

namespace
{

Status SyncDmaBuf(int fd, std::uint64_t flags)
{
  if (fd < 0)
    return Status::kOk;
  dma_buf_sync sync{flags};
  int result = 0;
  do {
    result = ::ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
  } while (result != 0 && (errno == EINTR || errno == EAGAIN));
  // Heaps of uncached memory do not implement the ioctl.
  return result == 0 || errno == ENOTTY ? Status::kOk : Status::kResourceError;
}

}  // namespace

HcmBufferAccess InspectHcmBuffer(const QC::Memory::QCBufferDescriptorBase_t & descriptor, int & fd)
{
  fd = -1;
  const auto * buffer = dynamic_cast<const QC::Memory::BufferDescriptor_t *>(&descriptor);
  if (buffer == nullptr || buffer->pBuf == nullptr)
    return HcmBufferAccess::kNone;
  if (buffer->offset > buffer->size)
    return HcmBufferAccess::kUnusable;
  if (buffer->allocatorType == QC::Memory::QC_MEMORY_ALLOCATOR_HEAP)
    return HcmBufferAccess::kHostMemory;
  // Every DMA allocator (system heap, camera, GPU, ...) hands out a DMA-BUF fd.
  if (buffer->allocatorType < QC::Memory::QC_MEMORY_ALLOCATOR_DMA ||
      buffer->allocatorType >= QC::Memory::QC_MEMORY_ALLOCATOR_LAST ||
      buffer->dmaHandle > static_cast<std::uint64_t>(INT_MAX) ||
      (buffer->pid != 0 && buffer->pid != ::getpid())) {
    return HcmBufferAccess::kUnusable;
  }
  fd = static_cast<int>(buffer->dmaHandle);
  return HcmBufferAccess::kLocalDmaBuf;
}

Status BeginDmaBufCpuRead(int fd)
{
  return SyncDmaBuf(fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
}

Status EndDmaBufCpuRead(int fd)
{
  return SyncDmaBuf(fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
}

Status BeginDmaBufCpuWrite(int fd)
{
  return SyncDmaBuf(fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE);
}

Status EndDmaBufCpuWrite(int fd)
{
  return SyncDmaBuf(fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);
}

}  // namespace qrb::adapter
