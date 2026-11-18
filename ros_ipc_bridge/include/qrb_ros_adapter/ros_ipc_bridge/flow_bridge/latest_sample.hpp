// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_LATEST_SAMPLE_HPP
#define QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_LATEST_SAMPLE_HPP

#include <mutex>
#include <utility>

namespace qrb::adapter
{

/**
 * @brief Holds the most recent message from a Subscription until the HCM IPC Adapter
 * asks for it.
 *
 * The subscription callback parks the message here instead of the bridge
 * calling `Subscription::take()` at fetch time: `take()` reads only the
 * middleware queue, which intra-process messages (the qrb_ros_transport
 * zero-copy path) never enter.
 *
 * Only the newest message is kept: the HCM IPC Adapter pulls at its
 * own cadence, and for sensor and pose data the newest sample is the useful one.
 *
 * Thread-safe: the executor stores, the Flow's service thread takes.
 */
template <typename MessageT>
class LatestSample
{
public:
  /** @brief Replace the held sample. Called from the subscription callback. */
  void Store(MessageT message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    message_ = std::move(message);
    fresh_ = true;
  }

  /**
   * @brief Consume the held sample, if one has arrived since the last take.
   *
   * @return false if nothing new is immediately available.
   */
  bool Take(MessageT & out)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fresh_)
      return false;
    out = std::move(message_);
    message_ = MessageT();
    fresh_ = false;
    return true;
  }

  /** @brief True if a sample is waiting. */
  bool HasSample() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return fresh_;
  }

  /** @brief Drop any held sample, releasing whatever it owns. */
  void Clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    message_ = MessageT();
    fresh_ = false;
  }

private:
  mutable std::mutex mutex_;
  MessageT message_;
  bool fresh_{false};
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_ROS_IPC_BRIDGE_LATEST_SAMPLE_HPP
