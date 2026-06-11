#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string_view>
#include <sys/mman.h>
#include <thread>

#include <opencv2/core.hpp>

#include "autoaim_shm_image_transport/shared_memory_ring.hpp"

namespace autoaim_shm_image_transport
{

class AutoAimShmImagePublisher
{
public:
  explicit AutoAimShmImagePublisher(std::string_view shm_name, bool lock_memory = true)
  : region_(shm_name, true), ring_(region_.get())
  {
    if (lock_memory && ring_ != nullptr) {
      ::mlock(ring_, region_.size());
    }
  }

  ~AutoAimShmImagePublisher()
  {
    if (ring_ != nullptr) {
      ring_->shutdown_flag.store(1U, std::memory_order_release);
      autoaim_shm_futex_wake_all(&ring_->published_seq);
    }
  }

  void publish(const cv::Mat & image, uint64_t publish_time_ns = 0U)
  {
    if (image.empty()) {
      return;
    }
    if (image.type() != CV_8UC3) {
      throw std::runtime_error("AutoAimShmImagePublisher only supports CV_8UC3 images");
    }

    const auto row_bytes = static_cast<size_t>(image.cols) * image.elemSize();
    const auto bytes_used = static_cast<size_t>(image.rows) * row_bytes;
    if (bytes_used > AutoAimSharedRingBuffer::kImageBytes) {
      throw std::runtime_error("image size exceeds auto_aim shared memory slot capacity");
    }

    const uint32_t next_published =
      ring_->published_seq.load(std::memory_order_relaxed) + 1U;
    AutoAimSharedFrameSlot & slot =
      ring_->slots[next_published % AutoAimSharedRingBuffer::kSlotCount];

    uint32_t expected = 0U;
    while (!slot.reader_count.compare_exchange_weak(
        expected,
        AutoAimSharedRingBuffer::kWriterLocked,
        std::memory_order_acq_rel,
        std::memory_order_acquire))
    {
      expected = 0U;
      autoaim_shm_futex_wait(
        &ring_->published_seq,
        ring_->published_seq.load(std::memory_order_acquire));
      std::this_thread::yield();
    }

    slot.sequence = next_published;
    slot.publish_time_ns = publish_time_ns != 0U ? publish_time_ns : autoaim_shm_steady_time_ns();
    slot.width = static_cast<uint32_t>(image.cols);
    slot.height = static_cast<uint32_t>(image.rows);
    slot.step = static_cast<uint32_t>(row_bytes);
    slot.bytes_used = static_cast<uint32_t>(bytes_used);

    if (image.isContinuous() && image.step == row_bytes) {
      std::memcpy(slot.data.data(), image.data, bytes_used);
    } else {
      for (int row = 0; row < image.rows; ++row) {
        std::memcpy(
          slot.data.data() + static_cast<size_t>(row) * row_bytes,
          image.ptr(row),
          row_bytes);
      }
    }

    slot.reader_count.store(0U, std::memory_order_release);
    ring_->published_seq.store(next_published, std::memory_order_release);
    autoaim_shm_futex_wake_all(&ring_->published_seq);
    sequence_ = next_published;
  }

  AutoAimSharedRingBuffer * ring() const
  {
    return ring_;
  }

  uint64_t sequence() const
  {
    return sequence_;
  }

private:
  AutoAimSharedMemoryRegion region_;
  AutoAimSharedRingBuffer * ring_{nullptr};
  uint64_t sequence_{0U};
};

}  // namespace autoaim_shm_image_transport
