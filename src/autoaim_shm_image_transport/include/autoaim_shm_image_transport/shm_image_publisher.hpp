#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string_view>
#include <sys/mman.h>
#include <thread>
#include <utility>

#include <opencv2/core.hpp>

#include "autoaim_shm_image_transport/shared_memory_ring.hpp"

namespace autoaim_shm_image_transport
{

class AutoAimShmWritableFrame
{
public:
  AutoAimShmWritableFrame() = default;

  AutoAimShmWritableFrame(
    AutoAimSharedRingBuffer * ring,
    AutoAimSharedFrameSlot * slot,
    uint32_t sequence,
    cv::Mat image)
  : ring_(ring), slot_(slot), sequence_(sequence), image_(std::move(image))
  {
  }

  AutoAimShmWritableFrame(const AutoAimShmWritableFrame &) = delete;
  AutoAimShmWritableFrame & operator=(const AutoAimShmWritableFrame &) = delete;

  AutoAimShmWritableFrame(AutoAimShmWritableFrame && other) noexcept
  {
    move_from(other);
  }

  AutoAimShmWritableFrame & operator=(AutoAimShmWritableFrame && other) noexcept
  {
    if (this != &other) {
      cancel();
      move_from(other);
    }
    return *this;
  }

  ~AutoAimShmWritableFrame()
  {
    cancel();
  }

  bool valid() const
  {
    return ring_ != nullptr && slot_ != nullptr && !committed_;
  }

  cv::Mat & image()
  {
    return image_;
  }

  const cv::Mat & image() const
  {
    return image_;
  }

  uint32_t sequence() const
  {
    return sequence_;
  }

  void commit(uint64_t publish_time_ns = 0U)
  {
    if (!valid()) {
      return;
    }

    slot_->publish_time_ns =
      publish_time_ns != 0U ? publish_time_ns : autoaim_shm_steady_time_ns();
    slot_->reader_count.store(0U, std::memory_order_release);
    ring_->published_seq.store(sequence_, std::memory_order_release);
    autoaim_shm_futex_wake_all(&ring_->published_seq);
    committed_ = true;
    ring_ = nullptr;
    slot_ = nullptr;
  }

  void cancel()
  {
    if (slot_ != nullptr && !committed_) {
      slot_->reader_count.store(0U, std::memory_order_release);
      if (ring_ != nullptr) {
        autoaim_shm_futex_wake_all(&ring_->published_seq);
      }
    }
    ring_ = nullptr;
    slot_ = nullptr;
    image_.release();
  }

private:
  void move_from(AutoAimShmWritableFrame & other)
  {
    ring_ = other.ring_;
    slot_ = other.slot_;
    sequence_ = other.sequence_;
    image_ = std::move(other.image_);
    committed_ = other.committed_;
    other.ring_ = nullptr;
    other.slot_ = nullptr;
    other.sequence_ = 0U;
    other.committed_ = true;
  }

  AutoAimSharedRingBuffer * ring_{nullptr};
  AutoAimSharedFrameSlot * slot_{nullptr};
  uint32_t sequence_{0U};
  cv::Mat image_;
  bool committed_{false};
};

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

  AutoAimShmWritableFrame borrow_frame(
    uint32_t width = kImageWidth,
    uint32_t height = kImageHeight,
    int cv_type = CV_8UC3)
  {
    if (cv_type != CV_8UC3) {
      throw std::runtime_error("AutoAimShmImagePublisher only supports CV_8UC3 images");
    }

    const auto row_bytes = static_cast<size_t>(width) * kImageChannels;
    const auto bytes_used = static_cast<size_t>(height) * row_bytes;
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
    slot.publish_time_ns = 0U;
    slot.width = width;
    slot.height = height;
    slot.step = static_cast<uint32_t>(row_bytes);
    slot.bytes_used = static_cast<uint32_t>(bytes_used);

    cv::Mat wrapper(
      static_cast<int>(height),
      static_cast<int>(width),
      cv_type,
      slot.data.data(),
      slot.step);

    return AutoAimShmWritableFrame(ring_, &slot, next_published, wrapper);
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
    auto frame = borrow_frame(
      static_cast<uint32_t>(image.cols),
      static_cast<uint32_t>(image.rows),
      image.type());

    if (image.isContinuous() && image.step == row_bytes) {
      std::memcpy(frame.image().data, image.data, bytes_used);
    } else {
      for (int row = 0; row < image.rows; ++row) {
        std::memcpy(
          frame.image().data + static_cast<size_t>(row) * row_bytes,
          image.ptr(row),
          row_bytes);
      }
    }

    frame.commit(publish_time_ns);
    sequence_ = frame.sequence();
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
