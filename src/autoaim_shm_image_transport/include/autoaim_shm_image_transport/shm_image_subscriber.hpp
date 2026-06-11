#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <sys/mman.h>
#include <thread>

#include <opencv2/core.hpp>

#include "autoaim_shm_image_transport/shared_memory_ring.hpp"

namespace autoaim_shm_image_transport
{

namespace detail
{

class AutoAimShmReadGuard
{
public:
  AutoAimShmReadGuard(AutoAimSharedRingBuffer * ring, AutoAimSharedFrameSlot * slot)
  : ring_(ring), slot_(slot)
  {
  }

  AutoAimShmReadGuard(const AutoAimShmReadGuard &) = delete;
  AutoAimShmReadGuard & operator=(const AutoAimShmReadGuard &) = delete;

  ~AutoAimShmReadGuard()
  {
    release();
  }

  void release()
  {
    if (slot_ == nullptr) {
      return;
    }
    const uint32_t previous = slot_->reader_count.fetch_sub(1U, std::memory_order_acq_rel);
    if (previous == 1U && ring_ != nullptr) {
      autoaim_shm_futex_wake_all(&ring_->published_seq);
    }
    slot_ = nullptr;
  }

private:
  AutoAimSharedRingBuffer * ring_{nullptr};
  AutoAimSharedFrameSlot * slot_{nullptr};
};

}  // namespace detail

struct AutoAimShmImageFrame
{
  cv::Mat image;
  std::shared_ptr<detail::AutoAimShmReadGuard> read_guard;
  uint64_t sequence{0U};
  uint64_t publish_time_ns{0U};
  uint32_t width{0U};
  uint32_t height{0U};
  bool valid{false};
  bool copied{false};
};

class AutoAimShmImageSubscriber
{
public:
  AutoAimShmImageSubscriber(
    std::string_view shm_name,
    int requested_reader_id = -1,
    bool lock_memory = true)
  : shm_name_(shm_name),
    requested_reader_id_(requested_reader_id),
    lock_memory_(lock_memory)
  {
  }

  ~AutoAimShmImageSubscriber()
  {
    stop();
  }

  bool attach()
  {
    while (!stop_.load(std::memory_order_relaxed)) {
      try {
        region_ = std::make_unique<AutoAimSharedMemoryRegion>(shm_name_, false);
        ring_ = region_->get();
        break;
      } catch (const std::runtime_error &) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }

    if (ring_ == nullptr) {
      return false;
    }

    if (!claim_reader_slot()) {
      return false;
    }

    if (lock_memory_) {
      ::mlock(ring_, region_->size());
    }

    return true;
  }

  AutoAimShmImageFrame wait_for_frame(bool copy_image)
  {
    AutoAimShmImageFrame result;
    if (stop_.load(std::memory_order_acquire) || ring_ == nullptr) {
      return result;
    }

    uint32_t current = ring_->published_seq.load(std::memory_order_acquire);
    if (current == 0U) {
      autoaim_shm_futex_wait(&ring_->published_seq, current);
      return result;
    }

    if (next_sequence_ == 0U) {
      next_sequence_ = current;
    }

    if (current < next_sequence_) {
      if (
        stop_.load(std::memory_order_acquire) ||
        ring_->shutdown_flag.load(std::memory_order_acquire) != 0U)
      {
        return result;
      }
      autoaim_shm_futex_wait(&ring_->published_seq, current);
      return result;
    }

    if ((current - next_sequence_) >= AutoAimSharedRingBuffer::kSlotCount) {
      next_sequence_ =
        current - static_cast<uint32_t>(AutoAimSharedRingBuffer::kSlotCount) + 1U;
    }

    AutoAimSharedFrameSlot & slot =
      ring_->slots[next_sequence_ % AutoAimSharedRingBuffer::kSlotCount];
    if (slot.sequence != next_sequence_) {
      const uint32_t latest = ring_->published_seq.load(std::memory_order_acquire);
      if (latest <= next_sequence_) {
        autoaim_shm_futex_wait(&ring_->published_seq, current);
      } else {
        next_sequence_ = latest;
      }
      return result;
    }

    uint32_t reader_count = slot.reader_count.load(std::memory_order_acquire);
    while (reader_count != AutoAimSharedRingBuffer::kWriterLocked) {
      if (slot.reader_count.compare_exchange_weak(
          reader_count,
          reader_count + 1U,
          std::memory_order_acq_rel,
          std::memory_order_acquire))
      {
        break;
      }
    }

    if (reader_count == AutoAimSharedRingBuffer::kWriterLocked) {
      autoaim_shm_futex_wait(&ring_->published_seq, current);
      return result;
    }

    auto guard = std::make_shared<detail::AutoAimShmReadGuard>(ring_, &slot);
    if (slot.sequence != next_sequence_) {
      guard->release();
      const uint32_t latest = ring_->published_seq.load(std::memory_order_acquire);
      if (latest > next_sequence_) {
        next_sequence_ = latest;
      }
      return result;
    }

    result.image = cv::Mat(
      static_cast<int>(slot.height),
      static_cast<int>(slot.width),
      CV_8UC3,
      slot.data.data(),
      slot.step);
    result.sequence = slot.sequence;
    result.publish_time_ns = slot.publish_time_ns;
    result.width = slot.width;
    result.height = slot.height;
    result.valid = true;

    if (copy_image) {
      result.image = result.image.clone();
      result.copied = true;
      guard->release();
    } else {
      result.read_guard = guard;
    }

    ++next_sequence_;
    return result;
  }

  void stop()
  {
    stop_.store(true, std::memory_order_release);
    if (ring_ != nullptr) {
      autoaim_shm_futex_wake_all(&ring_->published_seq);
    }
    if (ring_ != nullptr && reader_slot_ >= 0) {
      ring_->reader_active[reader_slot_].store(0U, std::memory_order_release);
      ring_->active_readers.fetch_sub(1U, std::memory_order_acq_rel);
      autoaim_shm_futex_wake_all(&ring_->published_seq);
      reader_slot_ = -1;
    }
  }

  bool is_shutdown() const
  {
    return ring_ != nullptr && ring_->shutdown_flag.load(std::memory_order_acquire) != 0U;
  }

  bool is_stopped() const
  {
    return stop_.load(std::memory_order_acquire);
  }

  AutoAimSharedRingBuffer * ring() const
  {
    return ring_;
  }

  int32_t reader_slot() const
  {
    return reader_slot_;
  }

private:
  bool claim_reader_slot()
  {
    if (requested_reader_id_ >= 0) {
      const uint32_t requested = static_cast<uint32_t>(requested_reader_id_);
      if (requested >= ring_->max_readers) {
        return false;
      }
      uint32_t expected = 0U;
      if (ring_->reader_active[requested].compare_exchange_strong(
          expected,
          1U,
          std::memory_order_acq_rel,
          std::memory_order_acquire))
      {
        reader_slot_ = static_cast<int32_t>(requested);
        ring_->active_readers.fetch_add(1U, std::memory_order_acq_rel);
      }
    } else {
      for (uint32_t i = 0; i < ring_->max_readers; ++i) {
        uint32_t expected = 0U;
        if (ring_->reader_active[i].compare_exchange_strong(
            expected,
            1U,
            std::memory_order_acq_rel,
            std::memory_order_acquire))
        {
          reader_slot_ = static_cast<int32_t>(i);
          ring_->active_readers.fetch_add(1U, std::memory_order_acq_rel);
          break;
        }
      }
    }

    return reader_slot_ >= 0;
  }

  std::string shm_name_;
  int requested_reader_id_;
  bool lock_memory_;
  std::unique_ptr<AutoAimSharedMemoryRegion> region_;
  AutoAimSharedRingBuffer * ring_{nullptr};
  int32_t reader_slot_{-1};
  uint32_t next_sequence_{0U};
  std::atomic<bool> stop_{false};
};

}  // namespace autoaim_shm_image_transport
