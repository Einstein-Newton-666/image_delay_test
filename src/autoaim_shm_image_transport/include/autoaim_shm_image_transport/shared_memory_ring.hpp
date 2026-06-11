#pragma once

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/futex.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "autoaim_shm_image_transport/image_constants.hpp"

namespace autoaim_shm_image_transport
{

struct AutoAimSharedFrameSlot
{
  std::atomic<uint32_t> reader_count;
  uint32_t reserved1;
  uint64_t sequence;
  uint64_t publish_time_ns;
  uint32_t width;
  uint32_t height;
  uint32_t step;
  uint32_t bytes_used;
  std::array<uint8_t, kImagePayloadSize> data;
};

struct AutoAimSharedRingBuffer
{
  static constexpr uint32_t kMagic = 0x52494e47U;
  static constexpr uint32_t kVersion = 4U;
  static constexpr size_t kSlotCount = 1U;
  static constexpr size_t kImageBytes = kImagePayloadSize;
  static constexpr uint32_t kMaxReaders = 16U;
  static constexpr uint32_t kWriterLocked = UINT32_MAX;

  uint32_t magic;
  uint32_t version;
  uint32_t slot_count;
  uint32_t max_readers;
  std::atomic<uint32_t> published_seq;
  std::atomic<uint32_t> shutdown_flag;
  std::atomic<uint32_t> active_readers;
  uint32_t reserved0;
  std::atomic<uint32_t> reader_active[kMaxReaders];
  AutoAimSharedFrameSlot slots[kSlotCount];
};

inline uint64_t autoaim_shm_steady_time_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

inline int autoaim_shm_futex_wait(std::atomic<uint32_t> * addr, uint32_t expected)
{
  return static_cast<int>(::syscall(
    SYS_futex,
    reinterpret_cast<uint32_t *>(addr),
    FUTEX_WAIT,
    expected,
    nullptr,
    nullptr,
    0));
}

inline int autoaim_shm_futex_wake_all(std::atomic<uint32_t> * addr)
{
  return static_cast<int>(::syscall(
    SYS_futex,
    reinterpret_cast<uint32_t *>(addr),
    FUTEX_WAKE,
    INT_MAX,
    nullptr,
    nullptr,
    0));
}

class AutoAimSharedMemoryRegion
{
public:
  AutoAimSharedMemoryRegion(std::string_view name, bool create)
  : name_(name), owner_(create)
  {
    const int flags = create ? (O_CREAT | O_RDWR) : O_RDWR;
    fd_ = ::shm_open(name_.c_str(), flags, 0666);
    if (fd_ < 0) {
      throw std::runtime_error("shm_open failed for " + name_ + ": " + std::strerror(errno));
    }

    if (create && ::ftruncate(fd_, sizeof(AutoAimSharedRingBuffer)) != 0) {
      const std::string error = std::strerror(errno);
      cleanup();
      throw std::runtime_error("ftruncate failed for " + name_ + ": " + error);
    }

    void * mapped = ::mmap(
      nullptr,
      sizeof(AutoAimSharedRingBuffer),
      PROT_READ | PROT_WRITE,
      MAP_SHARED,
      fd_,
      0);
    if (mapped == MAP_FAILED) {
      const std::string error = std::strerror(errno);
      cleanup();
      throw std::runtime_error("mmap failed for " + name_ + ": " + error);
    }

    ring_ = static_cast<AutoAimSharedRingBuffer *>(mapped);
    if (create) {
      initialize_ring();
    } else if (
      ring_->magic != AutoAimSharedRingBuffer::kMagic ||
      ring_->version != AutoAimSharedRingBuffer::kVersion ||
      ring_->slot_count != AutoAimSharedRingBuffer::kSlotCount ||
      ring_->max_readers != AutoAimSharedRingBuffer::kMaxReaders)
    {
      cleanup();
      throw std::runtime_error("shared memory header mismatch for " + name_);
    }
  }

  AutoAimSharedMemoryRegion(const AutoAimSharedMemoryRegion &) = delete;
  AutoAimSharedMemoryRegion & operator=(const AutoAimSharedMemoryRegion &) = delete;

  ~AutoAimSharedMemoryRegion()
  {
    cleanup();
  }

  AutoAimSharedRingBuffer * get() const
  {
    return ring_;
  }

  size_t size() const
  {
    return sizeof(AutoAimSharedRingBuffer);
  }

private:
  void initialize_ring()
  {
    ring_->magic = AutoAimSharedRingBuffer::kMagic;
    ring_->version = AutoAimSharedRingBuffer::kVersion;
    ring_->slot_count = AutoAimSharedRingBuffer::kSlotCount;
    ring_->max_readers = AutoAimSharedRingBuffer::kMaxReaders;
    ring_->published_seq.store(0U, std::memory_order_relaxed);
    ring_->shutdown_flag.store(0U, std::memory_order_relaxed);
    ring_->active_readers.store(0U, std::memory_order_relaxed);
    ring_->reserved0 = 0U;
    for (size_t i = 0; i < AutoAimSharedRingBuffer::kMaxReaders; ++i) {
      ring_->reader_active[i].store(0U, std::memory_order_relaxed);
    }
    for (size_t i = 0; i < AutoAimSharedRingBuffer::kSlotCount; ++i) {
      auto & slot = ring_->slots[i];
      slot.reader_count.store(0U, std::memory_order_relaxed);
      slot.reserved1 = 0U;
      slot.sequence = 0U;
      slot.publish_time_ns = 0U;
      slot.width = 0U;
      slot.height = 0U;
      slot.step = 0U;
      slot.bytes_used = 0U;
    }
  }

  void cleanup()
  {
    if (ring_ != nullptr) {
      ::munmap(ring_, sizeof(AutoAimSharedRingBuffer));
      ring_ = nullptr;
    }
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    if (owner_) {
      ::shm_unlink(name_.c_str());
      owner_ = false;
    }
  }

  std::string name_;
  bool owner_{false};
  int fd_{-1};
  AutoAimSharedRingBuffer * ring_{nullptr};
};

}  // namespace autoaim_shm_image_transport
