#pragma once

#include <cstddef>
#include <cstdint>

namespace autoaim_shm_image_transport
{

constexpr uint32_t kImageWidth = 1920;
constexpr uint32_t kImageHeight = 1024;
constexpr uint32_t kImageChannels = 3;
constexpr uint32_t kImageStep = kImageWidth * kImageChannels;
constexpr size_t kImagePayloadSize = static_cast<size_t>(kImageHeight) * kImageStep;

}  // namespace autoaim_shm_image_transport
