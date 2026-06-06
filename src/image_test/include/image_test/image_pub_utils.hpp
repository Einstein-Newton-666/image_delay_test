#pragma once

#include <opencv2/core.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>

#include "shm_msgs/array_helper.hpp"
#include "shm_msgs/msg/image.hpp"

namespace image_test
{

constexpr uint32_t kImageWidth = 1920;
constexpr uint32_t kImageHeight = 1024;
constexpr uint32_t kImageChannels = 3;
constexpr uint32_t kImageStep = kImageWidth * kImageChannels;
constexpr size_t kImagePayloadSize = static_cast<size_t>(kImageHeight) * kImageStep;

inline void set_fixed_string(shm_msgs::msg::String & msg, const std::string & value)
{
  msg.data.fill(0);
  shm_msgs::set_str(msg, value);
}

inline cv::Mat prepare_image8m_payload(shm_msgs::msg::Image8m & msg)
{
  msg.height = kImageHeight;
  msg.width = kImageWidth;
  set_fixed_string(msg.header.frame_id, "camera_optical_frame");
  set_fixed_string(msg.encoding, "bgr8");
  msg.is_bigendian = 0;
  msg.step = kImageStep;

  return cv::Mat(kImageHeight, kImageWidth, CV_8UC3, msg.data.data());
}

inline void clear_image8m_unused_tail(shm_msgs::msg::Image8m & msg)
{
  static_assert(kImagePayloadSize <= shm_msgs::msg::Image8m::DATA_MAX_SIZE);
  std::fill(msg.data.begin() + kImagePayloadSize, msg.data.end(), 0);
}

}  // namespace image_test
