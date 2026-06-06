#include <gtest/gtest.h>

#include "image_test/image_pub_utils.hpp"

#include <memory>

TEST(ImagePubUtils, WrapsImage8mDataWithoutStamping)
{
  auto msg = std::make_shared<shm_msgs::msg::Image8m>(
    rosidl_runtime_cpp::MessageInitialization::SKIP);
  std::fill(msg->data.begin(), msg->data.end(), 9);
  std::fill(msg->header.frame_id.data.begin(), msg->header.frame_id.data.end(), 'x');
  std::fill(msg->encoding.data.begin(), msg->encoding.data.end(), 'x');
  msg->header.stamp.sec = 12;
  msg->header.stamp.nanosec = 34;

  auto wrapper = image_test::prepare_image8m_payload(*msg);

  EXPECT_EQ(msg->width, image_test::kImageWidth);
  EXPECT_EQ(msg->height, image_test::kImageHeight);
  EXPECT_EQ(msg->step, image_test::kImageStep);
  EXPECT_EQ(msg->is_bigendian, 0);
  EXPECT_EQ(msg->header.stamp.sec, 12);
  EXPECT_EQ(msg->header.stamp.nanosec, 34U);
  EXPECT_EQ(shm_msgs::get_str(msg->header.frame_id), "camera_optical_frame");
  EXPECT_EQ(shm_msgs::get_str(msg->encoding), "bgr8");
  EXPECT_EQ(msg->header.frame_id.data[msg->header.frame_id.size], 0);
  EXPECT_EQ(msg->encoding.data[msg->encoding.size], 0);
  EXPECT_EQ(static_cast<void *>(wrapper.data), static_cast<void *>(msg->data.data()));
  EXPECT_EQ(wrapper.rows, static_cast<int>(image_test::kImageHeight));
  EXPECT_EQ(wrapper.cols, static_cast<int>(image_test::kImageWidth));

  wrapper.setTo(cv::Scalar(7, 7, 7));
  image_test::clear_image8m_unused_tail(*msg);

  EXPECT_EQ(msg->data[0], 7);
  EXPECT_EQ(msg->data[1], 7);
  EXPECT_EQ(msg->data[2], 7);
  EXPECT_EQ(msg->data[image_test::kImagePayloadSize - 1], 7);
  EXPECT_EQ(msg->data[image_test::kImagePayloadSize], 0);
  EXPECT_EQ(msg->data.back(), 0);
}
