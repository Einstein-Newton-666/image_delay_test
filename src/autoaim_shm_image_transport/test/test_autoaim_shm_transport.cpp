#include <gtest/gtest.h>

#include "autoaim_shm_image_transport/autoaim_shm_image_transport.hpp"

#include <opencv2/core.hpp>

#include <cstdio>
#include <string>
#include <unistd.h>

namespace
{

std::string unique_shm_name(const char * suffix)
{
  return std::string("/image_test_autoaim_shm_") + std::to_string(::getpid()) + "_" + suffix;
}

}  // namespace

TEST(AutoAimShmTransport, PublishesAndCopiesFrameAcrossSharedMemory)
{
  const auto shm_name = unique_shm_name("copy");
  autoaim_shm_image_transport::AutoAimShmImagePublisher publisher(shm_name, false);
  autoaim_shm_image_transport::AutoAimShmImageSubscriber subscriber(shm_name, -1, false);
  ASSERT_TRUE(subscriber.attach());

  cv::Mat source(2, 3, CV_8UC3, cv::Scalar(1, 2, 3));
  source.at<cv::Vec3b>(1, 2) = cv::Vec3b(7, 8, 9);

  publisher.publish(source, 123456789ULL);
  auto frame = subscriber.wait_for_frame(true);

  ASSERT_TRUE(frame.valid);
  EXPECT_TRUE(frame.copied);
  EXPECT_EQ(frame.sequence, 1ULL);
  EXPECT_EQ(frame.publish_time_ns, 123456789ULL);
  EXPECT_EQ(frame.width, 3U);
  EXPECT_EQ(frame.height, 2U);
  EXPECT_EQ(frame.image.rows, 2);
  EXPECT_EQ(frame.image.cols, 3);
  EXPECT_NE(frame.image.data, subscriber.ring()->slots[0].data.data());
  EXPECT_EQ(frame.image.at<cv::Vec3b>(1, 2), cv::Vec3b(7, 8, 9));
  EXPECT_EQ(publisher.ring()->slots[0].reader_count.load(std::memory_order_acquire), 0U);
}

TEST(AutoAimShmTransport, ZeroCopyFrameHoldsReadGuardUntilFrameIsDestroyed)
{
  const auto shm_name = unique_shm_name("share");
  autoaim_shm_image_transport::AutoAimShmImagePublisher publisher(shm_name, false);
  autoaim_shm_image_transport::AutoAimShmImageSubscriber subscriber(shm_name, -1, false);
  ASSERT_TRUE(subscriber.attach());

  cv::Mat source(2, 3, CV_8UC3, cv::Scalar(4, 5, 6));
  publisher.publish(source, 42ULL);

  {
    auto frame = subscriber.wait_for_frame(false);

    ASSERT_TRUE(frame.valid);
    EXPECT_FALSE(frame.copied);
    EXPECT_EQ(frame.image.data, subscriber.ring()->slots[0].data.data());
    EXPECT_EQ(frame.image.at<cv::Vec3b>(0, 0), cv::Vec3b(4, 5, 6));
    EXPECT_EQ(publisher.ring()->slots[0].reader_count.load(std::memory_order_acquire), 1U);
  }

  EXPECT_EQ(publisher.ring()->slots[0].reader_count.load(std::memory_order_acquire), 0U);
}
