#pragma once

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <cv_bridge/cv_bridge.h>

#include <atomic>
#include <thread>
#include <vector>

#include "autoaim_shm_image_transport/autoaim_shm_image_transport.hpp"
#include "image_test/image_pack.hpp"
#include "shm_video_transmission/shm_video_transmission.h"
#include <umt/umt.hpp>
#include "shm_msgs/msg/image.hpp"
#include "shm_msgs/opencv_conversions.hpp"

#include "iceoryx_posh/popo/untyped_subscriber.hpp"

struct IceoryxImageHeader;

namespace image_test{
    class image_sub: public rclcpp::Node{

    public:
        image_sub(const rclcpp::NodeOptions & options);
        ~image_sub();

    private:

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

        std::shared_ptr<shm_video_trans::VideoReceiver> receiver;

        std::shared_ptr<umt::Subscriber<ImagePack>> sub;

        rclcpp::Subscription<shm_msgs::msg::Image8m>::SharedPtr shm_img_sub_;

        shm_video_trans::FrameBag receivedFrame;

        std::atomic_bool running_{true};

        std::vector<std::thread> worker_threads_;

        void startShmVideoReceiver(bool copy_image);

        void startUmtReceiver();

        void startIceoryxReceiver();

        void startAutoAimShmReceiver(bool copy_image);

        void imageCallback1(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

        void imageCallback2(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

        void uniqueImageCallback(sensor_msgs::msg::Image::UniquePtr img_msg);

        void shmImageCallback(const shm_msgs::msg::Image8m::SharedPtr img_msg);

        std::unique_ptr<autoaim_shm_image_transport::AutoAimShmImageSubscriber> autoaim_shm_subscriber_;
    };

}
