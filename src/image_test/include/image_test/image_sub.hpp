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

#include "image_test/image_pack.hpp"
#include "shm_video_transmission/shm_video_transmission.h"
#include <umt/umt.hpp>

namespace image_test{
    class image_sub: public rclcpp::Node{

    public:
        image_sub(const rclcpp::NodeOptions & options);
        ~image_sub();

    private:

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

        std::shared_ptr<shm_video_trans::VideoReceiver> receiver;

        std::shared_ptr<umt::Subscriber<ImagePack>> sub;

        shm_video_trans::FrameBag receivedFrame;

        void imageCallback1(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

        void imageCallback2(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

    };

}
