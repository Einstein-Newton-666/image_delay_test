#pragma once 

#include <opencv2/opencv.hpp>

// ROS
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/duration.hpp>

#include "shm_video_transmission/shm_video_transmission.h"
#include <umt/umt.hpp>
#include "shm_msgs/msg/image.hpp"
#include "shm_msgs/opencv_conversions.hpp"

namespace image_test{
    class image_pub : public rclcpp::Node
    {

    public:
        explicit image_pub(const rclcpp::NodeOptions & options);

        ~image_pub();

        void publish_image1();

        void publish_image2();

    private:    
        sensor_msgs::msg::Image::SharedPtr image_msg_;

        image_transport::Publisher img_pub_;

        rclcpp::Publisher<shm_msgs::msg::Image8m>::SharedPtr loaned_img_pub_;

        rclcpp::TimerBase::SharedPtr image_launcher;

        cv::Mat image;

        std::shared_ptr<shm_msgs::CvImage> shm_image;

        int mode;
        
        std::shared_ptr<shm_video_trans::VideoSender> sender;

        std::shared_ptr<umt::Publisher<ImagePack>> pub;

        bool move_image;

    };
}
