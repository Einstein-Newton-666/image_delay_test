#pragma once

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

struct ImagePack{
    cv::Mat image;
    builtin_interfaces::msg::Time time;
    ImagePack() = default;
    ImagePack(cv::Mat _image, rclcpp::Time _time): image(std::move(_image)), time(_time) {};
};