#include "image_test/image_sub.hpp"
#include "image_test/image_pack.hpp"

using namespace image_test;

image_sub::image_sub(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()):
Node("image_sub_node",options)
{   
    auto copy_image = this->declare_parameter("copy_image", false);
    auto queue_size = this->declare_parameter("queue_size", 5);
    int mode = this->declare_parameter("mode", 1);
    switch (mode)
    {
    case 1:
        if(copy_image){
            img_sub_ =  this->create_subscription<sensor_msgs::msg::Image>(
                "/image_raw", rclcpp::SensorDataQoS().keep_last(queue_size),
                std::bind(&image_sub::imageCallback2, this, std::placeholders::_1));
        } else {
            img_sub_ =  this->create_subscription<sensor_msgs::msg::Image>(
                "/image_raw", rclcpp::SensorDataQoS().keep_last(queue_size),
                std::bind(&image_sub::imageCallback1, this, std::placeholders::_1));
        }
        break;
    case 2:
        receiver = std::make_shared<shm_video_trans::VideoReceiver>("image");
        std::thread([this, &copy_image]() {
                while (!receiver->init())
                {
                    std::cout << "[WARNING] pub image not ready." << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                while(true){
                    if(this->receiver->receive()){
                        receiver->lock();
                        if(!copy_image){
                            receivedFrame = receiver->toCvShare();
                        }
                        else {
                            receivedFrame = receiver->toCvCopy();
                        }
                        receiver->unlock();
                        auto now_time = std::chrono::steady_clock::now();
                        auto latency = std::chrono::duration_cast<std::chrono::nanoseconds>(now_time - receivedFrame.time_stamp).count();
                        RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(latency / 1e6) + "ms");
                    }
                }
            }
        ).detach();
        break;
    case 3:
        sub = std::make_shared<umt::Subscriber<ImagePack>>("image",queue_size);
        std::thread([this](){
                ImagePack image_pack;
                while (true) {
                    try {
                        image_pack = this->sub->pop();

                    } catch(...) {
                        std::cout << "[WARNING] pub image not ready." << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        continue;
                    }
                    auto t1 =this->now();
                    auto latency = (t1 - image_pack.time).seconds() * 1000;
                    RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(latency) + "ms");
                }
            }            
        ).detach();
        break;
    default:
        break;
    }
}

image_sub::~image_sub(){

    RCLCPP_INFO(this->get_logger(), "Stop image_sub");
}

void image_sub::imageCallback1(const sensor_msgs::msg::Image::ConstSharedPtr img_msg){
    auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
    auto t1 =this->now();
    auto latency = (t1 - img_msg->header.stamp).seconds() * 1000;
    RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(latency) + "ms");
}

void image_sub::imageCallback2(const sensor_msgs::msg::Image::ConstSharedPtr img_msg){
    auto image = cv_bridge::toCvCopy(img_msg)->image;
    auto t1 =this->now();
    auto latency = (t1 - img_msg->header.stamp).seconds() * 1000;
    RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(latency) + "ms");
}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_test::image_sub)
