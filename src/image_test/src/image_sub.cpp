#include "image_test/image_sub.hpp"
#include "image_test/image_pack.hpp"
#include "image_test/image_pub.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "iceoryx_posh/capro/service_description.hpp"

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
    case 4:
        shm_img_sub_ =  this->create_subscription<shm_msgs::msg::Image8m>(
            "/image_raw2", rclcpp::SensorDataQoS().keep_last(queue_size),
            std::bind(&image_sub::shmImageCallback, this, std::placeholders::_1));

        break;
    case 5:
    {
        try {
            iox::runtime::PoshRuntime::initRuntime("image_test");
        } catch (...) {}
        std::thread([this]() {
            iox::popo::UntypedSubscriber sub(
                iox::capro::ServiceDescription("Image", "Test", "RawImage"));
            while (true) {
                auto take_result = sub.take();
                if (take_result.has_error()) {
                    std::this_thread::sleep_for(std::chrono::microseconds(100));
                    continue;
                }
                const auto* ptr = take_result.value();
                const auto* header = reinterpret_cast<const IceoryxImageHeader*>(ptr);

                // 获取可用的 cv::Mat（零拷贝，直接引用共享内存）
                const auto* img_data = reinterpret_cast<const uint8_t*>(ptr) + sizeof(IceoryxImageHeader);
                cv::Mat received_image(header->height, header->width, CV_8UC3, const_cast<uint8_t*>(img_data));

                // 时间戳：获取到可用 cv::Mat 之后的时刻
                auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                double latency_ms = (now_ns - header->stamp_ns) / 1e6;

                RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(latency_ms) + "ms");

                sub.release(ptr);
            }
        }).detach();
        break;
    }
    default:
        break;
    }
}

image_sub::~image_sub(){

    RCLCPP_INFO(this->get_logger(), "Stop image_sub");
}

void image_sub::imageCallback1(const sensor_msgs::msg::Image::ConstSharedPtr img_msg){
    // 【优化2】修复编码不匹配：避免 toCvShare 因编码不同退化为 toCvCopy + 颜色转换
    // 原始问题：toCvShare(img_msg, "rgb8") 请求 rgb8，但发布端发 bgr8，编码不匹配触发 cvtColor 拷贝
    // 优化方式：不指定编码，直接共享原始数据，消除 5.9MB 颜色转换拷贝
    // 泛用性：不指定编码时 toCvShare 直接共享消息 buffer，适用于任意编码格式
    //   如需特定编码转换，在业务代码中按需调用 cv::cvtColor
    auto img = cv_bridge::toCvShare(img_msg)->image;
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

void image_sub::shmImageCallback(const shm_msgs::msg::Image8m::SharedPtr img_msg){
    auto image = shm_msgs::toCvShare(img_msg);
    auto t1 =this->now();
    auto latency = (t1 - img_msg->header.stamp).seconds() * 1000;
    RCLCPP_INFO_STREAM(this->get_logger(), std::to_string(latency) + "ms");
}



#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_test::image_sub)
