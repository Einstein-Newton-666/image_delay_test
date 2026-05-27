#include "image_test/image_pack.hpp"
#include "image_test/image_pub.hpp"
#include "shm_msgs/array_helper.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "iceoryx_posh/capro/service_description.hpp"

using namespace image_test;

image_pub::image_pub(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
: Node("img_pub_node", options)
{

    // Create camera publisher
    // rqt_image_view can't su1bscribe image msg with sensor_data QoS
    // https://github.com/ros-visualization/rqt/issues/187
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    img_pub_ = image_transport::create_publisher(this, "image_raw", qos);

    loaned_img_pub_ = this->create_publisher<shm_msgs::msg::Image8m>("image_raw2", rclcpp::SensorDataQoS());

    move_image = this->declare_parameter("move_image", false);

    int image_pub_frequency = this->declare_parameter("image_pub_frequency", 200);

    mode = this->declare_parameter("mode", 1);

    sender = std::make_shared<shm_video_trans::VideoSender>("image", 1920, 1024);

    pub = std::make_shared<umt::Publisher<ImagePack>>("image");

    switch (mode)
    {
    case 4:
        image_launcher = this->create_wall_timer(
            std::chrono::milliseconds(int(1000/image_pub_frequency)),
            std::bind(&image_pub::publish_image2,this)
        );
        break;
    case 5:
        try {
            iox::runtime::PoshRuntime::initRuntime("image_test");
        } catch (...) {}
        iceoryx_pub_ = std::make_unique<iox::popo::UntypedPublisher>(
            iox::capro::ServiceDescription("Image", "Test", "RawImage"));
        image = cv::Mat(1024, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
        image_launcher = this->create_wall_timer(
            std::chrono::milliseconds(int(1000/image_pub_frequency)),
            std::bind(&image_pub::publish_image_iceoryx,this)
        );
        break;
    default:
        image_launcher = this->create_wall_timer(
            std::chrono::milliseconds(int(1000/image_pub_frequency)),
            std::bind(&image_pub::publish_image1,this)
        );
        break;
    }


    RCLCPP_INFO(this->get_logger(), "Publishing image!");
}

image_pub::~image_pub()
{
    RCLCPP_INFO(this->get_logger(), "image_pub node destroyed!");
}

void image_pub::publish_image1(){
    // 【优化1】预分配 cv::Mat：避免每帧 5.9MB 的 malloc+memset 开销
    // 原始问题：每帧重建 cv::Mat(1024,1920,CV_8UC3,Scalar(0,0,0)) 触发堆分配+清零
    // 优化方式：首次分配后复用，后续帧只做 setTo(0) 清零（同为 memset 但省去 malloc）
    // 泛用性：实际相机场景中，图像来自相机回调，此处可直接接收相机 Mat，无需预分配
    if (image.empty()) {
        image = cv::Mat(1024, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
    } else {
        image.setTo(cv::Scalar(0, 0, 0));
    }
    auto time = rclcpp::Clock().now();
    auto std_time = std::chrono::steady_clock::now();
    switch (this->mode)
    {
    case 1:
        if(move_image){
            image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", std::move(image)).toImageMsg();
        }
        else{
            image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        }
        image_msg_->header.frame_id = "camera_optical_frame";
        image_msg_->header.stamp  = time;
        img_pub_.publish(image_msg_);
        break;
    case 2:
        sender->send(image, std_time);
            // RCLCPP_INFO_STREAM(this->get_logger(),"please set the mode to 1,2,3 or 4");
        break;
    case 3:
        // 【优化4】使用 move 语义：避免 ImagePack 构造时拷贝 5.9MB cv::Mat
        // 原始问题：ImagePack(image, time) 传 lvalue，触发 cv::Mat 深拷贝
        // 优化方式：std::move(image) 将 Mat 的所有权转移给 ImagePack，避免拷贝
        // 泛用性：move 后 image 变为空 Mat，下一帧由预分配逻辑重新创建
        //   实际相机场景中每帧本来就是新 Mat，move 无副作用
        pub->push(ImagePack(std::move(image), time));
        break;
    default:
        RCLCPP_INFO_STREAM(this->get_logger(),"error mode"+ std::to_string(mode) + ", please set the mode to 1,2 or 3");
        break;
    }
}

void image_pub::publish_image2(){
    // 【优化3】直接写入 loaned 消息：跳过 CvImage 中间层，消除 5.9MB memcpy
    // 原始问题：每帧 make_shared<CvImage> + new Mat + toImageMsg memcpy，共 3 次分配 + 1 次拷贝
    // 优化方式：先 borrow loaned 消息，再用 cv::Mat wrapper 直接引用 loaned 消息的共享内存 buffer
    //   图像数据直接写入共享内存，省去中间 Mat 分配和 memcpy
    // 泛用性：wrapper 的尺寸/类型由实际图像决定，此处使用测试用的 1920x1024 CV_8UC3
    //   实际相机场景中，用 camera_frame.copyTo(wrapper) 将相机数据直接写入共享内存
    auto loanedMsg = loaned_img_pub_->borrow_loaned_message();
    if (loanedMsg.is_valid()) {
        auto& msg = loanedMsg.get();

        // 设置消息头（使用 shm_msgs 辅助函数处理自定义 String 类型）
        msg.header.stamp = now();
        shm_msgs::set_str(msg.header.frame_id, "camera_optical_frame");
        msg.height = 1024;
        msg.width = 1920;
        shm_msgs::set_str(msg.encoding, "bgr8");
        msg.is_bigendian = 0;
        msg.step = 1920 * 3;

        // 直接在 loaned 消息的共享内存上创建 cv::Mat，避免中间 buffer 和 memcpy
        cv::Mat wrapper(1024, 1920, CV_8UC3, msg.data.data());
        wrapper.setTo(cv::Scalar(0, 0, 0));

        loaned_img_pub_->publish(std::move(loanedMsg));
    } else {
        RCLCPP_INFO(this->get_logger(), "Failed to get LoanMessage!");
    }
}

void image_pub::publish_image_iceoryx(){
    const size_t img_size = image.rows * image.step[0];
    const size_t total_size = sizeof(IceoryxImageHeader) + img_size;

    auto result = iceoryx_pub_->loan(total_size, alignof(IceoryxImageHeader));
    if (result.has_error()) {
        return;
    }
    auto* ptr = result.value();

    auto* header = reinterpret_cast<IceoryxImageHeader*>(ptr);
    auto now_steady = std::chrono::steady_clock::now();
    header->stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now_steady.time_since_epoch()).count();
    header->width = image.cols;
    header->height = image.rows;
    header->step = image.step[0];
    std::strncpy(header->encoding, "bgr8", sizeof(header->encoding) - 1);
    header->encoding[sizeof(header->encoding) - 1] = '\0';

    auto* img_data = reinterpret_cast<uint8_t*>(ptr) + sizeof(IceoryxImageHeader);
    std::memcpy(img_data, image.data, img_size);

    iceoryx_pub_->publish(ptr);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(image_test::image_pub)