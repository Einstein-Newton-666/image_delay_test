#include "image_test/image_pack.hpp"
#include "image_test/image_pub.hpp"
#include "image_test/image_pub_utils.hpp"
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
    // true: create the test image directly in the transport-owned buffer.
    // false: create a normal cv::Mat first, then copy it into the transport buffer.
    generate_in_transport_buffer_ = this->declare_parameter("generate_in_transport_buffer", true);
    const auto autoaim_shm_name = this->declare_parameter(
        "autoaim_shm_name", std::string("/image_test_autoaim_shm_ring"));
    const bool autoaim_shm_lock_memory = this->declare_parameter(
        "autoaim_shm_lock_memory", true);

    int image_pub_frequency = this->declare_parameter("image_pub_frequency", 200);

    mode = this->declare_parameter("mode", 1);

    sender = std::make_shared<shm_video_trans::VideoSender>("image", 1920, 1024);

    pub = std::make_shared<umt::Publisher<ImagePack>>("image");

    can_loan_image_msg_ = loaned_img_pub_->can_loan_messages();

    switch (mode)
    {
    case 4:
        RCLCPP_INFO(
            this->get_logger(),
            "mode 4 loaned image messages: %s",
            can_loan_image_msg_ ? "enabled" : "unsupported, using reusable local message");
        if (!can_loan_image_msg_) {
            reusable_image_msg_ = std::make_shared<shm_msgs::msg::Image8m>(
                rosidl_runtime_cpp::MessageInitialization::SKIP);
            clear_image8m_unused_tail(*reusable_image_msg_);
        }
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
        image_launcher = this->create_wall_timer(
            std::chrono::milliseconds(int(1000/image_pub_frequency)),
            std::bind(&image_pub::publish_image_iceoryx,this)
        );
        break;
    case 6:
        raw_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "image_raw_unique", rclcpp::SensorDataQoS());
        image_launcher = this->create_wall_timer(
            std::chrono::milliseconds(int(1000/image_pub_frequency)),
            std::bind(&image_pub::publish_image_unique,this)
        );
        break;
    case 7:
        autoaim_shm_publisher_ =
            std::make_unique<autoaim_shm_image_transport::AutoAimShmImagePublisher>(
            autoaim_shm_name, autoaim_shm_lock_memory);
        image_launcher = this->create_wall_timer(
            std::chrono::milliseconds(int(1000/image_pub_frequency)),
            std::bind(&image_pub::publish_image_autoaim_shm,this)
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

    const auto image_ready_time = this->now();
    const auto image_ready_steady = std::chrono::steady_clock::now();

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
        image_msg_->header.stamp  = image_ready_time;
        img_pub_.publish(image_msg_);
        break;
    case 2:
        sender->send(image, image_ready_steady);
            // RCLCPP_INFO_STREAM(this->get_logger(),"please set the mode to 1,2,3 or 4");
        break;
    case 3:
        // 【优化4】使用 move 语义：避免 ImagePack 构造时拷贝 5.9MB cv::Mat
        // 原始问题：ImagePack(image, time) 传 lvalue，触发 cv::Mat 深拷贝
        // 优化方式：std::move(image) 将 Mat 的所有权转移给 ImagePack，避免拷贝
        // 泛用性：move 后 image 变为空 Mat，下一帧由预分配逻辑重新创建
        //   实际相机场景中每帧本来就是新 Mat，move 无副作用
        pub->push(ImagePack(std::move(image), image_ready_time));
        break;
    default:
        RCLCPP_INFO_STREAM(this->get_logger(),"error mode"+ std::to_string(mode) + ", please set the mode to 1,2 or 3");
        break;
    }
}

void image_pub::publish_image2(){
    rclcpp::Time image_ready_time;
    if (!generate_in_transport_buffer_) {
        // Copy benchmark path: the source Mat is the start point.  The measured
        // latency intentionally includes borrowing/loaning and copyTo().
        if (image.empty()) {
            image = cv::Mat(kImageHeight, kImageWidth, CV_8UC3, cv::Scalar(0, 0, 0));
        } else {
            image.setTo(cv::Scalar(0, 0, 0));
        }
        image_ready_time = now();
    }

    // 只有 RMW 真正支持 loan 时才借中间件内存；否则复用本地大消息，避免每帧 fallback 构造时清零 8MB。
    if (can_loan_image_msg_) {
        auto loanedMsg = loaned_img_pub_->borrow_loaned_message();
        if (!loanedMsg.is_valid()) {
            RCLCPP_INFO(this->get_logger(), "Failed to get LoanMessage!");
            return;
        }
        auto& msg = loanedMsg.get();
        cv::Mat wrapper = prepare_image8m_payload(msg);

        if (generate_in_transport_buffer_) {
            // Direct path: wrapper is already the loaned payload, so the
            // timestamp is taken after the target cv::Mat becomes usable.
            wrapper.setTo(cv::Scalar(0, 0, 0));
            image_ready_time = now();
        } else {
            image.copyTo(wrapper);
        }
        clear_image8m_unused_tail(msg);

        msg.header.stamp = image_ready_time;
        loaned_img_pub_->publish(std::move(loanedMsg));
        return;
    }

    auto& msg = *reusable_image_msg_;
    cv::Mat wrapper = prepare_image8m_payload(msg);

    if (generate_in_transport_buffer_) {
        // Fallback still writes directly into the reusable message buffer; it
        // is not a true RMW loan, but keeps the same timing semantics.
        wrapper.setTo(cv::Scalar(0, 0, 0));
        image_ready_time = now();
    } else {
        image.copyTo(wrapper);
    }
    msg.header.stamp = image_ready_time;
    loaned_img_pub_->publish(msg);
}

void image_pub::publish_image_iceoryx(){
    std::chrono::steady_clock::time_point image_ready_steady;
    if (!generate_in_transport_buffer_) {
        // Copy benchmark path: source Mat is ready before the iceoryx loan, so
        // latency includes loan(), copyTo(), publish(), take(), and wrapping.
        if (image.empty()) {
            image = cv::Mat(kImageHeight, kImageWidth, CV_8UC3, cv::Scalar(0, 0, 0));
        } else {
            image.setTo(cv::Scalar(0, 0, 0));
        }
        image_ready_steady = std::chrono::steady_clock::now();
    }

    const size_t img_size = kImagePayloadSize;
    const size_t total_size = sizeof(IceoryxImageHeader) + img_size;

    auto result = iceoryx_pub_->loan(total_size, alignof(IceoryxImageHeader));
    if (result.has_error()) {
        return;
    }
    auto* ptr = result.value();

    // 写入头信息
    auto* header = reinterpret_cast<IceoryxImageHeader*>(ptr);
    header->width = kImageWidth;
    header->height = kImageHeight;
    header->step = kImageStep;
    std::strncpy(header->encoding, "bgr8", sizeof(header->encoding) - 1);
    header->encoding[sizeof(header->encoding) - 1] = '\0';

    auto* img_data = reinterpret_cast<uint8_t*>(ptr) + sizeof(IceoryxImageHeader);
    cv::Mat wrapper(kImageHeight, kImageWidth, CV_8UC3, img_data);

    if (generate_in_transport_buffer_) {
        // Direct path: the shared memory chunk itself backs the cv::Mat.
        // Timestamp after fill so the metric starts at "publishable Mat ready".
        wrapper.setTo(cv::Scalar(0, 0, 0));
        image_ready_steady = std::chrono::steady_clock::now();
    } else {
        image.copyTo(wrapper);
    }
    header->stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        image_ready_steady.time_since_epoch()).count();

    iceoryx_pub_->publish(ptr);
}

void image_pub::publish_image_unique(){
    rclcpp::Time image_ready_time;
    if (!generate_in_transport_buffer_) {
        if (image.empty()) {
            image = cv::Mat(kImageHeight, kImageWidth, CV_8UC3, cv::Scalar(0, 0, 0));
        } else {
            image.setTo(cv::Scalar(0, 0, 0));
        }
        image_ready_time = this->now();
    }

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->height = kImageHeight;
    msg->width = kImageWidth;
    msg->encoding = "bgr8";
    msg->is_bigendian = 0;
    msg->step = kImageStep;
    msg->data.resize(kImagePayloadSize);

    // rclcpp intra-process can transfer this unique_ptr without converting it
    // through image_transport. The payload still belongs to the ROS message.
    cv::Mat wrapper(kImageHeight, kImageWidth, CV_8UC3, msg->data.data(), msg->step);
    if (generate_in_transport_buffer_) {
        wrapper.setTo(cv::Scalar(0, 0, 0));
        image_ready_time = this->now();
    } else {
        image.copyTo(wrapper);
    }

    msg->header.frame_id = "camera_optical_frame";
    msg->header.stamp = image_ready_time;

    raw_img_pub_->publish(std::move(msg));
}

void image_pub::publish_image_autoaim_shm(){
    if (image.empty()) {
        image = cv::Mat(kImageHeight, kImageWidth, CV_8UC3, cv::Scalar(0, 0, 0));
    } else {
        image.setTo(cv::Scalar(0, 0, 0));
    }

    const auto image_ready_steady_ns =
        autoaim_shm_image_transport::autoaim_shm_steady_time_ns();
    autoaim_shm_publisher_->publish(image, image_ready_steady_ns);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(image_test::image_pub)
