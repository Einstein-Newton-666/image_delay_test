#include "image_test/image_pack.hpp"
#include "image_test/image_pub.hpp"

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

    move_image = this->declare_parameter("move_image", false);

    int image_pub_frequency = this->declare_parameter("image_pub_frequency", 200);

    mode = this->declare_parameter("mode", 1);

    sender = std::make_shared<shm_video_trans::VideoSender>("image", 1920, 1024);

    pub = std::make_shared<umt::Publisher<ImagePack>>("image");

    image_launcher = this->create_wall_timer(
        std::chrono::milliseconds(int(1000/image_pub_frequency)), 
        std::bind(&image_pub::publish_image,this)
    );

    RCLCPP_INFO(this->get_logger(), "Publishing image!");
}

image_pub::~image_pub()
{
    RCLCPP_INFO(this->get_logger(), "image_pub node destroyed!");
}

void image_pub::publish_image(){
    image = cv::Mat(1024, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
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
            // RCLCPP_INFO_STREAM(this->get_logger(),"please set the mode to 1,2 or 3");
        break;
    case 3:
        pub->push(ImagePack(image,time));
        break;
    default:
        RCLCPP_INFO_STREAM(this->get_logger(),"error mode"+ std::to_string(mode) + ", please set the mode to 1,2 or 3");
        break;
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(image_test::image_pub)