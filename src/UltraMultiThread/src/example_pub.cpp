#include <umt/umt.hpp>

#include <rclcpp/rclcpp.hpp>

namespace umt_example{

    class MSG {
    private:
        int id;
        std::string msg;

    public:
        MSG() = default;
        MSG(int _id, std::string _img): id(_id), msg(std::move(_img)) {};
        void print() {
            std::cout << "[LOGINFO] id = " << id << ", data = " << msg << '.' << std::endl;
        }
    };

    class example_pub: public rclcpp::Node{

    public:
        example_pub(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("example_pub_node", options){
            std::thread([]() {
                    umt::Publisher<MSG> pub("link_A");
                    int cnt = 0;
                    while(true) {
                        pub.push(MSG(cnt++, "Hey, bro!"));
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // It is recommended to delay in this way.
                    }
                }            
            ).detach();

        };
        ~example_pub(){};

    };

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(umt_example::example_pub)

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     auto image_pub_node = std::make_shared<umt_example::example_pub>();
//     rclcpp::spin(image_pub_node);
//     rclcpp::shutdown();
//     return 0;
// }