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

    class example_sub: public rclcpp::Node{

    public:
        example_sub(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("example_sub_node", options){
            std::thread([](){
                    umt::Subscriber<MSG> sub("link_A", 1);
                    MSG msg;
                    while (true) {
                        try {
                            msg = sub.pop();
                            std::cout << "Link A: ";
                            msg.print();
                        } catch(...) {
                            std::cout << "[WARNING] pub_A not ready." << std::endl;
                            std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        }
                    }
                }            
            ).detach();
        };
        ~example_sub() = default;

    };

}



#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(umt_example::example_sub)

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     auto image_sub_node = std::make_shared<umt_example::example_sub>();
//     rclcpp::spin(image_sub_node);
//     rclcpp::shutdown();
//     return 0;
// }