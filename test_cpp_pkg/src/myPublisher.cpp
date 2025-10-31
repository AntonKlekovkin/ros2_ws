#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv)
{
    int count = 0;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("my_node_pub");

    auto publisher = node->create_publisher<std_msgs::msg::String>("platform_vel", 10);
    node->declare_parameter<int>("startValue", 0);
    count = node->get_parameter("startValue").as_int();
    
    while(rclcpp::ok())
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello World!!! " +  std::to_string(count++);
        publisher->publish(message);
        RCLCPP_INFO(node->get_logger(), "Pub: %s", message.data.c_str());
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }
    
    rclcpp::shutdown();
    return 0;
}