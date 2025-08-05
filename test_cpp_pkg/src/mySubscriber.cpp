#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

rclcpp::Node::SharedPtr node;

void SubCallback(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(node->get_logger(), "Receive: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("my_node_sub");

    auto subscriber = node->create_subscription<std_msgs::msg::String>("topic_cpp", 10, SubCallback);
    
    RCLCPP_INFO(node->get_logger(), "Subscriber is ready!");

    rclcpp::spin(node);    
    rclcpp::shutdown();
    return 0;
}