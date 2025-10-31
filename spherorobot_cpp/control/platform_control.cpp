#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "csignal"

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

void signal_handler(int sig)
{
    geometry_msgs::msg::Twist vel;
    publisher->publish(vel);
    rclcpp::shutdown(); // это обязательно
}


int main(int argc, char **argv)
{
    double amp = 1.0;
    double hz = 1.0;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("platform_control_node");

    publisher = node->create_publisher<geometry_msgs::msg::Twist>("platform_vel", 10);
    node->declare_parameter<double>("amp", 1.0);
    amp = node->get_parameter("amp").as_double();

    node->declare_parameter<double>("hz", 1.0);
    hz = node->get_parameter("hz").as_double();
    
    std::signal(SIGINT, signal_handler);

    geometry_msgs::msg::Twist vel;
    
    while(rclcpp::ok())
    {
        double timeNow = node->now().seconds();
        double currentVel = amp * sin(2*M_PI*hz*timeNow);
        vel.linear.x = currentVel;

        publisher->publish(vel);
        RCLCPP_INFO(node->get_logger(), "Vel: %f", currentVel);
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    
    rclcpp::shutdown();
    return 0;
}