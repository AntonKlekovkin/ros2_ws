#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "csignal"
#include <cmath>

rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubLinVel;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubAngVel;

void signal_handler(int sig)
{
    std_msgs::msg::Float64 zeroMessage;
    zeroMessage.data = 0.0f;
    pubLinVel->publish(zeroMessage);
    pubAngVel->publish(zeroMessage);
    rclcpp::shutdown(); 
}

void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{    
    double linear = msg->linear.x;
    double angular = msg->angular.z;
    std_msgs::msg::Float64 linMessage;
    std_msgs::msg::Float64 angMessage;

    linMessage.data = linear;
    angMessage.data = angular;

    pubLinVel->publish(linMessage);
    pubAngVel->publish(angMessage);
}
    

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sphero_vw_control_node");

    pubLinVel = node->create_publisher<std_msgs::msg::Float64>("sphero_target_lin_vel", 10);
    pubAngVel = node->create_publisher<std_msgs::msg::Float64>("sphero_target_ang_vel", 10);

    auto subscriberCmdVel = node->create_subscription<geometry_msgs::msg::Twist>("sphero_cmd_vel", 1, CmdVelCallback);

    std::signal(SIGINT, signal_handler);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}