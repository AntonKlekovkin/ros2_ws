#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define PI 3.1415926f

rclcpp::Node::SharedPtr node;

double xReal = 0;
double yReal = 0;
double linVelReal = 0;
double angVelReal = 0;
double alpha = 0;

void theor_trajectory_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) { }
void theor_lin_vel_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) { }
void theor_ang_vel_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) { }
void real_lin_vel_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) { }
void real_ang_vel_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) { }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("modelling_subscribers");
    auto subscriber1 = node->create_subscription<geometry_msgs::msg::Vector3>("theor_trajectory", 10, theor_trajectory_callback);
    auto subscriber2 = node->create_subscription<geometry_msgs::msg::Vector3>("theor_lin_vel", 10, theor_lin_vel_callback);
    auto subscriber3 = node->create_subscription<geometry_msgs::msg::Vector3>("theor_ang_vel", 10, theor_ang_vel_callback);
    auto subscriber4 = node->create_subscription<geometry_msgs::msg::Vector3>("real_lin_vel", 10, real_lin_vel_callback);
    auto subscriber5 = node->create_subscription<geometry_msgs::msg::Vector3>("real_ang_vel", 10, real_ang_vel_callback);

    rclcpp::spin(node);    
    rclcpp::shutdown();
    return 0;
}