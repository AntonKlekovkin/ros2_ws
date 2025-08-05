#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"  // Для tf2::Quaternion
#include "tf2/LinearMath/Matrix3x3.h"   // Для tf2::Matrix3x3 и getRPY()
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // Для tf2::fromMsg()

#define PI 3.1415926f

rclcpp::Node::SharedPtr node;

double xReal = 0;
double yReal = 0;
double linVelReal = 0;
double angVelReal = 0;
double alpha = 0;

void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
   
	xReal = msg->pose.pose.position.x;
	yReal = msg->pose.pose.position.y;
    linVelReal = msg->twist.twist.linear.x;
    angVelReal = msg->twist.twist.angular.z;
    alpha = yaw *180.0 / PI;
    RCLCPP_INFO(node->get_logger(), "angle: %f, x: %f", alpha, xReal);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("m2wr_lidar");
    auto subscriber = node->create_subscription<nav_msgs::msg::Odometry>("odom", 10, OdomCallback);
        
    rclcpp::spin(node);    
    rclcpp::shutdown();
    return 0;
}