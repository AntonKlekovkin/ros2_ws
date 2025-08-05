#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <csignal>
#define PI 3.1415926f

rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
rclcpp::Node::SharedPtr node;

const int distArrLength = 360;
float distArr[distArrLength];
float angleIncrement;


void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	angleIncrement = msg->angle_increment;
    for(int i = 0; i < distArrLength; i++)
    {
        distArr[i] = msg->ranges[i];
        RCLCPP_INFO(node->get_logger(), "angle: %f, dist: %f", i*angleIncrement*180.0f/PI, distArr[i]);
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("m2wr_lidar");
    auto subscriber = node->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, LidarCallback);
        
    rclcpp::spin(node);    
    rclcpp::shutdown();
    return 0;
}