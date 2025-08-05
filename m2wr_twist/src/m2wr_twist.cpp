#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <csignal>

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
rclcpp::Node::SharedPtr node;
geometry_msgs::msg::Twist vel;

void StopRobot()
{
    vel.linear.x = 0;
    vel.angular.z = 0;
    publisher->publish(vel);
}

void MoveRobot(float lin, float ang)
{
    vel.linear.x = lin;
    vel.angular.z = ang;
    publisher->publish(vel);
}

void signal_handler(int sig) 
{
    StopRobot();
    RCLCPP_INFO(node->get_logger(), "Received SIGINT %d, shutting down...", sig);
    rclcpp::shutdown();   
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("m2wr_twist");
    publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    vel.angular.z = 1.0f;

    std::signal(SIGINT, signal_handler);

    while(rclcpp::ok())
    {
        char ch = getchar();
        if(ch == 'w')
        {
            MoveRobot(1,0); //эту функцию нужно прописать
        }
        else if(ch == 's')
        {
            StopRobot(); //эту функцию нужно прописать
        }

        rclcpp::spin_some(node);
        //rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }
    
    rclcpp::shutdown();
    return 0;
}