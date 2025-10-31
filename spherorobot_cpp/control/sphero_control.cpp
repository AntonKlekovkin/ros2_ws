#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "csignal"

rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;

double pendulumRealAngle = 0.0;

void signal_handler(int sig)
{
    std_msgs::msg::Float64 vel;
    publisher->publish(vel);
    rclcpp::shutdown(); // это обязательно
}

void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    pendulumRealAngle = msg->velocity[0];
}


int main(int argc, char **argv)
{
    double kp = 1.0;
    double kd = 1.0;
    double ki = 0.001;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("platform_control_node");

    publisher = node->create_publisher<std_msgs::msg::Float64>("motor_torque", 10);
    auto subscriber = node->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, JointStateCallback);
    
    node->declare_parameter<double>("kp", 1.0);
    kp = node->get_parameter("kp").as_double();

    node->declare_parameter<double>("kd", 1.0);
    kd = node->get_parameter("kd").as_double();

    node->declare_parameter<double>("ki", 1.0);
    kd = node->get_parameter("ki").as_double();
    
    std::signal(SIGINT, signal_handler);

    std_msgs::msg::Float64 pendulum_torque;
    
    double angleOld = pendulumRealAngle;
    double oldTime = node->now().seconds();
    while(rclcpp::ok())
    {
        double currentTime = node->now().seconds();
        double dt = currentTime - oldTime;
        double torque = kp * pendulumRealAngle + kd * (pendulumRealAngle - angleOld) / dt;

        pendulum_torque.data = torque;

        publisher->publish(pendulum_torque);
        RCLCPP_INFO(node->get_logger(), "Torque: %f", torque);
        
        oldTime = currentTime;
        angleOld = pendulumRealAngle;

        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    
    rclcpp::shutdown();
    return 0;
}