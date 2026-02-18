#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "csignal"
#include <cmath>

rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;

double spheroRealYaw = 0.0;
double yawStar = 0.0;

double normalizeAngle(double angle)
{
    // Приводим угол в диапазон [-π, π]
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

void signal_handler(int sig)
{
    std_msgs::msg::Float64 rotor_torque;
    rotor_torque.data = 0.0f;
    publisher->publish(rotor_torque);
    rclcpp::shutdown(); 
}

void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    spheroRealYaw = msg->position[4];
}

void TargetYawCallback(const std_msgs::msg::Float64::SharedPtr msg)
{    
    double yaw = msg->data;    
    // yawStar = normalizeAngle(yaw); // Преобразуем в [-π, π]
    yawStar = yaw;
}
    

int main(int argc, char **argv)
{
    
    double kp = 1.0;
    double kd = 0.0;
    double ki = 0.000;
    
    double maxRotorTorque = 0.05;
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sphero_yaw_control_node");

    publisher = node->create_publisher<std_msgs::msg::Float64>("rotor_torque", 10);
    auto subscriberState = node->create_subscription<sensor_msgs::msg::JointState>("sphero_states", 1, JointStateCallback);
    auto subscriberTargetYaw = node->create_subscription<std_msgs::msg::Float64>("sphero_target_yaw", 1, TargetYawCallback);
    
    node->declare_parameter<double>("yaw", 0.0);
    yawStar = node->get_parameter("yaw").as_double();

    node->declare_parameter<double>("kp", 1.0);
    kp = node->get_parameter("kp").as_double();

    node->declare_parameter<double>("kd", 0.0);
    kd = node->get_parameter("kd").as_double();

    node->declare_parameter<double>("ki", 0.0);
    ki = node->get_parameter("ki").as_double();
        
    RCLCPP_INFO(node->get_logger(), "yaw*: %f", yawStar);
    RCLCPP_INFO(node->get_logger(), "kp: %f, kd: %f, ki: %f", kp, kd, ki);

    std::signal(SIGINT, signal_handler);

    std_msgs::msg::Float64 rotor_torque;
    
    double errOld = 0.0;
    double oldTime = node->now().seconds();
    double integralPart = 0.0;
    
    while(rclcpp::ok())
    {
        double currentTime = node->now().seconds();
        double dt = currentTime - oldTime;
        double err = spheroRealYaw - yawStar;
        double derr = (err - errOld) / dt;
        RCLCPP_INFO(node->get_logger(), "Err: %f", err);

        double integral_max = maxRotorTorque / ki * 0.8;
        integralPart += err*dt;
        integralPart = std::clamp(integralPart, -integral_max, integral_max);
        double feedback = kp * err  + kd * derr + ki * integralPart;

        double torque = -feedback;
        double clampedTorque = std::clamp(torque, -maxRotorTorque, maxRotorTorque);
        rotor_torque.data = clampedTorque;

        publisher->publish(rotor_torque);
        
        RCLCPP_INFO(node->get_logger(), "Torque: %f", torque);
        RCLCPP_INFO(node->get_logger(), "ClampedTorque: %f", clampedTorque);
        
        oldTime = currentTime;
        errOld = err;

        rclcpp::sleep_for(std::chrono::milliseconds(10));
        rclcpp::spin_some(node);        
    }
    
    
    rclcpp::shutdown();
    return 0;
}