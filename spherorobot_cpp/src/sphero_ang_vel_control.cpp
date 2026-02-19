#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "csignal"
#include <cmath>

rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;

double spheroRealAngVel = 0.0;
double angVelStar = 0.0;

void signal_handler(int sig)
{
    std_msgs::msg::Float64 rotor_torque;
    rotor_torque.data = 0.0f;
    publisher->publish(rotor_torque);
    rclcpp::shutdown(); 
}

void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    spheroRealAngVel = msg->velocity[4];
}

void TargetAngVelCallback(const std_msgs::msg::Float64::SharedPtr msg)
{    
    angVelStar = msg->data;        
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
    auto subscriberTargetYaw = node->create_subscription<std_msgs::msg::Float64>("sphero_target_ang_vel", 1, TargetAngVelCallback);
    
    node->declare_parameter<double>("ang_vel", 0.0);
    angVelStar = node->get_parameter("ang_vel").as_double();

    node->declare_parameter<double>("ang_vel_kp", 0.07);
    kp = node->get_parameter("ang_vel_kp").as_double();

    node->declare_parameter<double>("ang_vel_kd", 0.00001);
    kd = node->get_parameter("ang_vel_kd").as_double();

    node->declare_parameter<double>("ang_vel_ki", 0.1);
    ki = node->get_parameter("ang_vel_ki").as_double();
        
    RCLCPP_INFO(node->get_logger(), "angVel*: %f", angVelStar);
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
        double err = spheroRealAngVel - angVelStar;
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