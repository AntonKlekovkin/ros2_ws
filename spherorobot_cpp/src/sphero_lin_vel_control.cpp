#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "csignal"
#include <cmath>

rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;

double spheroRealLinVel = 0.0;
double linVelStar = 0.0;

void signal_handler(int sig)
{
    std_msgs::msg::Float64 rotor_torque;
    rotor_torque.data = 0.0f;
    publisher->publish(rotor_torque);
    rclcpp::shutdown(); 
}

void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    spheroRealLinVel = msg->velocity[1];
}

void TargetLinVelCallback(const std_msgs::msg::Float64::SharedPtr msg)
{    
    linVelStar = msg->data;        
}
    

int main(int argc, char **argv)
{    
    double kp = 1.0;
    double kd = 0.0;
    double ki = 0.000;
    
    double maxPendulumTorque = 0.5;
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sphero_lin_vel_control_node");

    publisher = node->create_publisher<std_msgs::msg::Float64>("pendulum_torque", 10);
    auto subscriberState = node->create_subscription<sensor_msgs::msg::JointState>("sphero_states", 1, JointStateCallback);
    auto subscriberTargetYaw = node->create_subscription<std_msgs::msg::Float64>("sphero_target_lin_vel", 1, TargetLinVelCallback);
    
    node->declare_parameter<double>("vel", 0.0);
    linVelStar = node->get_parameter("vel").as_double();

    node->declare_parameter<double>("kp", 0.07);
    kp = node->get_parameter("kp").as_double();

    node->declare_parameter<double>("kd", 0.00001);
    kd = node->get_parameter("kd").as_double();

    node->declare_parameter<double>("ki", 0.1);
    ki = node->get_parameter("ki").as_double();
        
    RCLCPP_INFO(node->get_logger(), "angVel*: %f", linVelStar);
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
        double err = spheroRealLinVel - linVelStar;
        double derr = (err - errOld) / dt;
        RCLCPP_INFO(node->get_logger(), "Err: %f", err);

        double integral_max = maxPendulumTorque / ki * 0.8;
        integralPart += err*dt;
        integralPart = std::clamp(integralPart, -integral_max, integral_max);
        double feedback = kp * err  + kd * derr + ki * integralPart;

        double torque = -feedback;
        double clampedTorque = std::clamp(torque, -maxPendulumTorque, maxPendulumTorque);
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