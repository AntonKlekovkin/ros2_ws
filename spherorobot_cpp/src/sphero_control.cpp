#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "csignal"

rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;

double pendulumRealAngle = 0.0;
double pendulumRealAngVel = 0.0;

void signal_handler(int sig)
{
    std_msgs::msg::Float64 vel;
    publisher->publish(vel);
    rclcpp::shutdown(); 
}

void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    pendulumRealAngle = msg->position[0];
    pendulumRealAngVel = msg->velocity[0];
}


int main(int argc, char **argv)
{
    double kp = 1.0;
    double kd = 0.0;
    double ki = 0.000;
    double angleStar = 0.0;

    double maxMotorTorque = 0.5;

    double mBody = 0.086, mPend = 0.148, iBody = 0.00025234, iPend = 7.5331E-05, R=0.06, rho=0.2656, g=9.8;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("platform_control_node");

    publisher = node->create_publisher<std_msgs::msg::Float64>("motor_torque", 10);
    auto subscriber = node->create_subscription<sensor_msgs::msg::JointState>("sphero_states", 1, JointStateCallback);
    
    node->declare_parameter<double>("kp", 1.0);
    kp = node->get_parameter("kp").as_double();

    node->declare_parameter<double>("kd", 0.0);
    kd = node->get_parameter("kd").as_double();

    node->declare_parameter<double>("ki", 0.0);
    ki = node->get_parameter("ki").as_double();

    node->declare_parameter<double>("ang", 0.0);
    angleStar = node->get_parameter("ang").as_double();
    
    RCLCPP_INFO(node->get_logger(), "kp: %f, kd: %f, ki: %f", kp, kd, ki);

    std::signal(SIGINT, signal_handler);

    std_msgs::msg::Float64 pendulum_torque;
    
    double errOld = 0.0;
    double oldTime = node->now().seconds();
    double integralPart = 0.0;

    //double feedforward = (mPend*g * sin(angleStar)*rho*(mBody*R*R + mPend*R*R + iBody + iPend))/(mPend*rho*cos(angleStar)*R + mBody*R*R + mPend*R*R+iBody);
    double feedforward = mPend * g * rho * sin(angleStar);
    
    while(rclcpp::ok())
    {
        double currentTime = node->now().seconds();
        double dt = currentTime - oldTime;
        double err = pendulumRealAngle - angleStar;
        RCLCPP_INFO(node->get_logger(), "Err: %f", err);
        integralPart += err*dt;
        double feedback = kp * err  + kd * pendulumRealAngVel + ki*integralPart;

        double torque = 0*feedforward - feedback;
        double clampedTorque = std::clamp(torque, -maxMotorTorque, maxMotorTorque);
        pendulum_torque.data = clampedTorque;

        publisher->publish(pendulum_torque);
        RCLCPP_INFO(node->get_logger(), "Feedforward: %f", feedforward);
        RCLCPP_INFO(node->get_logger(), "Torque: %f", torque);
        
        oldTime = currentTime;
        errOld = err;

        rclcpp::sleep_for(std::chrono::milliseconds(10));
        rclcpp::spin_some(node);        
    }
    
    rclcpp::shutdown();
    return 0;
}