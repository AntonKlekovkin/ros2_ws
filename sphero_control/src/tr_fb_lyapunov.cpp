#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <csignal>
#include <vector>
#include <cmath>

#define PI 3.1415926

rclcpp::Node::SharedPtr node;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubTheorTrajectory;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubTheorLinVel;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubTheorAngVel;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubRealLinVel;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubRealAngVel;

// geometry_msgs::msg::Twist messageStop;
// geometry_msgs::msg::Twist messageMotion;
geometry_msgs::msg::Vector3 points;

geometry_msgs::msg::Twist vel;

double linVelReal = 0;
double angVelReal = 0;

double spheroX = 0.0;
double spheroY = 0.0;
double spheroAng_rad = 0.0;

double scaleTrajectory = 0.5;
double parCoeff = 0.045;

double xStar(double t) { return scaleTrajectory * sin(parCoeff * t + M_PI / 2.0); }
double dxStar(double t) { return parCoeff*scaleTrajectory * cos(parCoeff*t + M_PI / 2); }
double ddxStar(double t) { return -parCoeff * parCoeff*scaleTrajectory * sin(parCoeff * t + M_PI / 2); }

double yStar(double t) { return scaleTrajectory * sin(2 * parCoeff*t); }
double dyStar(double t) { return parCoeff*scaleTrajectory * 2 * cos(2 * parCoeff*t); }
double ddyStar(double t) { return-parCoeff* parCoeff*scaleTrajectory * 4 * sin(2 * parCoeff*t); }

void StopRobot()
{
    vel.linear.x = 0;
    vel.angular.z = 0;
    pub->publish(vel);
}

void MoveRobot(float lin, float ang)
{
    vel.linear.x = lin;
    vel.angular.z = ang;
    pub->publish(vel);
}

void Sleep_ms(int ms)
{
    rclcpp::sleep_for(std::chrono::milliseconds(ms));
}

void mySigintHandler(int sig)
{
    StopRobot();
    RCLCPP_INFO(node->get_logger(), "Received SIGINT %d, shutting down...", sig);  
    rclcpp::shutdown();   
}

void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    spheroX = msg->position[1];
    spheroY = msg->position[2];
    spheroAng_rad = msg->position[4];

    linVelReal = msg->velocity[1];
    angVelReal = msg->velocity[4];
}




void SetupPublishers(rclcpp::Node::SharedPtr node)
{
    pub = node->create_publisher<geometry_msgs::msg::Twist>("sphero_cmd_vel", 100);
    pubTheorTrajectory = node->create_publisher<geometry_msgs::msg::Vector3>("theor_trajectory", 1000);
    pubTheorLinVel = node->create_publisher<geometry_msgs::msg::Vector3>("theor_lin_vel", 1000);
    pubTheorAngVel = node->create_publisher<geometry_msgs::msg::Vector3>("theor_ang_vel", 1000);
    pubRealLinVel = node->create_publisher<geometry_msgs::msg::Vector3>("real_lin_vel", 1000);
    pubRealAngVel = node->create_publisher<geometry_msgs::msg::Vector3>("real_ang_vel", 1000);
}

int main(int argc, char* argv[])
{
    const double k1 = 1.0;
    const double k2 = 1.0;
    const double k3 = 1.0;
    
    // init ROS
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("tr_without_fb_node");
    // init of subscriber to get real velocities while motion
    //auto subOdom = node->create_subscription<nav_msgs::msg::Odometry>("odom", 1, msgCallbackOdom);
    auto subscriberState = node->create_subscription<sensor_msgs::msg::JointState>("sphero_states", 1, JointStateCallback);
    pub = node->create_publisher<geometry_msgs::msg::Twist>("sphero_cmd_vel", 100);
    pubTheorTrajectory = node->create_publisher<geometry_msgs::msg::Vector3>("theor_trajectory", 1000);
    rclcpp::spin_some(node);

    // init sigint event, this event occurs when node exit (CTRL+C)
    std::signal(SIGINT, mySigintHandler);
    
    // CalculatePointsOfTrajectory(numberPoints, x, y, t);
    // // publish calculated linear and angular velocities
    // PublishTheoreticalVelocities(numberPoints-1, v, k, pubTheorLinVel);
    // PublishTheoreticalVelocities(numberPoints-1, w, k, pubTheorAngVel);

    double startTime_s = node->now().seconds();
    while(rclcpp::ok())
    {
        double currentTime_s = node->now().seconds();
        double time_s = currentTime_s - startTime_s;

        if(time_s > 2*2*M_PI/parCoeff)
        {
            StopRobot();
            std::cout << "Trajectory is done!";
            break;
        }

        double x = spheroX;
        double y = spheroY;
        double alpha = spheroAng_rad;

        double xS = xStar(time_s);
        double dxS = dxStar(time_s);
        double ddxS = ddxStar(time_s);
        double yS = yStar(time_s);
        double dyS = dyStar(time_s);
        double ddyS = ddyStar(time_s);

        double vS = sqrt(dxS * dxS + dyS * dyS);
        double wS = (ddyS*dxS - dyS*ddxS) / (dxS*dxS + dyS*dyS);
        double alphaS =atan2(dyS, dxS);

        double e1 = cos(alpha) * (xS - x) + sin(alpha) * (yS - y);
        double e2 = -sin(alpha) * (xS - x) + cos(alpha) * (yS - y);
        double e3 = alphaS - alpha;

        double u1 = -k1 * e1;
        double u2 = 0.0;

        if (e3 > 0.2)
        {
            u2 = -k2 * e3 - k3 * e2 * vS * sin(e3) / e3;
        }
        else
        {
            u2 = -k2 * e3 - k3 * e2 * vS;
        }

        double v = vS * cos(e3) - u1;
        double w = wS - u2;

        points.x = xS;
        points.y = yS;
        pubTheorTrajectory->publish(points);

        MoveRobot(v, w);
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    StopRobot();
    
    rclcpp::shutdown();
    return 0;    
}