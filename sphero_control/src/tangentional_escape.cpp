#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <csignal>
#include <cmath>

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
rclcpp::Node::SharedPtr node;
geometry_msgs::msg::Twist vel;

double spheroX = 0.0;
double spheroY = 0.0;
double spheroAng_rad = 0.0;

struct Point2D {
    double x, y;
};

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

void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    spheroX = msg->position[1];
    spheroY = msg->position[2];
    spheroAng_rad = msg->position[4];
}

void GetMinDistanceAndAngle(double &minDist, double &minAngle, Point2D robotPosition, double robotOrientation)
{
    // obstacle - cilinder in (1,1), rad = 0.3
    const double radSphere = 0.06;
    const double radObst = 0.3;
    const double xObst = 1.0;
    const double yObst = 1.0;

    double r = sqrt( (xObst - robotPosition.x) * (xObst - robotPosition.x) + (yObst - robotPosition.y) * (yObst - robotPosition.y) );
    minDist = r - radSphere - radObst;

    double angleObstGlobal = atan2(yObst, xObst);
    minAngle = angleObstGlobal - robotOrientation;
}

void GetGoal(double &x, double &y)
{
    std::cout << "Введите X и Y через пробел: ";
    std::cin >> x >> y;
    
    std::cout << "Получены координаты: X = " << x << ", Y = " << y << std::endl;
}

double AngleNormalise(double angle_rad)
{
    while (angle_rad > M_PI) angle_rad -= 2 * M_PI;
    while (angle_rad < -M_PI) angle_rad += 2 * M_PI;
    return angle_rad;
}

double GetDistanceBetweenPoints(Point2D pt1, Point2D pt2)
{
    double res;
    res = sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
    return res;
}

int main(int argc, char **argv)
{
    const double minDistanceToGoal_m = 0.05;
    const double maxLinearVelocity_m_s = 0.1;
    const double maxAngularVelocity_rad_s = 1.0;
    const double dObs_m = 0.25;

    double angleMinDist = 0.0;
    double minDist = 0.0;
    
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("sphero_tangentional_escape_node");

    publisher = node->create_publisher<geometry_msgs::msg::Twist>("sphero_cmd_vel", 10);
    auto subscriberState = node->create_subscription<sensor_msgs::msg::JointState>("sphero_states", 1, JointStateCallback);
        
    std::signal(SIGINT, signal_handler);

    while(rclcpp::ok())
    {
        Point2D goal, goalTemp;
        
        GetGoal(goal.x, goal.y);
        std::cout << "Main: Получены координаты: X = " << goal.x << ", Y = " << goal.y << std::endl;

        while(rclcpp::ok())
        {
            goalTemp = goal;
            Point2D robotPosition = {spheroX, spheroY};
            GetMinDistanceAndAngle(minDist, angleMinDist, robotPosition, spheroAng_rad);

            double angleGoal_Rad = atan2(goalTemp.y - robotPosition.y, goalTemp.x - robotPosition.x);
            double beta_Rad = angleGoal_Rad - spheroAng_rad;
            double xi_Rad = angleMinDist;
            //double phi_Rad = 0.0;

            beta_Rad = AngleNormalise(beta_Rad);
            xi_Rad = AngleNormalise(xi_Rad);

            if (minDist < dObs_m)
            {
                double phi_Rad = std::signbit(xi_Rad) * (M_PI / 2) - (xi_Rad - beta_Rad);

                Point2D goalTemp1 = {goalTemp.x - robotPosition.x, goalTemp.y - robotPosition.y};

                goalTemp.x = cos(phi_Rad) * goalTemp1.x + sin(phi_Rad) * goalTemp1.y;
                goalTemp.x = -sin(phi_Rad) * goalTemp1.x + cos(phi_Rad) * goalTemp1.y;

                goalTemp = {goalTemp.x + robotPosition.x, goalTemp.y + robotPosition.y};  

                angleGoal_Rad = atan2(goalTemp.y - robotPosition.y, goalTemp.x - robotPosition.x);
                beta_Rad = angleGoal_Rad - spheroAng_rad;
                beta_Rad = AngleNormalise(beta_Rad);
            }

            double rho = GetDistanceBetweenPoints(goalTemp, robotPosition);
            if (rho < minDistanceToGoal_m)
            {
                StopRobot();
                std::cout << "Sphero is in goal!";
                break;
                //goalPoint = null; // stop and return
            }

            double kOmega = abs((maxAngularVelocity_rad_s - 0.5 * maxLinearVelocity_m_s) / (M_PI / 4));
                
            double linearVelocity = maxLinearVelocity_m_s * tanh(2*rho) * cos(beta_Rad);
            double angularVelocity = 0.0;

            if(rho < 0.2)
            {
                angularVelocity = kOmega * beta_Rad + maxLinearVelocity_m_s * sin(beta_Rad) * cos(beta_Rad);
            }
            else
            {
                angularVelocity = kOmega * beta_Rad + maxLinearVelocity_m_s * (tanh(2*rho) / (2*rho)) * sin(beta_Rad) * cos(beta_Rad);
            }
            
            MoveRobot(linearVelocity, angularVelocity);
            rclcpp::spin_some(node);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        rclcpp::spin_some(node);
        //rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }
    
    rclcpp::shutdown();
    return 0;
}