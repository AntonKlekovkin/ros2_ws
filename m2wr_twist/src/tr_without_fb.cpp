#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <csignal>
#include <vector>

#define PI 3.1415926

rclcpp::Node::SharedPtr node;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubTheorTrajectory;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubTheorLinVel;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubTheorAngVel;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubRealLinVel;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubRealAngVel;

geometry_msgs::msg::Twist messageStop;
geometry_msgs::msg::Twist messageMotion;
geometry_msgs::msg::Vector3 points;

double linVelReal = 0;
double angVelReal = 0;

void StopRobot(void);

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

void msgCallbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    linVelReal = msg->twist.twist.linear.x;
    angVelReal = msg->twist.twist.angular.z;
}

void CalculateParametrizationFunction(int length, float * arr)
{
    int i = 0;

    const float a1=23.55, b1=0.2186, c1=1.579, a2=23.25, b2=0.2487, c2=4.755, 
        a3=0.1469, b3=1.11, c3=4.841, a4=0.1867, b4=3.519, c4=-1.52;
    
    //float endT = PI/2;    // for sin^2
    float endT = 8;         // for sum of sin
    //float endT = 2*PI;    // for linear

    for(i = 0; i < length; i++)
    {
        float s = ((float)i/(float)length)*endT;
        //arr[i] = sin(s)*sin(s)*2*PI;      // sin^2
        arr[i] = a1*sin(b1*s+c1) + a2*sin(b2*s+c2) + a3*sin(b3*s+c3) + a4*sin(b4*s+c4); // sum of sin
        //arr[i] = s;                       // linear
        
        RCLCPP_INFO(node->get_logger(),"i=%d, t=%f", i, arr[i]);    
    }    
}

void CalculatePointsOfTrajectory(int length, float * x, float * y, float * t)
{
    float kTr = 1;  
    int i =0;  
    for(i=0; i < length; i++)
    {
        // trajectory "infinity"
        x[i] = sin(t[i]+(PI/2))*kTr;
        y[i] = sin(2*t[i])*kTr;

        //trajectory circle
        //x[i] = sin(t[i])*kTr;
        //y[i] = cos(t[i])*kTr;

        points.x = y[i];
        points.y = (x[i]-kTr) *(-1);
        pubTheorTrajectory->publish(points);
        Sleep_ms(20);
    }
}

float Range(float val, float min, float max)
{
    if(val < min) return min;
    if(val > max) return max;
    return val;
}

float RemapAngle(float val)
{
    while(val > PI) { val -= 2*PI; }
    while(val < -PI) { val += 2*PI; }

    return val;
}
float GetMax(int length, float *arr)
{
    float max = 0;

    for(int i = 0; i < length; i++)
    {
        if(arr[i] > max)
        {
            max = arr[i];
        }
    }

    return max;
}
float GetMin(int length, float *arr)
{
    float min = arr[0];

    for(int i = 0; i < length; i++)
    {
        if(arr[i] < min)
        {
            min = arr[i];
        }
    }

    return min;
}
void CalculateVelocities(const int length, float dt, float thetaStart, float * x, float * y, float * v, float * w)
{
    float dx, dy;
    float tetta[length];
    float dl[length];
    float dw[length];
    
    for(int i = 0; i < length; i++)
    {
        dx = x[i+1] - x[i];
        dy = y[i+1] - y[i];
        dl[i] = sqrt(dx*dx + dy*dy);
        tetta[i] = atan2(y[i+1] - y[i], x[i+1] - x[i]);

        if(tetta[i] < 0) tetta[i] += 2*PI;

        if(i==0)
        {
            dw[i] = tetta[i] - thetaStart;
        }
        else
        {
            dw[i] = tetta[i] - tetta[i-1];
        }

        dw[i] = RemapAngle(dw[i]);

        v[i] = dl[i] / dt;
        w[i] = dw[i] / dt;             
    }    
}

void PublishTheoreticalVelocities(int length, float * arr, float coeff, rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub)
{
    geometry_msgs::msg::Vector3 message;
    for(int i = 0; i < length; i++)
    {
        message.x = i;
        message.y = arr[i]*coeff;
        
        pub->publish(message);
        Sleep_ms(20);        
    }
}

void TransmitVelocitiesToRobot(int length, float dt, float k, float * v, float * w,  bool transmit)
{
    geometry_msgs::msg::Vector3 messageLinVel;
    geometry_msgs::msg::Vector3 messageAngVel;

    for(int i = 0; i < length; i++)
    {
        RCLCPP_INFO(node->get_logger(),"i=%d, v=%f, w=%f", i, v[i], w[i]);
                
        messageMotion.linear.x = v[i]*k;
        messageMotion.angular.z = w[i]*k;
        
        if(transmit)
        {   
            pub->publish(messageMotion);
            
            messageLinVel.x = i;
            messageLinVel.y = linVelReal;
            pubRealLinVel->publish(messageLinVel);

            messageAngVel.x = i;
            messageAngVel.y = angVelReal;
            pubRealAngVel->publish(messageAngVel);

            Sleep_ms(dt/k * 1000);            
        }
        rclcpp::spin_some(node);
    }
}

void StopRobot()
{
    pub->publish(messageStop);
    Sleep_ms(1000);
}

void SetupPublishers(rclcpp::Node::SharedPtr node)
{
    pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100);
    pubTheorTrajectory = node->create_publisher<geometry_msgs::msg::Vector3>("theor_trajectory", 1000);
    pubTheorLinVel = node->create_publisher<geometry_msgs::msg::Vector3>("theor_lin_vel", 1000);
    pubTheorAngVel = node->create_publisher<geometry_msgs::msg::Vector3>("theor_ang_vel", 1000);
    pubRealLinVel = node->create_publisher<geometry_msgs::msg::Vector3>("real_lin_vel", 1000);
    pubRealAngVel = node->create_publisher<geometry_msgs::msg::Vector3>("real_ang_vel", 1000);
}

int main(int argc, char* argv[])
{
    const int numberPoints = 500;
    const bool transmit = true;
    const float maxLinVelocityReal = 1.5;
    const float maxAngVelocityReal = 1.5;
    const float thetaStart = PI/2;

    // init arrays
    float t[numberPoints];
    float x[numberPoints];
    float y[numberPoints];
    
    float v[numberPoints-1];
    float w[numberPoints-1];

    float dt = 0.05;

    // init ROS
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("tr_without_fb_node");
    // init of subscriber to get real velocities while motion
    auto subOdom = node->create_subscription<nav_msgs::msg::Odometry>("odom", 1, msgCallbackOdom);
    rclcpp::spin_some(node);

    // init sigint event, this event occurs when node exit (CTRL+C)
    std::signal(SIGINT, mySigintHandler);
    
    // init publishers
    SetupPublishers(node);
        
    // calculate parameters for motion
    CalculateParametrizationFunction(numberPoints, t);
    CalculatePointsOfTrajectory(numberPoints, x, y, t);
    CalculateVelocities(numberPoints-1, dt, thetaStart, x, y, v, w);

    float maxLinVelocity = GetMax(numberPoints-1, v);
    float maxAngVelocity = GetMax(numberPoints-1, w);

    RCLCPP_INFO(node->get_logger(),"maxV = %f, maxW = %f", maxLinVelocity, maxAngVelocity);

    rclcpp::spin_some(node);

    // calculate coefficient of scale velocities
    float k1,k2,k;
    float coeffSafety = 1;
    k1 = maxLinVelocityReal/maxLinVelocity * coeffSafety;
    k2 = maxAngVelocityReal/maxAngVelocity * coeffSafety;
    k = GetMin(2, new float[2] {k1, k2});
        
    RCLCPP_INFO(node->get_logger(), "k=%f", k);

    // publish calculated linear and angular velocities
    PublishTheoreticalVelocities(numberPoints-1, v, k, pubTheorLinVel);
    PublishTheoreticalVelocities(numberPoints-1, w, k, pubTheorAngVel);

    TransmitVelocitiesToRobot(numberPoints-1, dt, k, v, w, transmit);

    StopRobot();
    
    rclcpp::shutdown();
    return 0;    
}