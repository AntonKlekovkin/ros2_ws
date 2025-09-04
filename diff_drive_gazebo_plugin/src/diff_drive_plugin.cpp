#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>
#include <random>
#include <cmath>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo 
{
    std::default_random_engine generator{std::random_device{}()};
    std::normal_distribution<double> distribution(0.0, 1.0);

    class DiffDrivePlugin : public ModelPlugin 
    {
    public:
        void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override 
        {
            ros_node_ = gazebo_ros::Node::Get(sdf, "diff_drive_controller");
            
            // Чтение параметров из urdf с значениями по умолчанию
            ppr_left_ = sdf->Get<int>("ppr_left", 2000).first;
            ppr_right_ = sdf->Get<int>("ppr_right", 2000).first;
            wheel_radius_ = sdf->Get<double>("wheel_radius", 0.1).first;
            wheel_separation_ = sdf->Get<double>("wheel_separation", 0.5).first;
            custom_odom_topic_ = sdf->Get<std::string>("custom_odom_topic", "custom_odom").first;
            
            // Инициализация переменных
            prev_left_angle_ = 0.0;
            prev_right_angle_ = 0.0;
            prev_left_ticks_ = 0;
            prev_right_ticks_ = 0;
            left_ticks_ = 0;
            right_ticks_ = 0;
            x_ = 0.0;
            y_ = 0.0;
            theta_ = 0.0;
            prev_x_ = 0.0;
            prev_y_ = 0.0;
            prev_theta_ = 0.0;
            last_update_time_ = model->GetWorld()->SimTime();
            
            //параметры помеж энкодера

            // Публикатор данных энкодера
            encoder_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            custom_odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(custom_odom_topic_, 10);
            // Подписка на команды управления
            cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel_wheel", 10,
                [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                    // Расчет скоростей колес
                    left_vel_ = (msg->linear.x - msg->angular.z * wheel_separation_ / 2) / wheel_radius_;
                    right_vel_ = (msg->linear.x + msg->angular.z * wheel_separation_ / 2) / wheel_radius_;
                    
                    // Установка скоростей в Gazebo
                    model_->GetJoint("left_wheel_joint")->SetVelocity(0, left_vel_);
                    model_->GetJoint("right_wheel_joint")->SetVelocity(0, right_vel_);
                });
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);
            // Подключение к циклу обновления Gazebo
            update_connection_ = event::Events::ConnectWorldUpdateBegin(
                std::bind(&DiffDrivePlugin::OnUpdate, this));
                
            // Сохранение указателей на модель и суставы
            model_ = model;
            left_joint_ = model_->GetJoint("left_wheel_joint");
            right_joint_ = model_->GetJoint("right_wheel_joint");
            
            RCLCPP_INFO(ros_node_->get_logger(), 
                    "DiffDrivePlugin loaded: PPR left=%d, PPR right=%d, Wheel radius=%.3f m, Wheel separation=%.3f m",
                    ppr_left_, ppr_right_, wheel_radius_, wheel_separation_);
        }

        void OnUpdate() {
            // Расчет времени дискретизации
            common::Time current_time = model_->GetWorld()->SimTime();
            double dt = (current_time - last_update_time_).Double();
            last_update_time_ = current_time;
            
            // Текущие углы поворота колес
            double left_angle = left_joint_->Position(0);
            double right_angle = right_joint_->Position(0);
            
            // Разница углов с предыдущим шагом
            double delta_left = left_angle - prev_left_angle_;
            double delta_right = right_angle - prev_right_angle_;

            // Коррекция при переходе через 2π
            if (delta_left > M_PI) delta_left -= 2 * M_PI;
            if (delta_left < -M_PI) delta_left += 2 * M_PI;
            if (delta_right > M_PI) delta_right -= 2 * M_PI;
            if (delta_right < -M_PI) delta_right += 2 * M_PI;
            
            // Расчет импульсов энкодера с возможностью пропуска
            left_ticks_ += static_cast<int>(std::round(((delta_left / (2 * M_PI)) * ppr_left_) + (std::abs((delta_left / (2 * M_PI)) * ppr_left_) * error_scale * distribution(generator))));
            right_ticks_ += static_cast<int>(std::round(((delta_right / (2 * M_PI)) * ppr_right_) + (std::abs((delta_right / (2 * M_PI)) * ppr_right_) * error_scale * distribution(generator))));
            
            //Рассчет пройденого расстояния по энкодерам
            int delta_left_ticks = left_ticks_ - prev_left_ticks_;
            int delta_right_ticks = right_ticks_ - prev_right_ticks_;
            double left_dist = ((left_ticks_ - prev_left_ticks_) * 2*M_PI * wheel_radius_) / ppr_left_;
            double right_dist = ((right_ticks_ - prev_right_ticks_) * 2*M_PI * wheel_radius_) / ppr_right_;
            
            double dDist = (left_dist + right_dist) / 2;
            double dTheta = (left_dist - right_dist) / wheel_separation_;

            x_ = prev_x_ + dDist * cos(prev_theta_ + dTheta/2);
            y_ = prev_y_ + dDist * sin(prev_theta_ + dTheta/2);
            theta_ = prev_theta_ + dTheta;
            theta_ = atan2(sin(theta_), cos(theta_));

            // Сохранение текущих углов и импульсов энкодера
            prev_left_angle_ = left_angle;
            prev_right_angle_ = right_angle;
            prev_left_ticks_ = left_ticks_;
            prev_right_ticks_ = right_ticks_;

            prev_x_ = x_;
            prev_y_ = y_;
            prev_theta_ = theta_;

            // Подготовка и публикация данных с энкодера
            auto encoder_msg = sensor_msgs::msg::JointState();
            encoder_msg.header.stamp = ros_node_->get_clock()->now();
            encoder_msg.name = {"left_wheel_joint", "right_wheel_joint"};
            encoder_msg.position = {left_angle, right_angle};
            encoder_msg.velocity = {left_vel_, right_vel_};
            encoder_msg.effort = {static_cast<double>(left_ticks_), static_cast<double>(right_ticks_)};

            //Подготовка и публикация данных одометрии
            auto odom_msg = nav_msgs::msg::Odometry();
            odom_msg.header.stamp = encoder_msg.header.stamp;
            odom_msg.header.frame_id = "custom_odom";
            odom_msg.child_frame_id = "base_link";
            odom_msg.pose.pose.position.x = x_;
            odom_msg.pose.pose.position.y = y_;
            tf2::Quaternion q1;
            q1.setRPY(0, 0, theta_);
            odom_msg.pose.pose.orientation = tf2::toMsg(q1);
            odom_msg.twist.twist.linear.x = (x_ - prev_x_) / dt;
            odom_msg.twist.twist.linear.y = (y_ - prev_y_) / dt;
            odom_msg.twist.twist.angular.z = (theta_ - prev_theta_) / dt;

            geometry_msgs::msg::TransformStamped odom_tf;
            odom_tf.header.stamp = ros_node_->get_clock()->now();
            odom_tf.header.frame_id = "custom_odom";
            odom_tf.child_frame_id = "base_link";
            odom_tf.transform.translation.x = x_;
            odom_tf.transform.translation.y = y_;
            odom_tf.transform.translation.z = 0.0;
            tf2::Quaternion q2;
            q2.setRPY(0, 0, theta_);
            odom_tf.transform.rotation = tf2::toMsg(q2);

            encoder_pub_->publish(encoder_msg);
            custom_odom_pub_->publish(odom_msg);
            tf_broadcaster_->sendTransform(odom_tf);
        }

    private:
        // Указатели на объекты Gazebo
        physics::ModelPtr model_;
        physics::JointPtr left_joint_, right_joint_;
        event::ConnectionPtr update_connection_;
        
        // ROS интерфейс
        gazebo_ros::Node::SharedPtr ros_node_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoder_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr custom_odom_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // Параметры
        int ppr_left_ = 2000;
        int ppr_right_ = 2000;
        double wheel_radius_ = 0.1;
        double wheel_separation_ = 0.5;
        std::string custom_odom_topic_;

        double error_scale = 0.01;

    
        // Состояние
        double left_vel_ = 0.0;
        double right_vel_ = 0.0;
        double prev_left_angle_ = 0.0;
        double prev_right_angle_ = 0.0;
        double x_ = 0.0;
        double y_ = 0.0;
        double theta_ = 0.0;
        double prev_x_ = 0.0;
        double prev_y_ = 0.0;
        double prev_theta_ = 0.0;
        int64_t left_ticks_ = 0;
        int64_t right_ticks_ = 0;
        int64_t prev_left_ticks_ = 0;
        int64_t prev_right_ticks_ = 0;
        common::Time last_update_time_;
    };
GZ_REGISTER_MODEL_PLUGIN(DiffDrivePlugin)
}  // namespace gazebo