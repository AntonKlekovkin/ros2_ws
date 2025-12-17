#include <gazebo/common/Plugin.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
    class PendulumSimplePlugin : public ModelPlugin
    {
        private:
            physics::ModelPtr model_;
            physics::JointPtr pendulum_joint_{nullptr};
            physics::LinkPtr pendulum_link_{nullptr};
            event::ConnectionPtr update_connection_;
            gazebo_ros::Node::SharedPtr node_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr torque_sub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pendulum_state_pub_;
            double target_torque_ = 0.0;
            bool initialized_ = false;
            double prevAngle_ = 0.0;
            bool firstCall_ = true;
            int fullRevolutions_ = 0;

            double getContinuousAngle(double currentAngle) 
            {   
                if (firstCall_) 
                {
                    // Первый вызов - просто инициализируем
                    prevAngle_ = currentAngle;
                    firstCall_ = false;
                    
                    // Преобразуем в [0, 2π]
                    double angle0To2Pi = (currentAngle < 0) ? currentAngle + 2*M_PI : currentAngle;
                    return angle0To2Pi; // angle0To2Pi для [0, 2π]
                }
                
                // Вычисляем разницу между текущим и предыдущим углом
                double diff = currentAngle - prevAngle_;
                
                // Корректируем разрыв через границу ±π
                if (diff > M_PI)  // Большой положительный скачок - вероятно, переход через -π
                {                    
                    fullRevolutions_--;
                }
                else if (diff < -M_PI) // Большой отрицательный скачок - вероятно, переход через +π
                {                    
                    fullRevolutions_++;
                }
                                
                prevAngle_ = currentAngle;  // Обновляем предыдущий угол
                double continuousAngle = currentAngle + 2*M_PI * fullRevolutions_; // Вычисляем непрерывный угол
                return continuousAngle;
            }

            double GetPitch(const ignition::math::v6::Quaterniond& q)
            {
                ignition::math::Vector3d euler = q.Euler();
                double roll = euler.X();   // Вращение вокруг оси X
                return getContinuousAngle(roll);
            }

        public:
            void Load(physics::ModelPtr model, sdf::ElementPtr _sdf) override
            {
                node_ = gazebo_ros::Node::Get(_sdf);

                pendulum_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("pendulum_state", 10);
                
                torque_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
                    "/pendulum_torque",
                    1,
                    [this](const std_msgs::msg::Float64::SharedPtr msg) {
                        target_torque_ = msg->data;
                    });
                model_ = model;
                RCLCPP_INFO(node_->get_logger(), "Available links in model:");
                for (const auto &link : model_->GetLinks())
                {
                    RCLCPP_INFO(node_->get_logger(), " - %s", link->GetName().c_str());
                }
                for (const auto &joint : model_->GetJoints())
                {
                    RCLCPP_INFO(node_->get_logger(), " - %s", joint->GetName().c_str());
                }
                pendulum_joint_ = model_->GetJoint("pendulum_joint");
                pendulum_link_ = model_->GetLink("pendulum_link");
                
                update_connection_ = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&PendulumSimplePlugin::OnUpdate, this));
                initialized_ = true;
                RCLCPP_INFO(node_->get_logger(), "PendulumSimplePlugin успешно загружен!\r\n");
            }

            void OnUpdate() 
            {                
                pendulum_joint_->SetForce(0, target_torque_);
                
                auto pendulum_link_angular_vel = pendulum_link_->WorldAngularVel();
                auto world_pendulum_angular_vel = pendulum_link_angular_vel.X();
                
                auto pendulum_link_pose = pendulum_link_->WorldCoGPose();
                auto pitch = GetPitch(pendulum_link_pose.Rot());
                
                // Публикуем угол в топик
                auto pendulum_state_msg = sensor_msgs::msg::JointState();
                const auto sim_time = model_->GetWorld()->SimTime();

                pendulum_state_msg.header.stamp.sec = sim_time.sec;
                pendulum_state_msg.header.stamp.nanosec = sim_time.nsec;

                pendulum_state_msg.name = {"pendulumAngle"};
                pendulum_state_msg.position = {pitch};
                pendulum_state_msg.velocity = {world_pendulum_angular_vel};
                pendulum_state_msg.effort = {target_torque_};

                pendulum_state_pub_->publish(pendulum_state_msg);      
            }

            void Reset() override
            {
                std::cout << "Plugin reset by simulation" << std::endl;
                firstCall_ = true;
                fullRevolutions_ = 0;
            }
    };
    GZ_REGISTER_MODEL_PLUGIN(PendulumSimplePlugin)
}