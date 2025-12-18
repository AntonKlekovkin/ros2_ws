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
    class PendulumDrivePlugin : public ModelPlugin
    {
        private:
            physics::ModelPtr model_;
            physics::JointPtr motor_joint_{nullptr}, sphere_joint_{nullptr};
            physics::LinkPtr pendulum_link_{nullptr}, sphere_link_{nullptr};
            event::ConnectionPtr update_connection_;
            gazebo_ros::Node::SharedPtr node_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr torque_sub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr angle_pub_;
            double target_torque_ = 0.0;
            bool initialized_ = false;
            double prevAngle_ = 0.0;
            bool firstCall_ = true;
            int fullRevolutions_ = 0;

            double getContinuousAngle(double currentAngle) {
                // currentAngle должен быть в диапазоне [-π, π]
                
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
                if (diff > M_PI) 
                {
                    // Большой положительный скачок - вероятно, переход через -π
                    fullRevolutions_--;
                }
                else if (diff < -M_PI) 
                {
                    // Большой отрицательный скачок - вероятно, переход через +π
                    fullRevolutions_++;
                }
                
                // Обновляем предыдущий угол
                prevAngle_ = currentAngle;
                
                // Вычисляем непрерывный угол
                double continuousAngle = currentAngle + 2*M_PI * fullRevolutions_;
                
                // Преобразуем в [0, 4π]
                return continuousAngle;
            }

            double GetPitch(const ignition::math::v6::Quaterniond& q)
            {
                // Получаем матрицу из кватерниона
                std::array<std::array<double, 3>, 3> R;
                double w = q.W(), x = q.X(), y = q.Y(), z = q.Z();
                
                R[0][0] = 1 - 2*(y*y + z*z);  R[0][1] = 2*(x*y - w*z);  R[0][2] = 2*(x*z + w*y);
                R[1][0] = 2*(x*y + w*z);      R[1][1] = 1 - 2*(x*x + z*z);  R[1][2] = 2*(y*z - w*x);
                R[2][0] = 2*(x*z - w*y);      R[2][1] = 2*(y*z + w*x);  R[2][2] = 1 - 2*(x*x + y*y);
                
                return getContinuousAngle(atan2(-R[2][0], R[2][2]));
            }

        public:
            void Load(physics::ModelPtr model, sdf::ElementPtr _sdf) override
            {
                // if (!rclcpp::ok()) 
                // {
                //     rclcpp::init(0, nullptr);
                // }
                
                node_ = gazebo_ros::Node::Get(_sdf);

                angle_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("sphero_states", 10);
                
                torque_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
                    "/motor_torque",
                    10,
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
                motor_joint_ = model_->GetJoint("sphere_joint");
                pendulum_link_ = model_->GetLink("base_link");
                
                update_connection_ = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&PendulumDrivePlugin::OnUpdate, this));
                initialized_ = true;
                gzmsg << "PendulumDrivePlugin успешно загружен!" << std::endl;
            }

            void OnUpdate() 
            {
                motor_joint_->SetForce(1, -target_torque_);
                
                //double world_pendulum_angle = 0.0;
                
                auto pendulum_link_pose = pendulum_link_->WorldCoGPose();
                double xPendulum = pendulum_link_pose.Pos().X();
                double yPendulum = pendulum_link_pose.Pos().Y();

                auto pendulumLinearVel = pendulum_link_->WorldLinearVel();

                double xLinVel = pendulumLinearVel.X();
                double yLinVel = pendulumLinearVel.Y();

                double xForce = pendulum_link_->WorldForce().X();
                double yForce = pendulum_link_->WorldForce().Y();
                
                auto pendulum_link_angular_vel = pendulum_link_->WorldAngularVel();
                auto world_pendulum_angular_vel = pendulum_link_angular_vel.Y();
                
                //world_pendulum_angle = pendulum_link_pose.Rot().Pitch();
                auto pitch = GetPitch(pendulum_link_pose.Rot());
                
                // Публикуем угол в топик
                auto angle_msg = sensor_msgs::msg::JointState();
                const auto sim_time = model_->GetWorld()->SimTime();

                angle_msg.header.stamp.sec = sim_time.sec;
                angle_msg.header.stamp.nanosec = sim_time.nsec;

                angle_msg.name = {"pendulumAngle", "pendulumX", "pendulumY"};
                angle_msg.position = {pitch, xPendulum, yPendulum};
                angle_msg.velocity = {world_pendulum_angular_vel, xLinVel, yLinVel};
                angle_msg.effort = {target_torque_, xForce, yForce};
                angle_pub_->publish(angle_msg);                
            }

            void Reset() override
            {
                std::cout << "Plugin reset by simulation" << std::endl;
                firstCall_ = true;
                fullRevolutions_ = 0;
            }
    };
    GZ_REGISTER_MODEL_PLUGIN(PendulumDrivePlugin)
}