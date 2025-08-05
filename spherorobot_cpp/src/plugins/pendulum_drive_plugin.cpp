#include <gazebo/common/Plugin.hh>
#include <ignition/math/Pose3.hh>
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
            double angle = -0.075;
            bool initialized_ = false;

        public:
            void Load(physics::ModelPtr model, sdf::ElementPtr _sdf) override
            {
                if (!rclcpp::ok()) 
                {
                    rclcpp::init(0, nullptr);
                }
                
                node_ = gazebo_ros::Node::Get(_sdf);

                angle_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
                
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
                motor_joint_ = model_->GetJoint("shaft_joint");
                pendulum_link_ = model_->GetLink("base_link");
                sphere_link_ = model_->GetLink("shaft_link");

                // if (!motor_joint_ || !sphere_joint_ || !pendulum_link_ || !sphere_link_) 
                // {
                //     gzerr << "Не удалось получить все необходимые joints/links!" << std::endl;
                //     if (!motor_joint_) gzerr << " - shaft_joint не найден" << std::endl;
                //     if (!sphere_joint_) gzerr << " - sphere_joint не найден" << std::endl;
                //     if (!pendulum_link_) gzerr << " - base_link не найден" << std::endl;
                //     if (!sphere_link_) gzerr << " - sphere_link не найден" << std::endl;
                //     return;
                // }
                update_connection_ = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&PendulumDrivePlugin::OnUpdate, this));
                initialized_ = true;
                gzmsg << "PendulumDrivePlugin успешно загружен!" << std::endl;
            }

            void OnUpdate() 
            {
                // if (!initialized_) return;
                motor_joint_->SetForce(0, target_torque_);
                double shaft_angle = motor_joint_->Position(0);
                double delta_angle = shaft_angle-angle;

                ignition::math::Pose3d pendulum_link_pose;
                ignition::math::Pose3d sphere_link_pose;
                double world_pendulum_angle = 0.0;
                double world_sphere_angle = 0.0;

                pendulum_link_pose = pendulum_link_->WorldCoGPose();
                double x = pendulum_link_pose.Pos().X();
                double y = pendulum_link_pose.Pos().Y();
                world_pendulum_angle = pendulum_link_pose.Rot().Pitch();

                sphere_link_pose = sphere_link_->WorldCoGPose();
                world_sphere_angle = sphere_link_pose.Rot().Pitch();
            
                // Публикуем угол в топик
                auto angle_msg = sensor_msgs::msg::JointState();
                const auto sim_time = model_->GetWorld()->SimTime();

                angle_msg.header.stamp.sec = sim_time.sec;
                angle_msg.header.stamp.nanosec = sim_time.nsec;

                angle_msg.name = {"pendulum_joint", "sphere_joint"};
                angle_msg.position = {x, y};
                angle_msg.velocity = {world_pendulum_angle,world_sphere_angle};
                angle_pub_->publish(angle_msg);
                angle = shaft_angle;
            }
    };
    GZ_REGISTER_MODEL_PLUGIN(PendulumDrivePlugin)
}