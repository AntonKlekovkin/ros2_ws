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
#include <gazebo/common/Time.hh>

namespace gazebo
{
    class QuaternionTracker
    {
    private:
        ignition::math::v6::Quaterniond prev_q_;
        ignition::math::v6::Quaterniond prev_q_inv_;
        bool initialized_ = false;
        double accumulated_yaw_ = 0.0;
        
    public:
        double getContinuousYaw(const ignition::math::v6::Quaterniond& q)
        {
            if (!initialized_) {
                prev_q_ = q;
                prev_q_inv_ = prev_q_.Inverse();
                initialized_ = true;
                accumulated_yaw_ = 0.0;
                return 0.0;
            }
            
            // Вычисляем относительный поворот
            ignition::math::v6::Quaterniond delta_q = prev_q_inv_ * q;
            
            // Получаем угол поворота вокруг оси Z из относительного кватерниона
            double delta_yaw = delta_q.Yaw();
            
            // Нормализуем дельту
            if (delta_yaw > M_PI) delta_yaw -= 2*M_PI;
            if (delta_yaw < -M_PI) delta_yaw += 2*M_PI;
            
            // Накопливаем yaw
            accumulated_yaw_ += delta_yaw;
            
            // Обновляем предыдущие значения
            prev_q_ = q;
            prev_q_inv_ = prev_q_.Inverse();
            
            return accumulated_yaw_;
        }
        
        void reset()
        {
            initialized_ = false;
            accumulated_yaw_ = 0.0;
        }
    };

    class AngleTracker
    {
    private:
        bool initialized_ = false;
        double prevAngle_ = 0.0;
        int fullRevolutions_ = 0;
    
    public:
        // Сброс трекера
        void reset()
        {
            initialized_ = false;
            prevAngle_ = 0.0;
            fullRevolutions_ = 0;
        }
        
        // Получение непрерывного угла
        double getContinuous(double currentAngle)
        {
            // currentAngle должен быть в диапазоне [-π, π]
            
            if (!initialized_)
            {
                prevAngle_ = currentAngle;
                initialized_ = true;
                fullRevolutions_ = 0;
                
                // Возвращаем угол в [0, 2π] для первого вызова
                return (currentAngle < 0) ? currentAngle + 2*M_PI : currentAngle;
            }
            
            // Вычисляем разницу
            double diff = currentAngle - prevAngle_;
            
            // Корректируем разрыв через границу ±π
            if (diff > M_PI)
            {
                fullRevolutions_--;
            }
            else if (diff < -M_PI)
            {
                fullRevolutions_++;
            }
            
            prevAngle_ = currentAngle;
            
            // Возвращаем непрерывный угол
            return currentAngle + 2*M_PI * fullRevolutions_;
        }
        
        // Получить текущий непрерывный угол без обновления
        double getCurrentContinuous(double currentAngle) const
        {
            if (!initialized_)
            {
                return (currentAngle < 0) ? currentAngle + 2*M_PI : currentAngle;
            }
            return currentAngle + 2*M_PI * fullRevolutions_;
        }
        
        // Получить количество полных оборотов
        int getRevolutions() const
        {
            return fullRevolutions_;
        }
    };

    class SpheroPlugin : public ModelPlugin
    {
        private:
            physics::ModelPtr model_;
            physics::JointPtr pendulum_joint_{nullptr}, sphere_joint_{nullptr}, rotor_joint_{nullptr};
            physics::LinkPtr pendulum_link_{nullptr}, sphere_link_{nullptr}, rotor_link_{nullptr};
            event::ConnectionPtr update_connection_;
            gazebo_ros::Node::SharedPtr node_;

            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pendulum_torque_sub_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rotor_torque_sub_;

            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr angle_pub_;

            double target_pendulum_torque_ = 0.0;
            double target_rotor_torque_ = 0.0;  

            AngleTracker pitch_tracker_;
            AngleTracker yaw_tracker_;
            

            double GetPitch(const ignition::math::v6::Quaterniond& q)
            {
                // Получаем матрицу из кватерниона
                std::array<std::array<double, 3>, 3> R;
                double w = q.W(), x = q.X(), y = q.Y(), z = q.Z();
                
                R[0][0] = 1 - 2*(y*y + z*z);  R[0][1] = 2*(x*y - w*z);  R[0][2] = 2*(x*z + w*y);
                R[1][0] = 2*(x*y + w*z);      R[1][1] = 1 - 2*(x*x + z*z);  R[1][2] = 2*(y*z - w*x);
                R[2][0] = 2*(x*z - w*y);      R[2][1] = 2*(y*z + w*x);  R[2][2] = 1 - 2*(x*x + y*y);
                
                return pitch_tracker_.getContinuous(atan2(-R[2][0], R[2][2]));                
            }


        public:
            void Load(physics::ModelPtr model, sdf::ElementPtr _sdf) override
            {                
                rclcpp::sleep_for(std::chrono::milliseconds(1000));
                node_ = gazebo_ros::Node::Get(_sdf);

                angle_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("sphero_states", 10);
                
                pendulum_torque_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
                    "/pendulum_torque",
                    10,
                    [this](const std_msgs::msg::Float64::SharedPtr msg) {
                        target_pendulum_torque_ = msg->data;
                    });

                rotor_torque_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
                    "/rotor_torque",  
                    10,
                    [this](const std_msgs::msg::Float64::SharedPtr msg) {
                        target_rotor_torque_ = msg->data;
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

                sphere_link_ = model->GetLink("base_link");
                
                rotor_joint_ = model_->GetJoint("rotor_joint");  
                rotor_link_ = model_->GetLink("rotor_link");

                update_connection_ = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&SpheroPlugin::OnUpdate, this));
                //initialized_ = true;
                gzmsg << "SpheroPlugin успешно загружен!" << std::endl;                
            }

            void OnUpdate() 
            {
                pendulum_joint_->SetForce(0, -target_pendulum_torque_);
                
                // Получаем текущую скорость ротора
                double current_rotor_vel = rotor_joint_->GetVelocity(0);
                double rotor_damping_coefficient_ = 0.0001;
                // Вычисляем момент с демпфированием
                double damping_torque = -rotor_damping_coefficient_ * current_rotor_vel;
                double total_torque = target_rotor_torque_ + damping_torque;
                        
                rotor_joint_->SetForce(0, total_torque);
                
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

                auto sphere_angle_yaw = yaw_tracker_.getContinuous(pendulum_link_pose.Rot().Yaw());
                auto sphere_ang_vel_yaw = sphere_link_->WorldAngularVel().Z();
                
                //world_pendulum_angle = pendulum_link_pose.Rot().Pitch();
                auto pitch = GetPitch(pendulum_link_pose.Rot());
                
                double rotor_angle = 0.0;
                double rotor_velocity = 0.0;
                
                if (rotor_joint_) 
                {
                    rotor_angle = rotor_joint_->Position(0);
                    rotor_velocity = rotor_joint_->GetVelocity(0);
                }

                // Публикуем угол в топик
                auto angle_msg = sensor_msgs::msg::JointState();
                const auto sim_time = model_->GetWorld()->SimTime();

                angle_msg.header.stamp.sec = sim_time.sec;
                angle_msg.header.stamp.nanosec = sim_time.nsec;

                angle_msg.name = {"pendulumAngle", "pendulumX", "pendulumY", "rotor", "sphereYaw"};
                angle_msg.position = {pitch, xPendulum, yPendulum, rotor_angle, sphere_angle_yaw};
                angle_msg.velocity = {world_pendulum_angular_vel, xLinVel, yLinVel, rotor_velocity, sphere_ang_vel_yaw};
                angle_msg.effort = {target_pendulum_torque_, xForce, yForce, target_rotor_torque_, 0.0};
                angle_pub_->publish(angle_msg);                
            }

            void Reset() override
            {
                std::cout << "Plugin reset by simulation" << std::endl;
                // firstCall_ = true;
                // fullRevolutions_ = 0;
                pitch_tracker_.reset();
                yaw_tracker_.reset();
                target_pendulum_torque_ = 0.0;
                target_rotor_torque_ = 0.0; 
            }
    };
    GZ_REGISTER_MODEL_PLUGIN(SpheroPlugin)
}