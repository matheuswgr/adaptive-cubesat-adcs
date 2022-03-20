#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <random>

#include "dc_motor_model.cc"

namespace gazebo
{
    class DcMotorPlugin: public ModelPlugin
    {
        private:
            DcMotor *dc_motor;
            physics::ModelPtr model;
            float last_update_time;
            float time_step;
            float elapsed_time;
            event::ConnectionPtr updateConnection;
            gazebo_ros::Node::SharedPtr ros_node;
            physics::JointPtr joint;
            rclcpp::TimerBase::SharedPtr timer;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_publisher;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_publisher;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dutycycle_subscriber;
            std::string pluginName;
            std::string jointName;
            std::default_random_engine velocityNoiseGenerator;
            std::normal_distribution<float> velocityNoise;

        public: 
            DcMotorPlugin(){}
        
            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
            {
                velocityNoise = std::normal_distribution<float>(0.0,40);
                this->model = _model;
                this->pluginName = _sdf->GetAttribute("name")->GetAsString();

                this->jointName = _sdf->Get<std::string>("motor_shaft_joint");


                this->joint = this->model->GetJoint(this->jointName);
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&DcMotorPlugin::OnUpdate, this));
                this->last_update_time = this->model->GetWorld()->SimTime().Float();
                this->ros_node = gazebo_ros::Node::Get(_sdf);
                RCLCPP_INFO(this->ros_node->get_logger(),"%s", this->jointName);

                this->velocity_publisher = this->ros_node->create_publisher<std_msgs::msg::Float32>(this->jointName + "/velocity", 10);
                this->current_publisher = this->ros_node->create_publisher<std_msgs::msg::Float32>(this->jointName + "/current", 10);

                this->dutycycle_subscriber = this->ros_node->create_subscription<std_msgs::msg::Float32>(this->jointName + "/duty_cycle", 10,std::bind(&DcMotorPlugin::OnDutyCycleUpdate, this, std::placeholders::_1));
                RCLCPP_INFO(this->ros_node->get_logger(), "Subscribing to: " + this->jointName + "/duty_cycle");

                this->dc_motor = new DcMotor(16, 26e-6, 8.98e-4,9.02e-4,4);
                elapsed_time = 0;
            }

            void OnUpdate()
            {  
                time_step = this->model->GetWorld()->SimTime().Float() - last_update_time;
                elapsed_time = elapsed_time + time_step;
                last_update_time = this->model->GetWorld()->SimTime().Float();
                float velocity = (float)this->joint->GetVelocity(0);
                dc_motor->propagateState(time_step,velocity);
                this->joint->SetForce(0,dc_motor->torque);

                if(elapsed_time > 1.67e-3)
                {
                    elapsed_time = 0;
                    auto message = std_msgs::msg::Float32();
                    message.data = dc_motor->velocity + velocityNoise(velocityNoiseGenerator);

                    velocity_publisher->publish(message);

                    message.data = dc_motor->current;
                    current_publisher->publish(message);   
                }
            }

            void OnDutyCycleUpdate(const std_msgs::msg::Float32::SharedPtr msg)
            {
                dc_motor->duty_cycle = msg->data;
            }
    };
    GZ_REGISTER_MODEL_PLUGIN(DcMotorPlugin)
}