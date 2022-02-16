#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <functional>
#include <memory>

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
            event::ConnectionPtr updateConnection;
            gazebo_ros::Node::SharedPtr ros_node;
            physics::JointPtr joint;
            rclcpp::TimerBase::SharedPtr timer;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_publisher;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_publisher;
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr dutycycle_subscriber;

        public: 
            DcMotorPlugin(){}
        
            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
            {
                this->model = _model;
                this->joint = this->model->GetJoint("reaction_wheel_z_axis_shaft");
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&DcMotorPlugin::OnUpdate, this));
                this->last_update_time = this->model->GetWorld()->SimTime().Float();
                this->ros_node = gazebo_ros::Node::Get(_sdf);

                this->timer = this->ros_node->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&DcMotorPlugin::Publish, this));

                this->velocity_publisher = this->ros_node->create_publisher<std_msgs::msg::Float32>("motor_velocity", 10);
                this->current_publisher = this->ros_node->create_publisher<std_msgs::msg::Float32>("motor_current", 10);

                this->dutycycle_subscriber = this->ros_node->create_subscription<std_msgs::msg::Int32>("duty_cycle", 10,std::bind(&DcMotorPlugin::OnDutyCycleUpdate, this, std::placeholders::_1));

                this->dc_motor = new DcMotor(1, 0.001, 0.005,0.1,12);
            }

            void OnUpdate()
            {  
                time_step = this->model->GetWorld()->SimTime().Float() - last_update_time;
                last_update_time = this->model->GetWorld()->SimTime().Float();
                float velocity = (float)this->joint->GetVelocity(0);
                dc_motor->propagateState(time_step,velocity);
                this->joint->SetForce(0,dc_motor->torque);
            }

            void Publish()
            {
                auto message = std_msgs::msg::Float32();
                message.data = dc_motor->velocity;

                RCLCPP_INFO(this->ros_node->get_logger(), "Publishing velocity: '%f'", message.data);
                velocity_publisher->publish(message);

                message.data = dc_motor->current;
                RCLCPP_INFO(this->ros_node->get_logger(), "Publishing current: '%f'", message.data);
                current_publisher->publish(message);
            }

            void OnDutyCycleUpdate(const std_msgs::msg::Int32::SharedPtr msg)
            {
                dc_motor->duty_cycle = ((float) msg->data)/100.0;
            }
    };
    GZ_REGISTER_MODEL_PLUGIN(DcMotorPlugin)
}