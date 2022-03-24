#ifndef __plugin_motor_velicity_sensor_h
#define __plugin_motor_velicity_sensor_h

#include "transducer.h"
#include "smartdata.h"
#include "units.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <string>
#include <memory>
#include "observed.h"
#include "observer.h"

class PluginMotorVelocitySensor : public Transducer<RADPS, float>, public rclcpp::Node
{
    public:
        static const bool active = true;

        float value;
        SmartData<float>* smartdata;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription;

    public: 
        PluginMotorVelocitySensor(std::string nodeName) : Node(nodeName)
        {
            return;
        }

        float sense()
        {
            return this->value;
        }

        void attach(SmartData<float>* smartdata)
        {
            this->smartdata = smartdata;
        }

        void initialize(std::string topicName)
        {
            this->subscription = this->create_subscription<std_msgs::msg::Float32>(topicName, 10,std::bind(&PluginMotorVelocitySensor::handleInterrupt, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribing to: " + topicName);
        }

        void handleInterrupt(std_msgs::msg::Float32::SharedPtr message)
        {
            this->value = message->data;
            //RCLCPP_INFO(this->get_logger(), "Scalar: %f", this->value);
            this->smartdata->setValue(this->value);
            this->smartdata->update();
        }

        void handleValueUpdate()
        {
            return;
        }
};

#endif