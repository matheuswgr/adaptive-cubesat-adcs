#ifndef __plugin_gyroscope_h
#define __plugin_gyroscope_h

#include "transducer.h"
#include "smartdata.h"
#include "units.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <string>
#include <memory>
#include "observed.h"
#include "observer.h"

class PluginGyroscope : public Transducer<RADPS, float>, public rclcpp::Node
{
    public:
        static const bool active = true;

        float value;
        SmartData<float>* smartdata;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription;

    public: 
        PluginGyroscope(std::string nodeName) : Node(nodeName)
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
            this->subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(topicName, 10,std::bind(&PluginGyroscope::handleInterrupt, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribing to: " + topicName);
        }

        void handleInterrupt(std_msgs::msg::Float32MultiArray::SharedPtr message)
        {
            this->value = message->data[6];
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