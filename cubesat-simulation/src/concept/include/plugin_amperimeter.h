#ifndef __plugin_amperimeter_h
#define __plugin_amperimeter_h

#include "transducer.h"
#include "smartdata.h"
#include "../../../include/units.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <string>
#include <memory>
#include "observed.h"
#include "observer.h"

class PluginAmperimeter : public Transducer<A, float>, public rclcpp::Node
{
    public:
        static const bool active = true;

        float value;
        SmartData* smartdata;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription;

    public: 
        PluginAmperimeter(std::string nodeName) : Node(nodeName)
        {
            return;
        }

        float sense()
        {
            return this->value;
        }

        void attach(SmartData* smartdata)
        {
            this->smartdata = smartdata;
        }

        void initialize(std::string topicName)
        {
            this->subscription = this->create_subscription<std_msgs::msg::Float32>(topicName, 10,std::bind(&PluginAmperimeter::handleInterrupt, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribing to: " + topicName);
        }

        void handleInterrupt(std_msgs::msg::Float32::SharedPtr message)
        {
            RCLCPP_INFO(this->get_logger(), "I'm about to update");
            this->value = message->data;
            this->smartdata->setValue(this->value);
            this->smartdata->update();
        }

        void handleValueUpdate()
        {
            return;
        }
};

#endif