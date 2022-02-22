#ifndef __plugin_motor_driver_h
#define __plugin_motor_driver_h

#include "transducer.h"
#include "../../../include/units.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <string>
#include <memory>

class PluginMotorDriver : public Transducer<D, float>, public rclcpp::Node
{
    public:
        static const bool active = true;

        float value;
        SmartData* smartdata;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher;

    public: 
        PluginMotorDriver(std::string nodeName) : Node(nodeName)
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
            this->publisher = this->create_publisher<std_msgs::msg::Float32>(topicName, 10);
        }

        void handleValueUpdate()
        {
            RCLCPP_INFO(this->get_logger(), "I'm being updated");
            auto message = std_msgs::msg::Float32();
            message.data = this->smartdata->value();
            this->publisher->publish(message);
            this->smartdata->update();
        }
};

#endif