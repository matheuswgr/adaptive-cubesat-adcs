#ifndef __plugin_transducer_h
#define __plugin_transducer_h

#include "transducer.h"
#include "smartdata.h"
#include "rclcpp/rclcpp.hpp"

template<unsigned long _UNIT, typename Value, typename Message>
class PluginTransducer: public Transducer<_UNIT,Value>
{
    public:
        static const unsigned long UNIT = _UNIT;
        static const bool active = true;
        rclcpp::Subscription<Message>::SharedPtr subscription;
        rclcpp::Publisher<Message>::SharedPtr publisher;
        SmartData* sd;

    public:
        PluginTransducer(SmartData* sd)
        {
            this->sd = sd;
        }

        void handleValueUpdate(std_msgs::msg::Float::SharedPtr msg)
        {
            sd->smartdataValue = msg.data;
        }

        Value sense()
        {
            return sd->smartdataValue;
        }

        void actuate(const Value &value)
        {
            auto message = std_msgs::msg::Float32();
            message.data = *value;

            publisher->publish(message);
        }
};

#endif