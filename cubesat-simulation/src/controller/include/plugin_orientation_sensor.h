#ifndef __plugin_orientation_sensor_h
#define __plugin_orientation_sensor_h

#include "transducer.h"
#include "smartdata.h"
#include "units.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <string>
#include <memory>
#include "observed.h"
#include "observer.h"
#include <eigen3/Eigen/Geometry>

class PluginOrientationSensor : public Transducer<QUAT, Eigen::Quaternion<float>>, public rclcpp::Node
{
    public:
        static const bool active = true;

        Eigen::Quaternion<float> value;
        SmartData<Eigen::Quaternion<float>>* smartdata;
        rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr subscription;

    public: 
        PluginOrientationSensor(std::string nodeName) : Node(nodeName)
        {
            return;
        }

        Eigen::Quaternion<float> sense()
        {
            return this->value;
        }

        void attach(SmartData<Eigen::Quaternion<float> >* smartdata)
        {
            this->smartdata = smartdata;
        }

        void initialize(std::string topicName)
        {
            this->subscription = this->create_subscription<geometry_msgs::msg::Quaternion>(topicName, 10,std::bind(&PluginOrientationSensor::handleInterrupt, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribing to: " + topicName);
        }

        void handleInterrupt(geometry_msgs::msg::Quaternion::SharedPtr message)
        {
            this->value.w() = message->w;
            this->value.x() = message->x;
            this->value.y() = message->y;
            this->value.z() = message->z;

            //RCLCPP_INFO(this->get_logger(), "Scalar: %f", this->value.w());

            this->smartdata->setValue(this->value);
            this->smartdata->update();
        }

        void handleValueUpdate()
        {
            return;
        }
};

#endif