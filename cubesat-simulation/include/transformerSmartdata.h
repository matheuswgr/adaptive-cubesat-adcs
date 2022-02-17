#ifndef __transformer_smartdata_h
#define __transformer_smartdata_h

#include <list>

#include "smartdata.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

template<typename Transformation, typename Value>
class TransformerSmartData :  public rclcpp::Node, public SmartData
{
    friend Transformation;

    protected:
        static const unsigned long UNIT = Transformation::UNIT;

        Value* smartdataValue;

    private:

        Transformation* transfomation;
        
        std::list<std::shared_ptr<SmartData>>* inputSmartData;
        std::list<std::shared_ptr<SmartData>>* outputSmartData;

        rclcpp::TimerBase::SharedPtr refreshTimer;
        
    public:
        TransformerSmartData(int period, std::string nodeName, Coordinates* location, std::list<std::shared_ptr<SmartData>>* inputSmartData, std::list<std::shared_ptr<SmartData>>* outputSmartData) : Node(nodeName)
        {
            this->transfomation = new Transformation();
            this->inputSmartData = inputSmartData;
            this->outputSmartData = outputSmartData;
            this->period = period;
            this->locationCoordinates = location;

            this->update();

            this->refreshTimer = this->create_wall_timer(std::chrono::milliseconds(this->period), std::bind(&TransformerSmartData<Transformation, Value>::update, this));
        }

        Value* value()
        {
            return this->smartdataValue;
        }

        Value* update()
        {
            this->transfomation->transform(inputSmartData,outputSmartData);
            this->smartdataValue = outputSmartData;

            return this->smartdataValue;
        }
        
        void wait()
        {
            return;
        }
};

#endif