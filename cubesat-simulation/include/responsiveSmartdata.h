#ifndef __responsive_smartdata_h
#define __responsive_smartdata_h

#include "smartdata.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

template<typename Transducer, typename Value>
class ResponsiveSmartData :  public rclcpp::Node, public SmartData<Transducer,Value>
{
    friend Transducer;
    friend class TransformerSmartData;  

    private:

        Transducer* transducer;
        
        int device;
        unsigned long expiry;        
        int mode;

        rclcpp::TimerBase::SharedPtr refreshTimer;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher;
        
    public:
        ResponsiveSmartData(unsigned int dev, int expiry, int mode, int period, std::string nodeName, Coordinates* location) : Node(nodeName)
        {
            this->transducer = new Transducer();
            this->smartdataValue = transducer->sense();
            this->mode = mode;
            this->device = dev;
            this->expiry = expiry;
            this->period = period;
            this->locationCoordinates = location;
            
            this->refreshTimer = this->create_wall_timer(std::chrono::milliseconds(this->period), std::bind(&ResponsiveSmartData<Transducer, Value>::update, this));
            this->publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(nodeName, 10);
        }

        Value value()
        {
            return this->smartdataValue;
        }

        Value update()
        {
            this->smartdataValue = this->transducer->sense();
            auto message = std_msgs::msg::Float32MultiArray();
            
            this->buildPacket(&message);

            this->publisher->publish(message);

            return this->smartdataValue;
        }
        
        void wait()
        {
            return;
        }
    
    private:
        void buildPacket(std_msgs::msg::Float32MultiArray *message)
        {
            message->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
            
            message->layout.dim[0].size = 8;
            message->layout.dim[0].stride = 1;
            message->layout.dim[0].label = "smartdata";

            message->data.clear();
            message->data.resize(8); 

            message->data[0] = (float)this->locationCoordinates->x;
            message->data[1] = (float)this->locationCoordinates->y;
            message->data[2] = (float)this->locationCoordinates->z;
            message->data[3] = (float)this->device;
            message->data[4] = (float)this->currentTime;
            message->data[5] = (float)this->UNIT;
            message->data[6] = (float)this->smartdataValue;
            message->data[7] = (float)this->expiry;
        }
};

#endif