#ifndef __remote_smartdata_h
#define __remote_smartdata_h

#include "smartdata.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

template<typename Transducer, typename Value>
class RemoteSmartData :  public rclcpp::Node, public SmartData<Transducer,Value>
{
    friend Transducer;
    friend class TransformerSmartData; 

    private:

        Transducer* transducer;
        
        int device;
        unsigned long expiry;        
        int mode;
        std::string type;

        Region *interestRegion;

        rclcpp::TimerBase::SharedPtr refreshTimer;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription;
        
    public:
        RemoteSmartData(Region *interestRegion, int expiry, int period, std::string nodeName, Coordinates* location, std::string topicRegistered, std::string type) : Node(nodeName)
        {
            this->transducer = new Transducer();
            this->smartdataValue = transducer->sense();
            this->interestRegion = interestRegion;
            this->expiry = expiry;
            this->period = period;
            this->locationCoordinates = location;
            this->type = type;
            RCLCPP_INFO(this->get_logger(), "Updated to: '%f'", 0.0);

            if (type == "sensor")
            {
                this->subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(topicRegistered, 10,std::bind(&RemoteSmartData<Transducer, Value>::handleRemoteUpdate, this, std::placeholders::_1));
                        RCLCPP_INFO(this->get_logger(), "Updated to: '%f'",0.1);

            }
            else if (type == "actuator")
            {
                this->refreshTimer = this->create_wall_timer(std::chrono::milliseconds(this->period), std::bind(&RemoteSmartData<Transducer, Value>::update, this));
                this->publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(topicRegistered, 10);
            }
        }

        Value value()
        {
            return this->smartdataValue;
        }

        Value update()
        {
            if (this->type != "sensor")
            {
                auto message = std_msgs::msg::Float32MultiArray();
            
                this->encodePacket(&message);

                this->publisher->publish(message);
            }

            RCLCPP_INFO(this->get_logger(), "Updated to: '%f'", this->smartdataValue);

            return this->smartdataValue;
        }
        
        void wait()
        {
            return;
        }
    
    private:
        void encodePacket(std_msgs::msg::Float32MultiArray *message)
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
        
        void decodePacket(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            this->smartdataValue = msg->data[6];
            this->expiry = msg->data[7];
        }

        void handleRemoteUpdate(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Updated to: '%f'",0.2);
            this->decodePacket(msg);
            this->update();
        }
};

#endif