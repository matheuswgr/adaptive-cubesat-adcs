#ifndef __responsive_smartdata_h
#define __responsive_smartdata_h

#include "smartdata.h"
#include "transformerSmartdata.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

template<typename Transducer, typename Value>
class ResponsiveSmartData :  public rclcpp::Node, public SmartData
{
    friend Transducer;
    
    template<typename Transformer, typename ValueList>
    friend class TransformerSmartData; 

    protected:
        static const unsigned long UNIT = Transducer::UNIT;
        static const bool active = Transducer::active;  

        //Value smartdataValue;

    private:

        Transducer* transducer;
        
        int device;
        unsigned long expiry;        
        int mode;

        rclcpp::TimerBase::SharedPtr refreshTimer;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription;
        
    public:
        ResponsiveSmartData(unsigned int dev, int expiry, int mode, int period, std::string nodeName, Coordinates* location, std::string commandsTopic) : Node(nodeName)
        {
            this->transducer = new Transducer();
            this->smartdataValue = transducer->sense();
            this->mode = mode;
            this->device = dev;
            this->expiry = expiry;
            this->period = period;
            this->locationCoordinates = location;
            
            this->refreshTimer = this->create_wall_timer(std::chrono::milliseconds(this->period), std::bind(&ResponsiveSmartData<Transducer, Value>::update, this));
            
            if(mode == 1)
            {
                this->publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(nodeName, 10);
            }
            else if (mode == 2)
            {
                this->publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(nodeName, 10);
                this->subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(commandsTopic, 10,std::bind(&ResponsiveSmartData<Transducer, Value>::handleRemoteUpdate, this, std::placeholders::_1));
            }
            
        }

        Value value()
        {
            return this->smartdataValue;
        }

        Value update()
        {
            if (mode == 1 || mode == 2)
            {  
                if (mode == 1)
                {
                    this->smartdataValue = this->transducer->sense();
                }
                else
                {
                    this->transducer->actuate(this->smartdataValue);
                }
                
                auto message = std_msgs::msg::Float32MultiArray();
                
                this->buildPacket(&message);

                this->publisher->publish(message);
            }

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
        
        void decodePacket(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            this->smartdataValue = msg->data[6];
            this->expiry = msg->data[7];
        }

        void handleRemoteUpdate(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            this->decodePacket(msg);
            this->update();
        }
};

#endif