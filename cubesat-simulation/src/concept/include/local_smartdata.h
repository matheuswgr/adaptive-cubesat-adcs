#ifndef __local_smartdata_h
#define __local_smartdata_h

#include "smartdata.h"
#include "transducer.h"
#include "controller.h"
#include "rclcpp/rclcpp.hpp"
#include "topic_server.h"
#include "node_name_server.h"

#include "smartdata_modes.h"
#include "observer.h"
#include "observed.h"
#include <string>
#include <memory>
#include <algorithm>

/*
    Problems: 
        Got to add a setter for the smartdata interface
        Smartdata has to inherit the observer/observed or things won't work

*/

template <typename Value, typename Transducer>
class LocalSmartData : public SmartData, public Observer, public Observed<Controller>
{
    private:
        int dev;
        unsigned long expiry;
        int mode;

    public:
        std::shared_ptr<Transducer> transducer;
        Value smartdataValue;

    public:
        LocalSmartData(int dev, unsigned long expiry, unsigned long period, int mode)
        {
            this->dev = dev;
            this->expiry = expiry;
            this->period = period;
            this->mode = mode;

            std::string topicName = TopicServer::serve(this->transducer->unit, dev);
            std::string nodeName = NodeNameServer::serve(this->transducer->unit, dev);

            this->transducer = std::make_shared<Transducer>(nodeName);
            RCLCPP_INFO(this->transducer->get_logger(), "I'm being created");
            this->transducer->attach(this);
            this->transducer->initialize(topicName);
        }

        Value value()
        {
            return this->smartdataValue;
        }

        Value update()
        {
            this->expiry = this->transducer->get_clock()->now().nanoseconds() + this->period*1000000;
            RCLCPP_INFO(this->transducer->get_logger(), "I'm about to notify");
            this->notifyObserversWithId();
            return this->smartdataValue;
        }

        void setValue(Value value)
        {
            RCLCPP_INFO(this->transducer->get_logger(), "I'm being written to");
            this->smartdataValue = value;
        }

        void wait()
        {
            return;
        }

        void notify()
        {
            if (COMMANDED == mode)
            {
                this->transducer->handleValueUpdate();
            }
        }

        void spinIt()
        {
            rclcpp::spin_some(this->transducer); 
        }
};

#endif