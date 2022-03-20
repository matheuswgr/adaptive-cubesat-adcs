#ifndef __controller_smartdata_h
#define __controller_smartdata_h

#include "smartdata.h"
#include "controller.h"
#include "local_smartdata.h"
#include "rclcpp/rclcpp.hpp"
#include "observer.h"
#include "observed.h"

#include <list>
#include <vector>

template <typename Controller, typename SDIN, typename SDOUT>
class ControllerSmartData 
    : public SmartData, public Controller, public Observed<SDOUT> 
{
    private:
        std::list<std::shared_ptr<SDIN>> inputSmartData; 
        std::list<std::shared_ptr<SDOUT>> outputSmartData; 
        std::vector<bool> freshnessList;

        std::shared_ptr<Controller> controller;

    public:
        ControllerSmartData(std::list<std::shared_ptr<SDIN>> &inputSmartData, std::list<std::shared_ptr<SDOUT>> &outputSmartData)
        {
            this->inputSmartData = inputSmartData;
            this->outputSmartData = outputSmartData;

            int i = 0;
            for (typename std::list<std::shared_ptr<SDIN>>::iterator it = this->inputSmartData.begin(); it != this->inputSmartData.end(); ++it)
            {
                (*it)->startObserving(this);
                (*it)->giveId(i++);
                freshnessList.push_back(false);
            }        
            
            for (typename std::list<std::shared_ptr<SDOUT>>::iterator it = this->outputSmartData.begin(); it != this->outputSmartData.end(); ++it)
            {
                this->startObserving((*it).get());
            }  

            this->controller = std::make_shared<Controller>();
            this->controller->setup(inputSmartData, outputSmartData);
        }

        float value()
        {
            return this->outputSmartData.front()->smartdataValue;
        }
        
        float update()
        {
            for(int i = 0; i<this->freshnessList.size(); i++)
            {
                this->freshnessList[i] = false;
            }

            this->controller->control();
            this->notifyObservers();
            return 0.1;
        }

        void wait()
        {
            return;
        }

        void notify()
        {
            //RCLCPP_INFO(this->inputSmartData.front()->transducer->get_logger(), "I'm being notified");
        }

        void notify(int id)
        {
            //RCLCPP_INFO(this->inputSmartData.front()->transducer->get_logger(), "I'm being notified by %d",id);
            this->freshnessList[id] = true;

            bool timeToUpdate = true;
            for(int i = 0; i<this->freshnessList.size(); i++)
            {
                if (!this->freshnessList[i])
                {
                    timeToUpdate = false;
                }
            }

            if (timeToUpdate)
            {
                this->update();
            }
        }
};

#endif