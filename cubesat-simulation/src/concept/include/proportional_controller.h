#ifndef __proportional_controller_h
#define __proportional_controller_h

#include "controller.h"
#include "smartdata.h"
#include "local_smartdata.h"

#include <list>
#include <vector>

template<typename SDIN, typename SDOUT>
class ProportionalController : public Controller
{
    float gain = 0.01;
    float controlSignal;
    std::vector<std::shared_ptr<SDIN>> inputSmartData; 
    std::vector<std::shared_ptr<SDOUT>> outputSmartData; 
    public:
        ProportionalController(){}

        void setup(std::list<std::shared_ptr<SDIN>> inputSmartData, std::list<std::shared_ptr<SDOUT>> outputSmartData)
        {
            for (typename std::list<std::shared_ptr<SDIN>>::iterator it = inputSmartData.begin(); it != inputSmartData.end(); ++it)
            {
                this->inputSmartData.push_back((*it));
            } 

            for (typename std::list<std::shared_ptr<SDOUT>>::iterator it = outputSmartData.begin(); it != outputSmartData.end(); ++it)
            {
                this->outputSmartData.push_back((*it));
            }      
        }
        
        void control()
        {
            for(int i = 0; i<this->inputSmartData.size(); i++)
            {          
                this->controlSignal = this->controlSignal + this->gain*(1 - inputSmartData.at(i)->smartdataValue); 
                
                if (this->controlSignal >= 1.0)
                {
                    this->controlSignal = 1.0;
                }
                else if (this->controlSignal <= -1.0)
                {
                    this->controlSignal = -1.0;
                }
            }
                        
            for(int i = 0; i<this->outputSmartData.size(); i++)
            {
                this->outputSmartData.at(i)->setValue(this->controlSignal);
            }
        }

};

#endif