#include "../../../include/responsiveSmartdata.h"
#include "../../../include/remoteSmartdata.h"
#include "../../../include/transformerSmartdata.h"
#include "../../../include/transducer.h"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <list>


template <unsigned long UNIT,typename Value>
class Dummy_Transducer: public Transducer<UNIT,Value>
{

public:
    static const bool active = false;

public:
    Dummy_Transducer(){}
    
    Value sense()
    {
        value++;
        return value;
    }

    void actuate(Value value)
    {
        this->value = value;
    }

private:
    Value value;
};

template <unsigned long UNIT,typename Value, typename InputType, typename OutputType>
class Dummy_Transformation
{

public:
    Dummy_Transformation(){}
    
    void transform(std::list<InputType>* inputData, std::list<OutputType>* outputData)
    {
        Value x = 0;
        for(typename std::list<InputType>::iterator it=inputData->begin(); it != inputData->end(); ++it)
        {
            x = x + (*it)->smartdataValue;
        }
        
        int i = 1;
        for(typename std::list<OutputType>::iterator it=outputData->begin(); it != outputData->end(); ++it)
        {
            (*it)->smartdataValue = x*i;
            i = i + 10;
        }
    }
};

int main(int argc, char * argv[])
{ 
    rclcpp::init(argc, argv);
    std::list<std::shared_ptr<SmartData>> inputData;
    std::list<std::shared_ptr<SmartData>> outputData;

    auto sensor = std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),1,1000,"testing_sensor_remote",new Coordinates(0,0,0),"testing_sensor","sensor");
    auto actuator = std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),1,1000,"testing_actuator_remote",new Coordinates(0,0,0),"testing_actuator_remote","actuator");
    
    inputData.push_front(sensor);
    outputData.push_front(actuator);

    auto transformation = std::make_shared<TransformerSmartData<Dummy_Transformation<1,float,std::shared_ptr<SmartData>,std::shared_ptr<SmartData>>,std::list<std::shared_ptr<SmartData>>>>(1000,"testing_transformer",new Coordinates(0,0,0),&inputData,&outputData);

    rclcpp::Rate loop_rate(10);

    while(rclcpp::ok())
    {
        rclcpp::spin_some(sensor);
        rclcpp::spin_some(transformation);
        rclcpp::spin_some(actuator);
        loop_rate.sleep();
    }

    return 0;
}