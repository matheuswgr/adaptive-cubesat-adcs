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

    auto currentSensorX= std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"current_sensor_x",new Coordinates(0,0,0),"reaction_wheel_x_axis_shaft/current","sensor");
    auto currentSensorY =std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"current_sensor_y",new Coordinates(0,0,0),"reaction_wheel_y_axis_shaft/current","sensor");
    auto currentSensorZ =std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"current_sensor_z",new Coordinates(0,0,0),"reaction_wheel_z_axis_shaft/current","sensor");
    
    auto wheelSpeedSensorX= std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"wheel_speed_sensor_x",new Coordinates(0,0,0),"reaction_wheel_x_axis_shaft/velocity","sensor");
    auto wheelSpeedSensorY= std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"wheel_speed_sensor_y",new Coordinates(0,0,0),"reaction_wheel_y_axis_shaft/velocity","sensor");
    auto wheelSpeedSensorZ= std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"wheel_speed_sensor_z",new Coordinates(0,0,0),"reaction_wheel_z_axis_shaft/velocity","sensor");

    auto frameSpeedSensorX= std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"frame_speed_sensor_x",new Coordinates(0,0,0),"imu_orientation_x","sensor");
    auto frameSpeedSensorY= std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"frame_speed_sensor_y",new Coordinates(0,0,0),"imu_orientation_y","sensor");
    auto frameSpeedSensorZ= std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"frame_speed_sensor_z",new Coordinates(0,0,0),"imu_orientation_z","sensor");
    
    auto frameOrientationO= std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"orientation_o",new Coordinates(0,0,0),"imu_orientation_o","sensor");
    auto frameOrientationX= std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"orientation_x",new Coordinates(0,0,0),"imu_orientation_x","sensor");
    auto frameOrientationY= std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"orientation_y",new Coordinates(0,0,0),"imu_orientation_y","sensor");
    auto frameOrientationZ= std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"orientation_z",new Coordinates(0,0,0),"imu_orientation_z","sensor");

    inputData.push_front(currentSensorX);
    inputData.push_front(currentSensorY);
    inputData.push_front(currentSensorZ);

    inputData.push_front(wheelSpeedSensorX);
    inputData.push_front(wheelSpeedSensorY);
    inputData.push_front(wheelSpeedSensorZ);

    inputData.push_front(frameSpeedSensorX);
    inputData.push_front(frameSpeedSensorY);
    inputData.push_front(frameSpeedSensorZ);

    inputData.push_front(frameOrientationO);
    inputData.push_front(frameOrientationX);
    inputData.push_front(frameOrientationY);
    inputData.push_front(frameOrientationZ);

    auto dutyCycleWheelX= std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"duty_cycle_x",new Coordinates(0,0,0),"reaction_wheel_x_axis_shaft/duty_cycle","actuator");
    auto dutyCycleWheelY= std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"duty_cycle_y",new Coordinates(0,0,0),"reaction_wheel_y_axis_shaft/duty_cycle","actuator");
    auto dutyCycleWheelZ= std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),2,2,"duty_cycle_z",new Coordinates(0,0,0),"reaction_wheel_z_axis_shaft/duty_cycle","actuator");

    outputData.push_front(dutyCycleWheelX);
    outputData.push_front(dutyCycleWheelY);
    outputData.push_front(dutyCycleWheelZ);

    auto transformation = std::make_shared<TransformerSmartData<Dummy_Transformation<1,float,std::shared_ptr<SmartData>,std::shared_ptr<SmartData>>,std::list<std::shared_ptr<SmartData>>>>(1000,"testing_transformer",new Coordinates(0,0,0),&inputData,&outputData);

    rclcpp::Rate loop_rate(1);

    while(rclcpp::ok())
    {

        rclcpp::spin_some(currentSensorX);
        rclcpp::spin_some(currentSensorY);
        rclcpp::spin_some(currentSensorZ);

        rclcpp::spin_some(wheelSpeedSensorX);
        rclcpp::spin_some(wheelSpeedSensorY);
        rclcpp::spin_some(wheelSpeedSensorZ);

        rclcpp::spin_some(frameSpeedSensorX);
        rclcpp::spin_some(frameSpeedSensorY);
        rclcpp::spin_some(frameSpeedSensorZ);

        rclcpp::spin_some(frameOrientationO);
        rclcpp::spin_some(frameOrientationX);
        rclcpp::spin_some(frameOrientationY);
        rclcpp::spin_some(frameOrientationZ);

        rclcpp::spin_some(transformation);

        rclcpp::spin_some(dutyCycleWheelX);
        rclcpp::spin_some(dutyCycleWheelY);
        rclcpp::spin_some(dutyCycleWheelZ);
         
        loop_rate.sleep();
    }

    return 0;

}
