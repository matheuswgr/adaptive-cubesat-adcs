#include "../include/local_smartdata.h"
#include "../include/transducer.h"
#include "../include/controller_smartdata.h"
#include "../include/plugin_amperimeter.h"
#include "../include/plugin_motor_driver.h"
#include "../include/proportional_controller.h"

/*
    Build the model with all sensors
    Implement the cubature kalman filter
    Implement the bessel digital filter
    Implement the NN adaptive controller
    Implement the motor controller
    Implement the sliding mode controller
    Implement the state machine
*/

#include <list>
#include <memory>

int main(int argc, char * argv[])
{ 
    rclcpp::init(argc, argv);

    std::shared_ptr<LocalSmartData<float,PluginAmperimeter>> amperimeterX = std::make_shared<LocalSmartData<float,PluginAmperimeter>>(1, 100, 10, 0);
    std::shared_ptr<LocalSmartData<float,PluginAmperimeter>> amperimeterY = std::make_shared<LocalSmartData<float,PluginAmperimeter>>(2, 100, 10, 0);

    std::list<std::shared_ptr<LocalSmartData<float,PluginAmperimeter>>> inputs;
    inputs.push_back(amperimeterX);
    inputs.push_back(amperimeterY);

    std::shared_ptr<LocalSmartData<float,PluginMotorDriver>> driverX = std::make_shared<LocalSmartData<float,PluginMotorDriver>>(1, 100, 10, 2);
    std::shared_ptr<LocalSmartData<float,PluginMotorDriver>> driverY = std::make_shared<LocalSmartData<float,PluginMotorDriver>>(2, 100, 10, 2);


    std::list<std::shared_ptr<LocalSmartData<float,PluginMotorDriver>>> outputs;

    outputs.push_back(driverX);
    outputs.push_back(driverY);

    ControllerSmartData<ProportionalController<LocalSmartData<float, PluginAmperimeter>, LocalSmartData<float,PluginMotorDriver>>
                        ,LocalSmartData<float, PluginAmperimeter>,LocalSmartData<float, PluginMotorDriver>> controller(inputs, outputs);


    rclcpp::Rate loop_rate(2000);

    while(rclcpp::ok())
    {
        amperimeterX->spinIt();
        amperimeterY->spinIt();
        driverX->spinIt();
        driverY->spinIt();        
        loop_rate.sleep();
    }

    return 0;
}