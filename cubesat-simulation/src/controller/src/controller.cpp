#include "../include/local_smartdata.h"
#include "../include/transducer.h"
#include "../include/controller_smartdata.h"
#include "../include/plugin_orientation_sensor.h"
#include "../include/plugin_gyroscope.h"
#include "../include/plugin_motor_velocity_sensor.h"
#include "../include/plugin_motor_driver.h"
//#include "../include/hybrid_controller.h"

#include <list>
#include <memory>
#include <eigen3/Eigen/Geometry>

int main(int argc, char * argv[])
{ 
    rclcpp::init(argc, argv);

    //Create Sensors

    std::shared_ptr<LocalSmartData<Eigen::Quaternion<float>,PluginOrientationSensor,ControllerSmartData<HybridController,float>>> orientation = std::make_shared<LocalSmartData<Eigen::Quaternion<float>,PluginOrientationSensor,ControllerSmartData<HybridController,float>>>(1, 7, 7, 1);
    
    std::shared_ptr<LocalSmartData<float,PluginGyroscope,ControllerSmartData<HybridController,float>>> gyroX = std::make_shared<LocalSmartData<float,PluginGyroscope,ControllerSmartData<HybridController,float>>>(1, 7, 7, 1);
    std::shared_ptr<LocalSmartData<float,PluginGyroscope,ControllerSmartData<HybridController,float>>> gyroY = std::make_shared<LocalSmartData<float,PluginGyroscope,ControllerSmartData<HybridController,float>>>(2, 7, 7, 1);
    std::shared_ptr<LocalSmartData<float,PluginGyroscope,ControllerSmartData<HybridController,float>>> gyroZ = std::make_shared<LocalSmartData<float,PluginGyroscope,ControllerSmartData<HybridController,float>>>(3, 7, 7, 1);

    std::shared_ptr<LocalSmartData<float,PluginMotorVelocitySensor,ControllerSmartData<HybridController,float>>> velocimeterX = std::make_shared<LocalSmartData<float,PluginMotorVelocitySensor,ControllerSmartData<HybridController,float>>>(4, 7, 7, 1);
    std::shared_ptr<LocalSmartData<float,PluginMotorVelocitySensor,ControllerSmartData<HybridController,float>>> velocimeterY = std::make_shared<LocalSmartData<float,PluginMotorVelocitySensor,ControllerSmartData<HybridController,float>>>(5, 7, 7, 1);
    std::shared_ptr<LocalSmartData<float,PluginMotorVelocitySensor,ControllerSmartData<HybridController,float>>> velocimeterZ = std::make_shared<LocalSmartData<float,PluginMotorVelocitySensor,ControllerSmartData<HybridController,float>>>(6, 7, 7, 1);

    std::shared_ptr<LocalSmartData<float,PluginMotorDriver,ControllerSmartData<HybridController,float>>> driverX = std::make_shared<LocalSmartData<float,PluginMotorDriver,ControllerSmartData<HybridController,float>>>(1, 2, 2, 2);
    std::shared_ptr<LocalSmartData<float,PluginMotorDriver,ControllerSmartData<HybridController,float>>> driverY = std::make_shared<LocalSmartData<float,PluginMotorDriver,ControllerSmartData<HybridController,float>>>(2, 2, 2, 2);
    std::shared_ptr<LocalSmartData<float,PluginMotorDriver,ControllerSmartData<HybridController,float>>> driverZ = std::make_shared<LocalSmartData<float,PluginMotorDriver,ControllerSmartData<HybridController,float>>>(3, 2, 2, 2);

    ControllerSmartData<HybridController,float> controller(gyroX, gyroY, gyroZ, velocimeterX, velocimeterY, velocimeterZ, orientation, driverX, driverY, driverZ);

    rclcpp::Rate loop_rate(7000);

    while(rclcpp::ok())
    {
        gyroX->spinIt();
        gyroY->spinIt();
        gyroZ->spinIt();

        velocimeterX->spinIt();
        velocimeterY->spinIt();
        velocimeterZ->spinIt();
        
        orientation->spinIt();
        
        loop_rate.sleep();
    }

    return 0;
}