#ifndef __controller_smartdata_h
#define __controller_smartdata_h

#include "smartdata.h"
#include "controller.h"
#include "local_smartdata.h"
#include "rclcpp/rclcpp.hpp"
#include "observer.h"
#include "observed.h"

#include "../include/hybrid_controller.h"
#include "../include/adaptive_controller_system.h"
#include "../include/motor_control_system.h"
#include "../include/nonadaptive_control_system.h"
#include "../include/sliding_controller_system.h"

#include "../include/plugin_orientation_sensor.h"
#include "../include/plugin_gyroscope.h"
#include "../include/plugin_motor_velocity_sensor.h"
#include "../include/plugin_motor_driver.h"

#include <list>
#include <vector>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

template <typename Controller, typename Value>
class ControllerSmartData 
    : public SmartData<Value>, public Controller, public Observed<LocalSmartData<float,PluginMotorDriver,ControllerSmartData<HybridController,float>>> 
{
    private:
        std::shared_ptr<LocalSmartData<float,PluginGyroscope,ControllerSmartData<HybridController,float>>> gyroX;
        std::shared_ptr<LocalSmartData<float,PluginGyroscope,ControllerSmartData<HybridController,float>>> gyroY;
        std::shared_ptr<LocalSmartData<float,PluginGyroscope,ControllerSmartData<HybridController,float>>> gyroZ; 
        std::shared_ptr<LocalSmartData<float,PluginMotorVelocitySensor,ControllerSmartData<HybridController,float>>> velocimeterX;
        std::shared_ptr<LocalSmartData<float,PluginMotorVelocitySensor,ControllerSmartData<HybridController,float>>> velocimeterY; 
        std::shared_ptr<LocalSmartData<float,PluginMotorVelocitySensor,ControllerSmartData<HybridController,float>>> velocimeterZ;
        std::shared_ptr<LocalSmartData<Eigen::Quaternion<float>,PluginOrientationSensor,ControllerSmartData<HybridController,float>>> orientation;;
        std::shared_ptr<LocalSmartData<float,PluginMotorDriver,ControllerSmartData<HybridController,float>>> driverX;
        std::shared_ptr<LocalSmartData<float,PluginMotorDriver,ControllerSmartData<HybridController,float>>> driverY; 
        std::shared_ptr<LocalSmartData<float,PluginMotorDriver,ControllerSmartData<HybridController,float>>> driverZ;

        Eigen::Matrix<float,3,1> satelliteVelocity;
        Eigen::Matrix<float,3,1> reactionWheelVelocities;
        Eigen::Quaternion<float> attitude;

        std::vector<bool> freshnessList;
        
        std::vector<bool> driveFreshnessList;

        std::shared_ptr<Controller> controller;

        MotorControlSystem motorX;
        MotorControlSystem motorY;
        MotorControlSystem motorZ;

    public:
        ControllerSmartData(std::shared_ptr<LocalSmartData<float,PluginGyroscope,ControllerSmartData<HybridController,float>>> gyroX, std::shared_ptr<LocalSmartData<float,PluginGyroscope,ControllerSmartData<HybridController,float>>> gyroY,
                             std::shared_ptr<LocalSmartData<float,PluginGyroscope,ControllerSmartData<HybridController,float>>> gyroZ, std::shared_ptr<LocalSmartData<float,PluginMotorVelocitySensor,ControllerSmartData<HybridController,float>>> velocimeterX,
                              std::shared_ptr<LocalSmartData<float,PluginMotorVelocitySensor,ControllerSmartData<HybridController,float>>> velocimeterY, std::shared_ptr<LocalSmartData<float,PluginMotorVelocitySensor,ControllerSmartData<HybridController,float>>> velocimeterZ,
                              std::shared_ptr<LocalSmartData<Eigen::Quaternion<float>,PluginOrientationSensor,ControllerSmartData<HybridController,float>>> orientation, std::shared_ptr<LocalSmartData<float,PluginMotorDriver,ControllerSmartData<HybridController,float>>> driverX,
                              std::shared_ptr<LocalSmartData<float,PluginMotorDriver,ControllerSmartData<HybridController,float>>> driverY, std::shared_ptr<LocalSmartData<float,PluginMotorDriver,ControllerSmartData<HybridController,float>>> driverZ)
        {
            driveFreshnessList.push_back(false);
            driveFreshnessList.push_back(false);
            driveFreshnessList.push_back(false);

            this->gyroX = gyroX;
            this->gyroY = gyroY;
            this->gyroZ = gyroZ;

            this->gyroX->giveId(0);
            this->gyroY->giveId(1);
            this->gyroZ->giveId(2);

            freshnessList.push_back(false);
            freshnessList.push_back(false);
            freshnessList.push_back(false);

            this->velocimeterX = velocimeterX;
            this->velocimeterY = velocimeterY;
            this->velocimeterZ = velocimeterZ;

            this->velocimeterX->giveId(3);
            this->velocimeterY->giveId(4);
            this->velocimeterZ->giveId(5);

            freshnessList.push_back(false);
            freshnessList.push_back(false);
            freshnessList.push_back(false);

            this->orientation = orientation;
            
            this->orientation->giveId(6);

            freshnessList.push_back(false);

            this->driverX = driverX;
            this->driverY = driverY;
            this->driverZ = driverZ;

            this->driverX->setValue(0.0);
            this->driverY->setValue(0.0);
            this->driverZ->setValue(0.0);

            this->gyroX->startObserving(this);
            this->gyroY->startObserving(this);
            this->gyroZ->startObserving(this);

            this->velocimeterX->startObserving(this);
            this->velocimeterY->startObserving(this);
            this->velocimeterZ->startObserving(this);

            this->orientation->startObserving(this);

            this->startObserving(driverX.get());
            this->startObserving(driverY.get());
            this->startObserving(driverZ.get());

            SignalMonitor<HybridController>* signalMonitor = new SignalMonitor<HybridController>;

            Eigen::Matrix<float, 90,90> predictorLearningLawWeights = 1e-19*Eigen::Matrix<float,90,90>::Identity();
            DisturbancePredictor predictor(predictorLearningLawWeights);
            
            Eigen::Matrix<float, 3,3> satelliteFrameInertiaTensor = 2.22e-3*Eigen::Matrix<float,3,3>::Identity(); 
            Eigen::Matrix<float,3,3> reactionWheelsInertiaTensor = 5.87e-7*Eigen::Matrix<float,3,3>::Identity(); ;
            Eigen::Matrix<float, 6,6> learningLawWeights = 0*1e-10*Eigen::Matrix<float,6,6>::Identity();

            learningLawWeights(0,0) = 0*5e-8;
            learningLawWeights(1,1) = 0*5e-8;
            learningLawWeights(2,2) = 0*5e-8;

            SatelliteModel satelliteModel(satelliteFrameInertiaTensor,reactionWheelsInertiaTensor,predictor,learningLawWeights);  
            
            Eigen::Matrix<float,3,3> controllerGains = 1e-3*Eigen::Matrix<float,3,3>::Identity();
            Eigen::Matrix<float,3,3> slidingVariableGains = 30*Eigen::Matrix<float,3,3>::Identity();
            Eigen::Matrix<float,3,3> slidingControllerGain  = 1e-4*Eigen::Matrix<float,3,3>::Identity();

            Eigen::Matrix<float,3,1> satelliteVelocity {0,0,0};
            Eigen::Matrix<float,3,1> reactionWheelVelocities {0,0,0};
            Eigen::Quaternion<float> attitude {1,0,0,0};

            Eigen::Matrix<float,10,1> initialState;

            initialState(0) = satelliteVelocity(0);
            initialState(1) = satelliteVelocity(1);
            initialState(2) = satelliteVelocity(2);
            initialState(3) = reactionWheelVelocities(0);
            initialState(4) = reactionWheelVelocities(1);
            initialState(5) = reactionWheelVelocities(2);
            initialState(6) = attitude.w();
            initialState(7) = attitude.x();
            initialState(8) = attitude.y();
            initialState(9) = attitude.z();

            Eigen::Matrix<float,3,1> controlTorque {0.2e-4,0.2e-4,0.2e-3};
            Eigen::Matrix<float,3,3> controllerGain  = 1e-4*Eigen::Matrix<float,3,3>::Identity();

            AdaptiveSquareRootCubatureKalmanFilter kalmanFilter(satelliteModel,initialState);
            FeedbackLinearizationControler fcontroller(controllerGains, satelliteModel,slidingVariableGains);
            NonAdaptiveControlSystem ncontroller(predictor,satelliteModel, fcontroller, kalmanFilter);

            SlidingModeController slidingController(254.13, 2.5e-4,slidingVariableGains,controllerGain ,fcontroller);
            SlidingControllerSystem slidingControllerSystem(slidingController, satelliteModel, kalmanFilter);

            AdaptiveControllerControlSystem adaptiveController(predictor,satelliteModel,fcontroller,true);

            Eigen::Matrix<float,3,1> noise = Eigen::Matrix<float,3,1>::Random();
            Eigen::Quaternion<float> referenceAttitude {-0.6432894 , -0.1660942, 0.738829, 0.1127973};

            this->controller = std::make_shared<Controller>(signalMonitor,adaptiveController,ncontroller, slidingControllerSystem);

            this->controller->SetReferenceAttitude(referenceAttitude);
        }

        float value()
        {
            return this->driverX->value();
        }
        
        float update()
        {
            for(int i = 0; i<this->freshnessList.size(); i++)
            {
                this->freshnessList[i] = false;
            }

            satelliteVelocity(0) = gyroX->value();
            satelliteVelocity(1) = gyroY->value();
            satelliteVelocity(2) = gyroZ->value();

            reactionWheelVelocities(0) = velocimeterX->value();
            reactionWheelVelocities(1) = velocimeterY->value();
            reactionWheelVelocities(2) = velocimeterZ->value();

            attitude = orientation->value();

            this->controller->UpdateControlSignal(satelliteVelocity, reactionWheelVelocities, attitude);
            
            //RCLCPP_INFO(this->gyroX->transducer->get_logger(), "Control: %f",this->controller->controlSignal(0));

            float duty_cycle = ((16*this->controller->controlSignal(0)/(8.98e-4)) + 9.02e-4*reactionWheelVelocities(0))/4;
            if(duty_cycle > 1)
            {
                duty_cycle = 1;
            }
            else if (duty_cycle < -1)
            {
                duty_cycle = -1;
            }

            this->driverX->setValue(duty_cycle);

            duty_cycle = ((16*this->controller->controlSignal(1)/(8.98e-4)) + 9.02e-4*reactionWheelVelocities(1))/4;
            if(duty_cycle > 1)
            {
                duty_cycle = 1;
            }
            else if (duty_cycle < -1)
            {
                duty_cycle = -1;
            }
            this->driverY->setValue(duty_cycle);
            
            duty_cycle = ((16*this->controller->controlSignal(2)/(8.98e-4)) + 9.02e-4*reactionWheelVelocities(2))/4;
            if(duty_cycle > 1)
            {
                duty_cycle = 1;
            }
            else if (duty_cycle < -1)
            {
                duty_cycle = -1;
            }
            this->driverZ->setValue(duty_cycle);

            //this->motorX.SetReferenceTorque(this->controller->controlSignal(0));
            //this->motorY.SetReferenceTorque(this->controller->controlSignal(1));
            //this->motorZ.SetReferenceTorque(this->controller->controlSignal(2));

            return 0.1;
        }

        void updateDrive()
        {
            for(int i = 0; i<this->driveFreshnessList.size(); i++)
            {
                this->driveFreshnessList[i] = false;
            }
            //RCLCPP_INFO(this->gyroX->transducer->get_logger(), "I'm here");

            
            //this->motorX.UpdateControlSignal(this->velocimeterX->value());
            //this->motorY.UpdateControlSignal(this->velocimeterY->value());
            //this->motorZ.UpdateControlSignal(this->velocimeterZ->value());

            //RCLCPP_INFO(this->gyroX->transducer->get_logger(), "DutyCycle: %f",this->motorZ.controlSignal);
            //RCLCPP_INFO(this->gyroX->transducer->get_logger(), "Reference: %f",this->motorZ.controller.referenceVelocity);
            //RCLCPP_INFO(this->gyroX->transducer->get_logger(), "filteered: %f",this->motorZ.filteredVelocity);

            //this->driverX->setValue(this->motorX.controlSignal);
            //this->driverY->setValue(this->motorY.controlSignal);
            //this->driverZ->setValue(this->motorZ.controlSignal);

            this->notifyObservers();   
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

            if (id >= 3 && id <=5)
            {
                this->driveFreshnessList[id-3] = true;
            }

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

            timeToUpdate = true;
            for(int i = 0; i<this->driveFreshnessList.size(); i++)
            {
                if (!this->driveFreshnessList[i])
                {
                    timeToUpdate = false;
                }
            }

            if (timeToUpdate)
            {
                this->updateDrive();
            }
        }
};

#endif