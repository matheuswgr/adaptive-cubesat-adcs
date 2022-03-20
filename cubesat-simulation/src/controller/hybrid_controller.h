#ifndef __hybrid_controller_h
#define __hybrid_controller_h

#include "observer.h"
#include "signal_monitor.h"
#include "adaptive_controller_system.h"
#include "nonadaptive_control_system.h"
#include "sliding_controller_system.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>

class HybridController : public Observer
{
    private:
        int currentState;
        SignalMonitor<HybridController>*  signalMonitor;
        Eigen::Matrix<float,7,1> trackingError;
        Eigen::Matrix<float,3,1> referenceVelocity;
        Eigen::Quaternion<float> referenceAttitude;
        AdaptiveControllerControlSystem adaptiveController; 
        NonAdaptiveControlSystem controller; 
        SlidingControllerSystem slidingController;


    public:
        Eigen::Matrix<float,3,1> controlSignal;
    
    public:
        HybridController(SignalMonitor<HybridController>* signalMonitor, AdaptiveControllerControlSystem adaptiveController, 
                            NonAdaptiveControlSystem controller, SlidingControllerSystem slidingController)
        {
            this->adaptiveController = adaptiveController;
            this->controller = controller;
            this->slidingController = slidingController;
            signalMonitor->startObserving(this);
            this->signalMonitor = signalMonitor;
            currentState = 2;
        }

        void SetReferenceAttitude(Eigen::Quaternion<float> attitude)
        {
            referenceAttitude = attitude;
            controller.SetReferenceAttitude(attitude);
            adaptiveController.SetReferenceAttitude(attitude);
            slidingController.SetReferenceAttitude(attitude);
        }

        void UpdateControlSignal(float satelliteVelocityX, float satelliteVelocityY, float satelliteVelocityZ, 
                                    float reactionWheelVelocityX, float reactionWheelVelocityY, float reactionWheelVelocityZ,
                                        Eigen::Quaternion<float> attitude)
        {
            Eigen::Matrix<float,3,1> satelliteVelocity;
            satelliteVelocity(0) = satelliteVelocityX;
            satelliteVelocity(1) = satelliteVelocityY;
            satelliteVelocity(2) = satelliteVelocityZ;
            
            Eigen::Matrix<float,3,1> reactionWheelVelocities;
            reactionWheelVelocities(0) = reactionWheelVelocityX;
            reactionWheelVelocities(1) = reactionWheelVelocityY;
            reactionWheelVelocities(2) = reactionWheelVelocityZ;

            Eigen::Quaternion<float> attitudeError = referenceAttitude.conjugate()*attitude;

            trackingError(0) = satelliteVelocityX;
            trackingError(1) = satelliteVelocityY;
            trackingError(2) = satelliteVelocityZ;
            trackingError(3) = attitudeError.w();
            trackingError(4) = attitudeError.x();
            trackingError(5) = attitudeError.y();
            trackingError(6) = attitudeError.z();

            Eigen::Matrix<float,10,1> measurement;

            measurement(0) = satelliteVelocityX;
            measurement(1) = satelliteVelocityY;
            measurement(2) = satelliteVelocityZ;
            measurement(3) = reactionWheelVelocityX;
            measurement(4) = reactionWheelVelocityY;
            measurement(5) = reactionWheelVelocityZ;
            measurement(6) = attitude.w();
            measurement(7) = attitude.x();
            measurement(8) = attitude.y();
            measurement(9) = attitude.z();
            
            signalMonitor->UpdateSensorMeasurementsStatistics(measurement);
            signalMonitor->UpdateTrackingErrorStatisics(trackingError);

            if(currentState == 1)
            {
                controller.UpdateControlSignal(satelliteVelocity, reactionWheelVelocities, attitude);
                controlSignal = controller.controlSignal;
            }
            else
            if(currentState == 2)
            {
                adaptiveController.UpdateControlSignal(satelliteVelocity, reactionWheelVelocities, attitude);
                controlSignal = adaptiveController.controlSignal;
            }
            else
            if(currentState == 3)
            {
                slidingController.UpdateControlSignal(satelliteVelocity, reactionWheelVelocities, attitude);
                controlSignal = adaptiveController.controlSignal;
            }   
        }

        void notify()
        {
            int event = signalMonitor->lastEvent;
            
            if(currentState == 1)
            {
                if(event == 2)
                {
                    std::cout << "event: " << event << " state " << currentState << " -> " << "state" << 2;
                    currentState = 2;
                    //controlSystem = controlSystems[2];
                }
                if(event == 3)
                {
                    std::cout << "event: " << event << " state " << currentState << " -> " << "state" << 3;
                    currentState = 3;
                    //controlSystem = controlSystems[3];
                }
            }

            if(currentState == 2)
            {
                if(event == 1)
                {
                    std::cout << "event: " << event << " state " << currentState << " -> " << "state" << 1;
                    currentState = 1;
                    //controlSystem = controlSystems[1];
                }
                if(event == 3)
                {
                    std::cout << "event: " << event << " state " << currentState << " -> " << "state" << 3;
                    currentState = 3;
                    //controlSystem = controlSystems[3];
                }
            }

            if(currentState == 3)
            {
                if(event == 1)
                {
                    std::cout << "event: " << event << " state " << currentState << " -> " << "state" << 1;
                    currentState = 1;
                    //controlSystem = controlSystems[1];
                }
                if(event == 2)
                {
                    std::cout << "event: " << event << " state " << currentState << " -> " << "state" << 2;
                    currentState = 2;
                    //controlSystem = controlSystems[2];
                }
            }
            std::cout << "\n";
        }
};

#endif