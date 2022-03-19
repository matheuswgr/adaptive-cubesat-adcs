#ifndef __hybrid_controller_h
#define __hybrid_controller_h

#include "observer.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class HybridController : public Observer
{
    private:
        ControlSystem controlSystem;
        ControlSystem[] availableControlSystems;
        int currentState;
        SignalMonitor signalMonitor;
        Eigen::Matrix<float,7,1> trackingError;
        Eigen::Matrix<float,3,1> referenceVelocity;
        Eigen::Quaternion referenceAttitude;

    public:
        Eigen::Matrix<float,3,1> controlSignal;
    
    public:
        HybridController(SignalMonitor signalMonitor, ControlSystem[] controlSystems)
        {
            signalMonitor.startObserving();
            this->signalMonitor = signalMonitor;
            currentState = 2;
            controlSystem = controlSystems[currentState];
            this->controlSystems = controlSystems;
        }

        void SetReferenceSignal(float satelliteVelocityX, float satelliteVelocityY, float satelliteVelocityZ, 
                                        Eigen::Quaternion attitude)
        {
            referenceVelocity(0) = satelliteVelocityX
            referenceVelocity(1) = satelliteVelocityY;
            referenceVelocity(2) = satelliteVelocityZ;
            referenceAttitude = attitude;
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

            Eigen::Quaternion attitudeError = referenceAttitude.conjugate()*attitude;

            trackingError(0) = referenceVelocty(0) - satelliteVelocityX
            trackingError(1) = referenceVelocty(1) - satelliteVelocityY;
            trackingError(2) = referenceVelocty(2) - satelliteVelocityZ;
            trackingError(3) = attitudeError.w;
            trackingError(4) = attitudeError.x;
            trackingError(5) = attitudeError.y;
            trackingError(6) = attitudeError.z;

            Eigen::Matrix<float,10,1> measurement;

            measurement(0) = satelliteVelocityX;
            measurement(1) = satelliteVelocityY;
            measurement(2) = satelliteVelocityZ;
            measurement(3) = reactionWheelVelocityX;
            measurement(4) = reactionWheelVelocityY;
            measurement(5) = reactionWheelVelocityZ;
            measurement(6) = attitude.w;
            measurement(7) = attitude.x;
            measurement(8) = attitude.y;
            measurement(9) = attitude.z;

            signalMonitor.UpdateTrackingErrorStatisics(trackingError);
            signalMonitor.UpdateSensorMeasurementsStatistics(measurement);


            controlSystem.UpdateControlSignal(satelliteVelocity, reactionWheelVelocities,attitude, attitudeError );
            
            controlSignal = controlSystem.controlSignal;
        }

        void notify()
        {
            int event = signalMonitor.lastEvent;
            
            if(currentState == 1)
            {
                if(event == 2)
                {
                    controlSystem = controlSystems[2];
                }
                if(event == 3)
                {
                    controlSystem = controlSystems[3];
                }
            }

            if(currentState == 2)
            {
                if(event == 1)
                {
                    controlSystem = controlSystems[1];
                }
                if(event == 3)
                {
                    controlSystem = controlSystems[3];
                }
            }

            if(currentState == 3)
            {
                if(event == 1)
                {
                    controlSystem = controlSystems[1];
                }
                if(event == 2)
                {
                    controlSystem = controlSystems[2];
                }
            }
        }
}

#endif