#ifndef __non_adaptive_control_system_h
#define __non_adaptive_control_system_h

#include "feedback_linearization_controller.h"
#include "digital_filter.h"
#include "disturbance_predictor.h"
#include "satellite_model.h"
#include "motor_controller.h"
#include "sliding_mode_controller.h"
#include <list>
#include <iostream>
#include "adaptive_square_root_cubature_kalman_filter.h"

class NonAdaptiveControlSystem 
{
    private:
        DisturbancePredictor disturbancePredictor;
        SatelliteModel satelliteModel;
        Eigen::Matrix<float,3,3> controllerGains;
        Eigen::Matrix<float,3,3> slidingVariableGains;
        Eigen::Quaternion<float> referenceAttitude;
        FeedbackLinearizationControler controller;
        AdaptiveSquareRootCubatureKalmanFilter filter;

    public:
        Eigen::Matrix<float,3,1> controlSignal;
        Eigen::Matrix<float,3,1> filteredSatelliteVelocity;
        Eigen::Matrix<float,3,1> filteredReactionWheelVelocities;
        Eigen::Quaternion<float> filteredAttitude;

    private:
        Eigen::Matrix<float,3,1> SlidingVariable(Eigen::Matrix<float,3,1> satelliteVelocity,Eigen::Quaternion<float> atitudeError)
        {
            Eigen::Matrix<float,3,1> s = satelliteVelocity;
            Eigen::Matrix<float,3,1> quaterionVector = Eigen::Matrix<float,3,1>(atitudeError.vec());

            if (atitudeError.w() < 0)
            {
                s = s - slidingVariableGains*quaterionVector;
            }
            else
            {
                s = s + slidingVariableGains*quaterionVector;
            }

            return s;
        }

    public:
        NonAdaptiveControlSystem(){}
        NonAdaptiveControlSystem(DisturbancePredictor disturbancePredictor,SatelliteModel satelliteModel, 
                                        FeedbackLinearizationControler controller, AdaptiveSquareRootCubatureKalmanFilter filter)
        {            
            this->disturbancePredictor = disturbancePredictor;
            this->satelliteModel = satelliteModel;
            this->filter = filter;

            this->controllerGains = 3.33e-2*Eigen::Matrix<float,3,3>::Identity();
            this->slidingVariableGains = 30*Eigen::Matrix<float,3,3>::Identity();

            this->controller = controller;
            this->controlSignal = Eigen::Matrix<float,3,1>::Zero();
        }
        void SetReferenceAttitude(Eigen::Quaternion<float> referenceAttitude)
        {
            this->referenceAttitude = referenceAttitude;
        }

        void UpdateControlSignal(Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude)
        {

            Eigen::Matrix<float,10,1> stateVector;

            stateVector(0) = satelliteVelocity(0);
            stateVector(1) = satelliteVelocity(1);
            stateVector(2) = satelliteVelocity(2);
            stateVector(3) = reactionWheelVelocities(0);
            stateVector(4) = reactionWheelVelocities(1);
            stateVector(5) = reactionWheelVelocities(2);
            stateVector(6) = attitude.w();
            stateVector(7) = attitude.x();
            stateVector(8) = attitude.y();
            stateVector(9) = attitude.z();

            Eigen::Matrix<float,10,1> filteredstate = filter.Filter(stateVector, controlSignal);
            
            for (int i = 0; i < 6; i++)
            {
                if(i < 3)
                {
                    filteredSatelliteVelocity(i) = filteredstate(i);
                }
                else 
                if (i > 2 && i < 6)
                {
                    filteredReactionWheelVelocities(i-3) = filteredstate(i); 
                }
            }

            filteredAttitude.w() =  filteredstate(6);
            filteredAttitude.x() =  filteredstate(7);
            filteredAttitude.y() =  filteredstate(8);
            filteredAttitude.z() =  filteredstate(9);

            Eigen::Quaternion<float> atitudeError = referenceAttitude.conjugate()*filteredAttitude;

            controlSignal = controller.UpdateControlSignal(SlidingVariable(filteredSatelliteVelocity,filteredAttitude), filteredSatelliteVelocity, filteredReactionWheelVelocities, filteredAttitude, atitudeError);
            for(int i =0; i < 3; i++)
            {
                 if(controlSignal(i) > 2.22e-4)
                {
                    controlSignal(i) = 2.22e-4;
                }
                else
                if(controlSignal(i) < -2.22e-4)
                {
                    controlSignal(i) = -2.22e-4;
                }
            }
        }   
};

#endif