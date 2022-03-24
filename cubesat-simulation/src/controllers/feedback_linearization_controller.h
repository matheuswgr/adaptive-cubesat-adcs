#ifndef __feedback_linearization_controller_h
#define __feedback_linearization_controller_h

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include "satellite_model.h"

#include <iostream>


class FeedbackLinearizationControler
{
    private: 
        Eigen::Matrix<float,3,3> controllerGain;
        Eigen::Matrix<float,3,3> slidingVariableGains;
        Eigen::Matrix<float,3,1> controlSignal;
        SatelliteModel satelliteModel;

    public:
        FeedbackLinearizationControler(){}
        FeedbackLinearizationControler(Eigen::Matrix<float,3,3> controllerGain, SatelliteModel satelliteModel, Eigen::Matrix<float,3,3> slidingVariableGains)
        {

            this->controllerGain = controllerGain;
            this->controlSignal = Eigen::Matrix<float,3,1>::Zero();
            this->satelliteModel = satelliteModel;
            this->slidingVariableGains = slidingVariableGains;
        }

        Eigen::Matrix<float,3,1> UpdateControlSignal(Eigen::Matrix<float,3,1> slidingVariable,Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude, Eigen::Quaternion<float> atitudeError)
        {
            Eigen::Quaternion<float> velocityQuaternion;
            velocityQuaternion.w() = 0;
            velocityQuaternion.x() = satelliteVelocity(0);
            velocityQuaternion.y() = satelliteVelocity(1);
            velocityQuaternion.z() = satelliteVelocity(2);

            controlSignal = controllerGain*slidingVariable + satelliteModel.InverseDynamicsSignal(slidingVariable, satelliteVelocity, reactionWheelVelocities, attitude);
            
            Eigen::Matrix<float,3,1> vectorPartQuaternionErrorDerivative;

            Eigen::Quaternion<float> quaternionErrorDerivative = atitudeError*velocityQuaternion;

            vectorPartQuaternionErrorDerivative(0) = quaternionErrorDerivative.x();
            vectorPartQuaternionErrorDerivative(1) = quaternionErrorDerivative.y();
            vectorPartQuaternionErrorDerivative(2) = quaternionErrorDerivative.z();

            std::cout << slidingVariable.transpose() << "\n";
            
            return controlSignal;
        }
};

#endif