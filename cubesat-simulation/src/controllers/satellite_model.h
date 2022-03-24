#ifndef __satellite_model_h
#define __satellite_model_h

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Geometry>
#include "satellite_state.h"
#include "disturbance_predictor.h"
#include <math.h>
#include <iostream>

class SatelliteModel
{
    Eigen::Matrix<float,6,1> inertiaParameters;
    Eigen::Matrix<float,3,6> dynamicsMatrix;
    Eigen::Matrix<float,6,6> adaptationLawWeights;
    Eigen::Matrix<float,3,1> slidingVariable;
    DisturbancePredictor disturbancePredictor;

    public:
        Eigen::Matrix<float, 3,3> satelliteFrameInertiaTensor;
        Eigen::Matrix<float,3,3> reactionWheelsInertiaTensor;

    private:
        void UpdateDynamicMatrix(Eigen::Matrix<float,3,1> slidingVariable, Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude)
        {
            dynamicsMatrix(0,0) = slidingVariable(0) - satelliteVelocity(0);
            dynamicsMatrix(0,1) = satelliteVelocity(1)*satelliteVelocity(2);
            dynamicsMatrix(0,2) = -satelliteVelocity(1)*satelliteVelocity(2);
            dynamicsMatrix(0,3) = 0;
            dynamicsMatrix(0,4) = reactionWheelVelocities(1)*reactionWheelVelocities(2);
            dynamicsMatrix(0,5) = -reactionWheelVelocities(1)*reactionWheelVelocities(2);

            dynamicsMatrix(1,0) = -satelliteVelocity(0)*satelliteVelocity(2);
            dynamicsMatrix(1,1) = slidingVariable(1) - satelliteVelocity(1);
            dynamicsMatrix(1,2) = satelliteVelocity(0)*satelliteVelocity(2);
            dynamicsMatrix(1,3) = -reactionWheelVelocities(0)*reactionWheelVelocities(2);
            dynamicsMatrix(1,4) = 0;
            dynamicsMatrix(1,5) = reactionWheelVelocities(1)*reactionWheelVelocities(2);

            dynamicsMatrix(2,0) = satelliteVelocity(0)*satelliteVelocity(1);
            dynamicsMatrix(2,1) = -satelliteVelocity(0)*satelliteVelocity(1);
            dynamicsMatrix(2,2) = slidingVariable(2) - satelliteVelocity(2);
            dynamicsMatrix(2,3) = reactionWheelVelocities(0)*reactionWheelVelocities(1);
            dynamicsMatrix(2,4) = -reactionWheelVelocities(0)*reactionWheelVelocities(1);
            dynamicsMatrix(2,5) = 0;
        }

        void UpdateInertiaMatrices()
        {
            for(int i = 0; i < 6; i++)
            {
                if (i < 3)
                {
                    satelliteFrameInertiaTensor(i,i) = inertiaParameters(i);
                }
                else
                {
                    reactionWheelsInertiaTensor(i%3, i%3) = inertiaParameters(i);
                }
            }
        }

        Eigen::Quaternion<float> FiniteRotationQuaterion(Eigen::Matrix<float,3,1> satelliteVelocity, float timeInterval)
        {
            Eigen::Quaternion<float> finiteRotation(std::cos(0.5*satelliteVelocity.norm()*timeInterval), (satelliteVelocity(0)/satelliteVelocity.norm())*std::sin(0.5*satelliteVelocity.norm()*timeInterval),(satelliteVelocity(1)/satelliteVelocity.norm())*std::sin(0.5*satelliteVelocity.norm()*timeInterval),(satelliteVelocity(2)/satelliteVelocity.norm())*std::sin(0.5*satelliteVelocity.norm()*timeInterval));
            return finiteRotation;
        }

    public:
        SatelliteModel(){}
        SatelliteModel(Eigen::Matrix<float, 3,3> satelliteFrameInertiaTensor, Eigen::Matrix<float,3,3> reactionWheelsInertiaTensor, DisturbancePredictor disturbancePredictor, Eigen::Matrix<float,6,6> weights)
        {
            this->satelliteFrameInertiaTensor = satelliteFrameInertiaTensor;
            this->reactionWheelsInertiaTensor = reactionWheelsInertiaTensor;
            this->disturbancePredictor = disturbancePredictor;
            this->adaptationLawWeights = weights;
            this->dynamicsMatrix = Eigen::Matrix<float,3,6>::Zero();

            for(int i = 0; i < 6; i++)
            {
                if (i < 3)
                {
                    inertiaParameters(i) = satelliteFrameInertiaTensor(i,i);
                }
                else
                {
                    inertiaParameters(i)  = reactionWheelsInertiaTensor(i%3, i%3);
                }
            }
        }

        void SetInitialDynamicMatrix(Eigen::Matrix<float,3,1> slidingVariable, Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude)
        {
            UpdateDynamicMatrix(slidingVariable, satelliteVelocity, reactionWheelVelocities, attitude);
            UpdateInertiaMatrices();
        }

        void UpdateParameters(Eigen::Matrix<float,3,1> slidingVariable, Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude)
        {
            Eigen::Matrix<float,1,6> partialParameterUpdate;
            partialParameterUpdate = 0.003333*(this->slidingVariable.transpose())*(dynamicsMatrix*adaptationLawWeights);
            this->slidingVariable = slidingVariable;
            UpdateDynamicMatrix(slidingVariable, satelliteVelocity, reactionWheelVelocities, attitude);
            partialParameterUpdate = partialParameterUpdate + 0.003333*(this->slidingVariable.transpose())*(dynamicsMatrix*adaptationLawWeights);
            inertiaParameters = inertiaParameters + partialParameterUpdate.transpose();
            UpdateInertiaMatrices();
            disturbancePredictor.UpdateParameters(slidingVariable, satelliteVelocity,reactionWheelVelocities,attitude);
        }

        SatelliteState Predict(Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude, Eigen::Matrix<float, 3,1> controlTorque, float timeInterval)
        {
            SatelliteState predictedState;

            predictedState.satelliteVelocity = satelliteVelocity + timeInterval*( satelliteFrameInertiaTensor.inverse()*( disturbancePredictor.Predict(satelliteVelocity, reactionWheelVelocities, attitude) - controlTorque - satelliteVelocity.cross(satelliteFrameInertiaTensor*satelliteVelocity + reactionWheelsInertiaTensor*reactionWheelVelocities) ) );
            predictedState.reactionWheelVelocities = reactionWheelVelocities + timeInterval*( reactionWheelsInertiaTensor.inverse()*controlTorque);
            predictedState.attitude = FiniteRotationQuaterion(satelliteVelocity, timeInterval)*attitude;
            return predictedState;
        }

        Eigen::Matrix<float,3,1> InverseDynamicsSignal(Eigen::Matrix<float,3,1> slidingVariable, Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude)
        {
            Eigen::Matrix<float,3,1> inverseDynamicsSignal = disturbancePredictor.Predict(satelliteVelocity, reactionWheelVelocities, attitude) - satelliteVelocity.cross(satelliteFrameInertiaTensor*satelliteVelocity + reactionWheelsInertiaTensor*reactionWheelVelocities);
            
            return inverseDynamicsSignal;
        }
};

#endif