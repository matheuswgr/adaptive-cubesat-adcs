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
    public:
        Eigen::Matrix<float,6,1> inertiaParameters;
        Eigen::Matrix<float,3,6> dynamicsMatrix;
        Eigen::Matrix<float,6,6> adaptationLawWeights;
        Eigen::Matrix<float,3,1> slidingVariable;
        DisturbancePredictor disturbancePredictor;

    public:
        Eigen::Matrix<float, 3,3> satelliteFrameInertiaTensor;
        Eigen::Matrix<float,3,3> reactionWheelsInertiaTensor;

    private:
        void UpdateDynamicMatrix(Eigen::Matrix<float,3,1> slidingVariable, Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude,Eigen::Quaternion<float> attitudeErrorDerivative)
        {
            Eigen::Matrix<float,3,3> slidingVariableGains = 30*Eigen::Matrix<float,3,3>::Identity();

            Eigen::Matrix<float,3,1> quaternionErrorDerivativeVector;

            if(attitude.w() >= 0)
            {
                quaternionErrorDerivativeVector(0) = slidingVariableGains(0,0)*attitudeErrorDerivative.x();
                quaternionErrorDerivativeVector(1) = slidingVariableGains(0,0)*attitudeErrorDerivative.y();
                quaternionErrorDerivativeVector(2) = slidingVariableGains(0,0)*attitudeErrorDerivative.z();
            }
            else
            {
                quaternionErrorDerivativeVector(0) = -slidingVariableGains(0,0)*attitudeErrorDerivative.x();
                quaternionErrorDerivativeVector(1) = -slidingVariableGains(0,0)*attitudeErrorDerivative.y();
                quaternionErrorDerivativeVector(2) = -slidingVariableGains(0,0)*attitudeErrorDerivative.z();   
            }

            dynamicsMatrix(0,0) = quaternionErrorDerivativeVector(0);
            dynamicsMatrix(0,1) = satelliteVelocity(1)*satelliteVelocity(2);
            dynamicsMatrix(0,2) = -satelliteVelocity(1)*satelliteVelocity(2);
            dynamicsMatrix(0,3) = 0;
            dynamicsMatrix(0,4) = reactionWheelVelocities(1)*reactionWheelVelocities(2);
            dynamicsMatrix(0,5) = -reactionWheelVelocities(1)*reactionWheelVelocities(2);

            dynamicsMatrix(1,0) = -satelliteVelocity(0)*satelliteVelocity(2);
            dynamicsMatrix(1,1) = quaternionErrorDerivativeVector(1);
            dynamicsMatrix(1,2) = satelliteVelocity(0)*satelliteVelocity(2);
            dynamicsMatrix(1,3) = -reactionWheelVelocities(0)*reactionWheelVelocities(2);
            dynamicsMatrix(1,4) = 0;
            dynamicsMatrix(1,5) = reactionWheelVelocities(0)*reactionWheelVelocities(2);

            dynamicsMatrix(2,0) = satelliteVelocity(0)*satelliteVelocity(1);
            dynamicsMatrix(2,1) = -satelliteVelocity(0)*satelliteVelocity(1);
            dynamicsMatrix(2,2) = quaternionErrorDerivativeVector(2);
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
            if(satelliteVelocity.norm() != 0)
            {
                Eigen::Quaternion<float> finiteRotation(std::cos(0.5*satelliteVelocity.norm()*timeInterval), (satelliteVelocity(0)/satelliteVelocity.norm())*std::sin(0.5*satelliteVelocity.norm()*timeInterval),(satelliteVelocity(1)/satelliteVelocity.norm())*std::sin(0.5*satelliteVelocity.norm()*timeInterval),(satelliteVelocity(2)/satelliteVelocity.norm())*std::sin(0.5*satelliteVelocity.norm()*timeInterval));
            return finiteRotation;

            }
            else
            {
                Eigen::Quaternion<float> finiteRotation(1,0,0,0);
                        return finiteRotation;

            }
                        
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

        void SetInitialDynamicMatrix(Eigen::Matrix<float,3,1> slidingVariable, Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude,Eigen::Quaternion<float> attitudeErrorDerivative)
        {
            UpdateDynamicMatrix(slidingVariable, satelliteVelocity, reactionWheelVelocities, attitude,attitudeErrorDerivative);
        }

        void UpdateParameters(Eigen::Matrix<float,3,1> slidingVariable, Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude, Eigen::Quaternion<float> attitudeErrorDerivative)
        {
            Eigen::Matrix<float,1,6> partialParameterUpdate;
            partialParameterUpdate = 0.003333*(this->slidingVariable.transpose())*(dynamicsMatrix*adaptationLawWeights);
            this->slidingVariable = slidingVariable;
            UpdateDynamicMatrix(slidingVariable, satelliteVelocity, reactionWheelVelocities, attitude,attitudeErrorDerivative);
            partialParameterUpdate = partialParameterUpdate + 0.003333*(this->slidingVariable.transpose())*(dynamicsMatrix*adaptationLawWeights);
            
            Eigen::Matrix<float,1,6> updatedInertiaParameters = inertiaParameters + partialParameterUpdate.transpose();

            if (updatedInertiaParameters(0) > 0)
            {
                inertiaParameters(0) = updatedInertiaParameters(0);
            }

            if (updatedInertiaParameters(1) > 0)
            {
                inertiaParameters(1) = updatedInertiaParameters(1);
            }

            if (updatedInertiaParameters(2) > 0)
            {
                inertiaParameters(2) = updatedInertiaParameters(2);
            }
            UpdateInertiaMatrices();

            //std::cout << "\n" << partialParameterUpdate << "\n";
            //std::cout << "\n" << inertiaParameters.transpose() << "\n";
            //std::cout << "\n" << dynamicsMatrix << "\n";
            //std::cout << satelliteFrameInertiaTensor << "\n";

            disturbancePredictor.UpdateParameters(slidingVariable, satelliteVelocity,reactionWheelVelocities,attitude);
        }

        SatelliteState Predict(Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude, Eigen::Matrix<float, 3,1> controlTorque, float timeInterval)
        {
            SatelliteState predictedState;

            predictedState.satelliteVelocity = satelliteVelocity + timeInterval*( satelliteFrameInertiaTensor.inverse()*( disturbancePredictor.Predict(satelliteVelocity, reactionWheelVelocities, attitude) - controlTorque - satelliteVelocity.cross(satelliteFrameInertiaTensor*satelliteVelocity + reactionWheelsInertiaTensor*reactionWheelVelocities) ) );
            predictedState.reactionWheelVelocities = reactionWheelVelocities + timeInterval*( reactionWheelsInertiaTensor.inverse()*controlTorque);
            predictedState.attitude = FiniteRotationQuaterion(satelliteVelocity, timeInterval)*attitude;
            std::cout << "attitude: " << attitude << "\n";
            std::cout << "finite rotation: " << FiniteRotationQuaterion(satelliteVelocity, timeInterval) << "\n";
            std::cout << "final attitude: " << predictedState.attitude << "\n";

            return predictedState;
        }

        Eigen::Matrix<float,3,1> InverseDynamicsSignal(Eigen::Matrix<float,3,1> slidingVariable, Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude)
        {
            Eigen::Matrix<float,3,1> disturbancePrediction = disturbancePredictor.Predict(satelliteVelocity, reactionWheelVelocities, attitude);
            Eigen::Matrix<float,3,1> inverseDynamics = dynamicsMatrix*this->inertiaParameters;
            Eigen::Matrix<float,3,1> inverseDynamicsSignal = inverseDynamics + disturbancePrediction;
            
            //std::cout << "\n disturbance: " << disturbancePrediction.transpose() << "\n";
            //std::cout << "inverse dynamics:" << inverseDynamics.transpose() << "\n";
            //std::cout << "dynamic matrix: \n" << dynamicsMatrix << "\n";
            //std::cout << "inertia parameters:" << this->inertiaParameters.transpose() << "\n\n";

            return inverseDynamicsSignal;
        }
};

#endif