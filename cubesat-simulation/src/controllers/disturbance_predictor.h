#ifndef __disturbance_predictor_h
#define __disturbance_predictor_h

#include "satellite_state.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Geometry>
#include "chebyshev_polynomials.h"

class DisturbancePredictor
{
    private:
        Eigen::Matrix<float,90,1> weights;
        Eigen::Matrix<float,3,1> slidingVariable;
        Eigen::Matrix<float,90,90> learningLawWeights;
        Eigen::Matrix<float,3,90> kernel;

    private:
        void UpdateKernel(Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude)
        {
            int stateSpaceDimension = 10;

            for(int i = 0; i < 3;i++)
            {
                kernel(0,0*3*stateSpaceDimension + 3*i+0) = ChebyshevPolynomials::T0(satelliteVelocity(i)); 
                kernel(0,0*3*stateSpaceDimension + 3*i+1) = ChebyshevPolynomials::T1(satelliteVelocity(i));
                kernel(0,0*3*stateSpaceDimension + 3*i+2) = ChebyshevPolynomials::T2(satelliteVelocity(i));     

                kernel(1,1*3*stateSpaceDimension + 3*i+0) = ChebyshevPolynomials::T0(satelliteVelocity(i)); 
                kernel(1,1*3*stateSpaceDimension + 3*i+1) = ChebyshevPolynomials::T1(satelliteVelocity(i)); 
                kernel(1,1*3*stateSpaceDimension + 3*i+2) = ChebyshevPolynomials::T2(satelliteVelocity(i)); 

                kernel(2,2*3*stateSpaceDimension + 3*i+0) = ChebyshevPolynomials::T0(satelliteVelocity(i)); 
                kernel(2,2*3*stateSpaceDimension + 3*i+1) = ChebyshevPolynomials::T1(satelliteVelocity(i)); 
                kernel(2,2*3*stateSpaceDimension + 3*i+2) = ChebyshevPolynomials::T2(satelliteVelocity(i)); 
            }

            for(int i = 0; i < 3;i++)
            {
                kernel(0,0*3*stateSpaceDimension + 3*i+0 + 9) = ChebyshevPolynomials::T0(reactionWheelVelocities(i)); 
                kernel(0,0*3*stateSpaceDimension + 3*i+1 + 9) = ChebyshevPolynomials::T1(reactionWheelVelocities(i)); 
                kernel(0,0*3*stateSpaceDimension + 3*i+2 + 9) = ChebyshevPolynomials::T2(reactionWheelVelocities(i)); 

                kernel(1,1*3*stateSpaceDimension + 3*i+0 + 9) = ChebyshevPolynomials::T0(reactionWheelVelocities(i)); 
                kernel(1,1*3*stateSpaceDimension + 3*i+1 + 9) = ChebyshevPolynomials::T1(reactionWheelVelocities(i)); 
                kernel(1,1*3*stateSpaceDimension + 3*i+2 + 9) = ChebyshevPolynomials::T2(reactionWheelVelocities(i)); 

                kernel(2,2*3*stateSpaceDimension + 3*i+0 + 9) = ChebyshevPolynomials::T0(reactionWheelVelocities(i)); 
                kernel(2,2*3*stateSpaceDimension + 3*i+1 + 9) = ChebyshevPolynomials::T1(reactionWheelVelocities(i)); 
                kernel(2,2*3*stateSpaceDimension + 3*i+2 + 9) = ChebyshevPolynomials::T2(reactionWheelVelocities(i)); 
            }

            for(int i = 0; i < 4;i++)
            {
                kernel(0,0*3*stateSpaceDimension + 3*i+0 + 18) = ChebyshevPolynomials::T0(attitude.coeffs()(i)); 
                kernel(0,0*3*stateSpaceDimension + 3*i+1 + 18) = ChebyshevPolynomials::T1(attitude.coeffs()(i)); 
                kernel(0,0*3*stateSpaceDimension + 3*i+2 + 18) = ChebyshevPolynomials::T2(attitude.coeffs()(i)); 

                kernel(1,1*3*stateSpaceDimension + 3*i+0 + 18) = ChebyshevPolynomials::T0(attitude.coeffs()(i)); 
                kernel(1,1*3*stateSpaceDimension + 3*i+1 + 18) = ChebyshevPolynomials::T1(attitude.coeffs()(i)); 
                kernel(1,1*3*stateSpaceDimension + 3*i+2 + 18) = ChebyshevPolynomials::T2(attitude.coeffs()(i)); 

                kernel(2,2*3*stateSpaceDimension + 3*i+0 + 18) = ChebyshevPolynomials::T0(attitude.coeffs()(i)); 
                kernel(2,2*3*stateSpaceDimension + 3*i+1 + 18) = ChebyshevPolynomials::T1(attitude.coeffs()(i)); 
                kernel(2,2*3*stateSpaceDimension + 3*i+2 + 18) = ChebyshevPolynomials::T2(attitude.coeffs()(i)); 
            }
        }

    public:
        DisturbancePredictor(){}
        DisturbancePredictor(Eigen::Matrix<float, 90,90> learningLawWeights)
        {
            this->weights = Eigen::Matrix<float, 90,1>::Zero();
            this->learningLawWeights = learningLawWeights;
            this->kernel = Eigen::Matrix<float,3,90>::Zero();
        }  

        void UpdateParameters(Eigen::Matrix<float,3,1> slidingVariable, Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude)
        {   
            Eigen::Matrix<float,1,90> partialParameterUpdate;
            partialParameterUpdate = 0.003333*(this->slidingVariable.transpose())*(kernel*learningLawWeights);
            this->slidingVariable = slidingVariable;
            UpdateKernel(satelliteVelocity, reactionWheelVelocities, attitude);
            partialParameterUpdate = partialParameterUpdate + 0.003333*(this->slidingVariable.transpose())*(kernel*learningLawWeights);
            weights = weights + partialParameterUpdate.transpose();
        }      

        Eigen::Matrix<float,3,1> Predict(Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude)
        {
            Eigen::Matrix<float,3,90> lastKernel = kernel;
            UpdateKernel(satelliteVelocity,reactionWheelVelocities, attitude);
            Eigen::Matrix<float,3,1> prediction = kernel*weights;
            kernel = lastKernel;
            return prediction;
        }

};

#endif