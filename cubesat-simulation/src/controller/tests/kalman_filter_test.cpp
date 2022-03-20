#include <eigen3/Eigen/Dense>
#include "../satellite_model.h"
#include "../feedback_linearization_controller.h"
#include "../sliding_mode_controller.h"
#include "../adaptive_square_root_cubature_kalman_filter.h"

#include <iostream>
#include <fstream>

int main()
{
    Eigen::Matrix<float, 90,90> predictorLearningLawWeights = 2.14e-19*Eigen::Matrix<float,90,90>::Identity();
    DisturbancePredictor predictor(predictorLearningLawWeights);
    
    Eigen::Matrix<float, 3,3> satelliteFrameInertiaTensor = 0.222*Eigen::Matrix<float,3,3>::Identity(); 
    Eigen::Matrix<float,3,3> reactionWheelsInertiaTensor = 5e-7*Eigen::Matrix<float,3,3>::Identity(); ;
    Eigen::Matrix<float, 6,6> learningLawWeights = 7.18e-19*Eigen::Matrix<float,6,6>::Identity();

    SatelliteModel satelliteModel(satelliteFrameInertiaTensor,reactionWheelsInertiaTensor,predictor,learningLawWeights);  
    
    Eigen::Matrix<float,3,1> satelliteVelocity {0.001,0.001,0.001};
    Eigen::Matrix<float,3,1> reactionWheelVelocities {4188*0.3,4188*0.3,4188*0.3};
    Eigen::Quaternion<float> attitude {1,0,0,0};

    Eigen::Matrix<float,10,1> initialState;

    Eigen::Matrix<float,10,1> disturbance {0.001,0.001,0.001,100,100,100,0,0,0,0};

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

    Eigen::Matrix<float,3,1> controlTorque {0,0,0};

    AdaptiveSquareRootCubatureKalmanFilter kalmanFilter(satelliteModel,initialState);
    std::cout << initialState << "\n\n";

    for(int i = 0; i< 10; i++)
    {
        std::cout << kalmanFilter.Filter(initialState + disturbance, controlTorque) << "\n\n";
    }

    kalmanFilter.UpdateCovarianceMatrices(initialState + disturbance);
}