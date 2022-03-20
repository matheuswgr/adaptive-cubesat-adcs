#include "../sliding_controller_system.h"
#include "../satellite_model.h"
#include "../feedback_linearization_controller.h"
#include "../nonadaptive_control_system.h"
#include "../adaptive_square_root_cubature_kalman_filter.h"

#include <iostream>

int main()
{
    
    Eigen::Matrix<float, 90,90> predictorLearningLawWeights = 2.14e-19*Eigen::Matrix<float,90,90>::Identity();
    DisturbancePredictor predictor(predictorLearningLawWeights);
    
    Eigen::Matrix<float, 3,3> satelliteFrameInertiaTensor = 0.222*Eigen::Matrix<float,3,3>::Identity(); 
    Eigen::Matrix<float,3,3> reactionWheelsInertiaTensor = 5e-7*Eigen::Matrix<float,3,3>::Identity(); ;
    Eigen::Matrix<float, 6,6> learningLawWeights = 7.18e-19*Eigen::Matrix<float,6,6>::Identity();

    SatelliteModel satelliteModel(satelliteFrameInertiaTensor,reactionWheelsInertiaTensor,predictor,learningLawWeights);  
    
    Eigen::Matrix<float,3,3> controllerGains = 3.33e-2*Eigen::Matrix<float,3,3>::Identity();
    Eigen::Matrix<float,3,3> slidingVariableGains = 30*Eigen::Matrix<float,3,3>::Identity();
    Eigen::Matrix<float,3,3> slidingControllerGain  = 2.22e-4*Eigen::Matrix<float,3,3>::Identity();

    Eigen::Matrix<float,3,1> satelliteVelocity {0.1,0.1,0.1};
    Eigen::Matrix<float,3,1> reactionWheelVelocities {4188*0.3,4188*0.3,4188*0.3};
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
    Eigen::Matrix<float,3,3> controllerGain  = 2.22e-4*Eigen::Matrix<float,3,3>::Identity();

    AdaptiveSquareRootCubatureKalmanFilter kalmanFilter(satelliteModel,initialState);
    FeedbackLinearizationControler controller(controllerGains, satelliteModel,slidingVariableGains);
    NonAdaptiveControlSystem controlSystem(predictor,satelliteModel, controller, kalmanFilter);
    Eigen::Quaternion<float> referenceAttitude {0,0,1,0};


    controlSystem.SetReferenceAttitude(referenceAttitude);

    for(int i = 0; i<10; i++)
    {
       controlSystem.UpdateControlSignal(satelliteVelocity, reactionWheelVelocities, attitude);
        /*std::cout << satelliteVelocity.transpose() << "\n";
        std::cout << reactionWheelVelocities.transpose() << "\n";
        std::cout << attitude << "\n";

        std::cout << controlSystem.controlSignal << "\n";
        
        std::cout << controlSystem.filteredSatelliteVelocity.transpose() << "\n";
        std::cout << controlSystem.filteredReactionWheelVelocities.transpose() << "\n";
        std::cout << controlSystem.filteredAttitude << "\n\n";*/
    }


}