#include "../adaptive_controller_system.h"

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

    Eigen::Matrix<float,3,1> satelliteVelocity {0.1,0.1,0.1};
    Eigen::Matrix<float,3,1> reactionWheelVelocities {4188*0.3,4188*0.3,4188*0.3};
    Eigen::Quaternion<float> attitude {1,0,0,0};

    Eigen::Quaternion<float> referenceAttitude {0.5,0.5,0.5,0.5};

    FeedbackLinearizationControler controller(controllerGains, satelliteModel,slidingVariableGains);
    AdaptiveControllerControlSystem adaptive(predictor,satelliteModel,controller,true);

    adaptive.SetReferenceAttitude(referenceAttitude);

    for(int i = 0; i<10; i++)
    {
       adaptive.UpdateControlSignal(satelliteVelocity, reactionWheelVelocities, attitude);
    }

    std::cout << satelliteVelocity << "\n\n";
    std::cout << reactionWheelVelocities << "\n\n";
    std::cout << attitude << "\n\n";

    std::cout << adaptive.controlSignal << "\n\n";
    
    std::cout << adaptive.filteredSatelliteVelocity << "\n\n";
    std::cout << adaptive.filteredReactionWheelVelocities << "\n\n";
    std::cout << adaptive.filteredAttitude << "\n\n";

}