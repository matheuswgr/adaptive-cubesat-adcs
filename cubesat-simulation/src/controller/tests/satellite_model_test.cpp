#include "../disturbance_predictor.h"
#include "../satellite_model.h"
#include <iostream>
#include <fstream>

#include <eigen3/Eigen/Dense>

int main()
{
    Eigen::Matrix<float, 90,90> predictorLearningLawWeights = 2.14e-19*Eigen::Matrix<float,90,90>::Identity();
    DisturbancePredictor predictor(predictorLearningLawWeights);
    
    Eigen::Matrix<float, 3,3> satelliteFrameInertiaTensor = 0.222*Eigen::Matrix<float,3,3>::Identity(); 
    Eigen::Matrix<float,3,3> reactionWheelsInertiaTensor = 5e-7*Eigen::Matrix<float,3,3>::Identity(); ;
    Eigen::Matrix<float, 6,6> learningLawWeights = 7.18e-19*Eigen::Matrix<float,6,6>::Identity();

    SatelliteModel satelliteModel(satelliteFrameInertiaTensor,reactionWheelsInertiaTensor,predictor,learningLawWeights);   
    
    Eigen::Matrix<float,3,1> slidingVariable {0.1*31,0.1*31,0.1*31};
    Eigen::Matrix<float,3,1> satelliteVelocity {0.3,0.3,0.3};
    Eigen::Matrix<float,3,1> reactionWheelVelocities {4188*0.1,4188*0.1,4188*0.1};
    Eigen::Quaternion<float> attitude {1,0,0,0};

    for(int i = 0; i < 50; i++)
    {
        satelliteModel.UpdateParameters(slidingVariable, satelliteVelocity,reactionWheelVelocities,attitude);
    }

    Eigen::Matrix<float,3,1> controlTorque {0.2e-2,0.2e-2,0.2e-2};

    SatelliteState state = satelliteModel.Predict(satelliteVelocity,reactionWheelVelocities,attitude, controlTorque, 1.0/150);

    std::cout << satelliteVelocity << "\n\n";
    std::cout << reactionWheelVelocities << "\n\n";
    std::cout << attitude << "\n\n";

    std::cout << state.satelliteVelocity<< "\n\n";
    std::cout << state.reactionWheelVelocities<< "\n\n";
    std::cout << state.attitude.coeffs() << "\n\n";

}