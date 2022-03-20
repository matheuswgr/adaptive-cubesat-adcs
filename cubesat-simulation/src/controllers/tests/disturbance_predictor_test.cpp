#include "../disturbance_predictor.h"
#include <iostream>
#include <fstream>

#include <eigen3/Eigen/Dense>

int main()
{
    Eigen::Matrix<float, 90,90> learningLawWeights = 2.14e-19*Eigen::Matrix<float,90,90>::Identity();
    DisturbancePredictor predictor(learningLawWeights);
    Eigen::Matrix<float,3,1> slidingVariable {0.3*31,0.3*31,0.3*31};
    Eigen::Matrix<float,3,1> satelliteVelocity {0.3,0.3,0.3};
    Eigen::Matrix<float,3,1> reactionWheelVelocities {4188*0.3,4188*0.3,4188*0.3};
    Eigen::Quaternion<float> attitude {1,0,0,0};

    for(int i = 0; i < 15; i++)
    {
        predictor.UpdateParameters(slidingVariable, satelliteVelocity,reactionWheelVelocities,attitude);
    }
    std::cout << predictor.Predict(slidingVariable, satelliteVelocity,reactionWheelVelocities,attitude) << "\n\n";
}