#ifndef __sliding_mode_controller_h
#define __sliding_mode_controller_h

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <math.h>


class SlidingModeController
{
    public:
        float boundaryLayerThickness;
        float errorLimit;
        Eigen::Matrix<float,3,3> controllerGain;
        Eigen::Matrix<float,3,3> slidingVariableGain;
        Eigen::Matrix<float,3,1> controlSignal;
        FeedbackLinearizationControler feedbackLinearizationController;

    Eigen::Matrix<float,3,1> tanh(Eigen::Matrix<float,3,1> argument)
    {
        Eigen::Matrix<float,3,1> result;
        for(int i = 0; i < 3; i++)
        {
            result(i) = std::tanh(boundaryLayerThickness*argument(i));
        }
        return result;
    }

    public: 
        SlidingModeController(){}
        SlidingModeController(float boundaryLayerThickness, float errorLimit,Eigen::Matrix<float,3,3> slidingVariableGain,  Eigen::Matrix<float,3,3> controllerGain, FeedbackLinearizationControler feedbackLinearizationController)
        {
            this->boundaryLayerThickness = boundaryLayerThickness;
            this->errorLimit = errorLimit;
            this->slidingVariableGain = slidingVariableGain;
            this->controllerGain = controllerGain;
            this->feedbackLinearizationController = feedbackLinearizationController;
        }

        Eigen::Matrix<float,3,1> UpdateControlSignal(Eigen::Matrix<float,3,1> slidingVariable, Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude, Eigen::Quaternion<float> atitudeError)
        {
            controlSignal = feedbackLinearizationController.UpdateControlSignal(slidingVariable,satelliteVelocity, reactionWheelVelocities, attitude, atitudeError) + controllerGain*tanh(slidingVariable);
            return controlSignal;
        }

};

#endif