#include "../hybrid_controller.h"
#include "../adaptive_controller_system.h"
#include "../nonadaptive_control_system.h"
#include "../sliding_controller_system.h"
#include <iostream>
#include <fstream>

int main()
{
    SignalMonitor<HybridController>* signalMonitor = new SignalMonitor<HybridController>;

    Eigen::Matrix<float, 90,90> predictorLearningLawWeights = 2.14e-19*Eigen::Matrix<float,90,90>::Identity();
    DisturbancePredictor predictor(predictorLearningLawWeights);
    
    Eigen::Matrix<float, 3,3> satelliteFrameInertiaTensor = 0.222*Eigen::Matrix<float,3,3>::Identity(); 
    Eigen::Matrix<float,3,3> reactionWheelsInertiaTensor = 5e-7*Eigen::Matrix<float,3,3>::Identity(); ;
    Eigen::Matrix<float, 6,6> learningLawWeights = 7.18e-19*Eigen::Matrix<float,6,6>::Identity();

    SatelliteModel satelliteModel(satelliteFrameInertiaTensor,reactionWheelsInertiaTensor,predictor,learningLawWeights);  
    
    Eigen::Matrix<float,3,3> controllerGains = 3.33e-2*Eigen::Matrix<float,3,3>::Identity();
    Eigen::Matrix<float,3,3> slidingVariableGains = 30*Eigen::Matrix<float,3,3>::Identity();
    Eigen::Matrix<float,3,3> slidingControllerGain  = 2.22e-4*Eigen::Matrix<float,3,3>::Identity();

    Eigen::Matrix<float,3,1> satelliteVelocity {7,7,7};
    Eigen::Matrix<float,3,1> reactionWheelVelocities {6,6,6};
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
    NonAdaptiveControlSystem ncontroller(predictor,satelliteModel, controller, kalmanFilter);

    SlidingModeController slidingController(254.13, 2.5e-4,slidingVariableGains,controllerGain ,controller);
    SlidingControllerSystem slidingControllerSystem(slidingController, satelliteModel, kalmanFilter);

    AdaptiveControllerControlSystem adaptiveController(predictor,satelliteModel,controller,true);

    HybridController hcontroller(signalMonitor,adaptiveController,ncontroller, slidingControllerSystem);

    Eigen::Matrix<float,3,1> noise = Eigen::Matrix<float,3,1>::Random();

    Eigen::Quaternion<float> referenceAttitude {0,0,1,0};

    hcontroller.SetReferenceAttitude(referenceAttitude);

    for(int i = 0; i < 20000; i++)
    {
        noise = Eigen::Matrix<float,3,1>::Random();

        for(int i = 0; i < 100; i++)
        {
            noise = noise + Eigen::Matrix<float,3,1>::Random();
        }
        noise = noise/101;

        Eigen::Matrix<float,3,1> satelliteVelocityN = satelliteVelocity + noise;

        noise = Eigen::Matrix<float,3,1>::Random();

        for(int i = 0; i < 100; i++)
        {
            noise = noise + Eigen::Matrix<float,3,1>::Random();
        }
        noise = noise/101;

        if (i==600)
        {
            reactionWheelVelocities = Eigen::Matrix<float,3,1>::Constant(1);
        }

        Eigen::Matrix<float,3,1> reactionWheelVelocitiesN =reactionWheelVelocities + noise;

        hcontroller.UpdateControlSignal(satelliteVelocityN(0), satelliteVelocityN(1), satelliteVelocityN(2), 
                                    reactionWheelVelocitiesN(0), reactionWheelVelocitiesN(1), reactionWheelVelocitiesN(2),attitude);
        //std::cout << hcontroller.controlSignal.transpose() << "\n\n";
    }
    
    
    return 0;
}