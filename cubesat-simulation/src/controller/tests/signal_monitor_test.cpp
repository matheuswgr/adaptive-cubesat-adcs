#include "../hybrid_controller.h"
#include <iostream>
#include <fstream>

int main()
{
    SignalMonitor<HybridController>* signalMonitor = new SignalMonitor<HybridController>;
    HybridController controller(signalMonitor);
        
    Eigen::Matrix<float,3,1> satelliteVelocity {0.001,0.001,0.001};
    Eigen::Matrix<float,3,1> reactionWheelVelocities {0.3,0.3,0.3};
    Eigen::Quaternion<float> attitude {1,0,0,0};

    Eigen::Matrix<float,7,1> initialState = Eigen::Matrix<float,7,1>::Constant(1);

    Eigen::Matrix<float,7,1> noise = Eigen::Matrix<float,7,1>::Random();



    for(int i = 0; i<100;i++)
    {
        noise = Eigen::Matrix<float,7,1>::Random();

        for(int i = 0; i < 100; i++)
        {
            noise = noise + Eigen::Matrix<float,7,1>::Random();
        }
        noise = noise/101;

        if(i==50)
        {
            initialState = Eigen::Matrix<float,7,1>::Constant(0.75);
        }

        signalMonitor->UpdateTrackingErrorStatisics(initialState + noise);
    }
    
    return 0;
}