#include "motor_controller.h"
#include <iostream>
#include <fstream>

int main()
{
    float inputGains[] = {2.84,-7.16,5.8,-1.48};
    float ouputGains[] = {1,1,-1};
    float sampling_period = 1/600.0;
    float rotorInertia = 5.67e-7;
    MotorController motorController(inputGains, ouputGains, rotorInertia, sampling_period);
    motorController.SetReferenceTorque(2.2e-4);

    const int samples = 18000;
    float input = 0;
    
    float controlSignal[samples];

    std::ofstream resultsFile;
    resultsFile.open("motor_controller_test.csv");
    resultsFile << "control signal \n";
    for(int i = 0; i<samples; i++)
    {
        controlSignal[i] = motorController.UpdateControlSignal(input);
        resultsFile << controlSignal[i] << ",\n";
    }
    resultsFile.close();

    return 0;
}