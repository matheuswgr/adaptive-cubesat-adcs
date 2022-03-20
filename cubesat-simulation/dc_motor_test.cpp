#include "dc_motor_model.cc"

#include <iostream>

int main()
{
    float time_step = 1e-6;
    DcMotor dc_motor(16, 26e-6, 8.98e-4,9.02e-4,4);
    float velocity = 40.0;

    for(int i = 0; i < 100; i++)
    {
        dc_motor.duty_cycle = 1.0;
        dc_motor.propagateState(time_step,velocity);

        std::cout << "velocity: " << dc_motor.velocity << "\n";
        std::cout << "current: " << dc_motor.current << "\n";
        std::cout << "torque: " << dc_motor.torque << "\n";
        std::cout << "duty_cyle: " << dc_motor.duty_cycle << "\n\n";
    }
}