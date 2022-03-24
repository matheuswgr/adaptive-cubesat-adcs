#ifndef __motor_control_system_h
#define __motor_control_system_h

#include "digital_filter.h"
#include "motor_controller.h"
#include <list>
#include <iostream>

class MotorControlSystem 
{
    public:
        MotorController controller;
        DigitalFilter filter;

    public:
        float controlSignal;
        float filteredVelocity;

    public:
        MotorControlSystem()
        {            
            float inputGains[] = {0.0799,0.3196,0.4794,0.3196,0.0799};
            float ouputGains[] = {0.1968,-0.4984,0.041579,-0.01836};
            filter = DigitalFilter(inputGains, ouputGains,5,4);

            float cinputGains[] = {2.84,-7.16,5.8,-1.48};
            float couputGains[] = {1,1,-1};
            float sampling_period = 1.0/600.0;
            float rotorInertia = 5.67e-7;
            controller = MotorController(cinputGains, couputGains, rotorInertia, sampling_period);

        }
        void SetReferenceTorque(float torque)
        {
            this->controller.SetReferenceTorque(torque);
        }

        void UpdateControlSignal(float velocity)
        {

            filteredVelocity = filter.Filter(velocity);

            controlSignal = controller.UpdateControlSignal(filteredVelocity);
        }   
};

#endif