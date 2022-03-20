#ifndef __motor_controller_h
#define __motor_controller_h

#include "circular_buffer.h"
#include "iterator.h"

#include <iostream>

class MotorController
{
    private:
        float* inputGains;
        float* outputGains;
        float controlSignal;
        float referenceTorque;
        float rotorInertia;
        float referenceVelocity;
        float samplingPeriod;
        CircularBuffer<float>* inputBuffer;
        CircularBuffer<float>* outputBuffer;
        CircularBuffer<float>* referenceBuffer;
    
    public:
        MotorController(float* inputGains , float* outputGains, float rotorInertia, float samplingPeriod)
        {
            this->inputGains = inputGains;
            this->outputGains = outputGains;
            inputBuffer = new CircularBuffer<float>(4);
            outputBuffer = new CircularBuffer<float>(3);
            referenceBuffer = new CircularBuffer<float>(2);
            this->rotorInertia = rotorInertia;
            this->samplingPeriod = samplingPeriod;
            referenceTorque = 0;
            referenceVelocity = 0;

            for(int i = 0; i < 4; i++)
            {
                inputBuffer->push(0);
            }
            
            for(int i = 0; i < 3; i++)
            {
                outputBuffer->push(0);
            }
            
            for(int i = 0; i < 2; i++)
            {
                referenceBuffer->push(0);
            }

        }

        void SetReferenceTorque(float reference)
        {
            this->referenceTorque = reference;
        }

        float UpdateControlSignal(float measurement)
        {
            float referenceTorqueIntegral = 0;
            referenceBuffer->push(referenceTorque);
            Iterator<float>* iterator = referenceBuffer->getIterator();
            while(iterator->hasNext())
            {
                float* bufferContent = iterator->next();
                float acceleration = (*bufferContent)/rotorInertia;
                referenceTorqueIntegral = referenceTorqueIntegral + 0.5*samplingPeriod*acceleration;
            }

            referenceVelocity = referenceVelocity + referenceTorqueIntegral;

            inputBuffer->push(referenceVelocity-measurement);
            controlSignal = 0;
            
            iterator = inputBuffer->getIterator();
            int i = 0;
            while(iterator->hasNext())
            {
                float* bufferContent = iterator->next();
                controlSignal = controlSignal + inputGains[i]*(*bufferContent);
                i++;
            }


            iterator = outputBuffer->getIterator();
            i=0;
            while(iterator->hasNext())
            {
                float* bufferContent = iterator->next();
                controlSignal = controlSignal + outputGains[i]*(*bufferContent);
                i++;
            }
            if (controlSignal > 1)
            {
                controlSignal = 1;
            }
            else if (controlSignal < -1)
            {
                controlSignal = -1;
            }
            outputBuffer->push(controlSignal);
            return controlSignal;
        }
};

#endif