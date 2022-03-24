#ifndef __digital_filter_h
#define __digital_filter_h

#include "circular_buffer.h"
#include "iterator.h"
#include <iostream>

class DigitalFilter
{
    public:
        float* inputGains;
        float* outputGains;
        float outputSignal;
        CircularBuffer<float>* inputBuffer;
        CircularBuffer<float>* outputBuffer;
    
    public:
        DigitalFilter(){}
        DigitalFilter(float* inputGains, float* outputGains, int inputOrder, int outputOrder)
        {
            this->inputGains = new float[inputOrder];
            this->outputGains = new float[outputOrder];

            inputBuffer = new CircularBuffer<float>(inputOrder);
            outputBuffer = new CircularBuffer<float>(outputOrder);

            for(int i = 0; i < inputOrder; i++)
            {
                this->inputGains[i] = inputGains[i];
                inputBuffer->push(0.0);
            }
            for(int i = 0; i < outputOrder; i++)
            {
                this->outputGains[i] = outputGains[i];
                outputBuffer->push(0.0);
            }
        }
        
        float Filter(float input)
        {
            inputBuffer->push(input);
            outputSignal = 0;
            
            Iterator<float>* iterator = inputBuffer->getIterator();
            int i = 0;
            while(iterator->hasNext())
            {
                float* bufferContent = iterator->next();
                outputSignal = outputSignal + this->inputGains[i]*(*bufferContent);
                i++;
            }


            iterator = outputBuffer->getIterator();
            i=0;
            while(iterator->hasNext())
            {
                float* bufferContent = iterator->next();
                outputSignal = outputSignal + outputGains[i]*(*bufferContent);
                i++;
            }
            outputBuffer->push(outputSignal);
            return outputSignal;
        }
};

#endif