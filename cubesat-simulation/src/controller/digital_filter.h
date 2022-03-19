#ifndef __digital_filter_h
#define __digital_filter_h

#include "circular_buffer.h"
#include "iterator.h"

class DigitalFilter
{
    private:
        float* inputGains;
        float* outputGains;
        float outputSignal;
        CircularBuffer<float>* inputBuffer;
        CircularBuffer<float>* outputBuffer;
    
    public:
        DigitalFilter(float* inputGains, float* outputGains, int inputOrder, int outputOrder)
        {
            this->inputGains = inputGains;
            this->outputGains = outputGains;
            inputBuffer = new CircularBuffer<float>(inputOrder);
            outputBuffer = new CircularBuffer<float>(outputOrder);
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
                outputSignal = outputSignal + inputGains[i]*(*bufferContent);
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