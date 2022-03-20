#include "digital_filter.h"
#include <iostream>
#include <fstream>

int main()
{
    float inputGains[] = {0.0799,0.3196,0.4794,0.3196,0.0799};
    float ouputGains[] = {0.1968,-0.4984,0.041579,-0.01836};
    DigitalFilter digital_filter(inputGains, ouputGains,5,4);

    const int samples = 30;
    float input = 0.1;
    
    float outputSignal[samples];

    std::ofstream resultsFile;
    resultsFile.open("digital_filter.csv");
    resultsFile << "output signal \n";
    for(int i = 0; i<samples; i++)
    {
        outputSignal[i] = digital_filter.Filter(input);
        resultsFile << outputSignal[i] << ",\n";
    }
    resultsFile.close();

    return 0;
}