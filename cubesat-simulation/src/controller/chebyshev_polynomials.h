#ifndef __chebyshev_polynomials_h
#define __chebyshev_polynomials_h

#include <iostream>

class ChebyshevPolynomials
{
    public:
        static float T0(float x)
        {
            return 1.0;
        }
        
        static float T1(float x)
        {
            return x;
        }

        static float T2(float x)
        {
            return 2*x*T1(x) - T0(x);
        }
};
#endif