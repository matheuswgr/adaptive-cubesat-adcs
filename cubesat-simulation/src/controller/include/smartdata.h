#ifndef __smartdata_h
#define __smartdata_h

#include "observed.h"
#include "observer.h"
#include "controller.h"
#include "transducer.h"
#include <eigen3/Eigen/Geometry>

template<typename Value>
class SmartData
{
    public:
    class Coordinates
    {
        public:
            float x;
            float y;
            float z;
        
        public:
            Coordinates(float x, float y, float z)
            {
                this->x = x;
                this->y = y;
                this->z = z;
            }
    };

    protected:
        Coordinates* locationCoordinates;
        unsigned int period;
        unsigned long currentTime;

    public:
        virtual Value value(){}
        
        virtual Value update(){}

        virtual void setValue(Value value){}

        virtual void wait(){};
        
        Coordinates location()
        {
            return *locationCoordinates;
        }

        unsigned long time()
        {
            return currentTime;
        }

        virtual void notify()
        {
            return;
        }

        void spinIt()
        {
            return;
        }

};

#endif