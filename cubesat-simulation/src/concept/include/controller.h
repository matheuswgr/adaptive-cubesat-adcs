#ifndef __controller_h
#define __controller_h

class Controller: public Observer
{
    public:
        virtual void control() = 0;
};

#endif