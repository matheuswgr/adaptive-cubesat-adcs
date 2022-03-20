#ifndef __observer_h
#define __observer_h

class Observer
{
    public:
        virtual void notify(){};
        virtual void notify(int id){};
};

#endif