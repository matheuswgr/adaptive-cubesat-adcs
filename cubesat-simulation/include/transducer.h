#ifndef __transducer_h
#define __transducer_h


template<unsigned long _UNIT, typename Value>
class Transducer
{
    public:
        static const unsigned long UNIT = _UNIT;
        static const bool active = false;

    protected:
        Transducer(){}

    public:
        virtual ~ Transducer(){}
        virtual Value sense() = 0;
        virtual void actuate(const Value &value){}
};

#endif