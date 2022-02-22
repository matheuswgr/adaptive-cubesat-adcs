#ifndef __transducer_h
#define __transducer_h

template<unsigned long UNIT, typename Value>
class Transducer
{
    public:
        static const unsigned long unit = UNIT;
        static const bool active = false;

    protected:
        Transducer(){}

    public:
        virtual ~ Transducer(){}
        virtual Value sense(){}
        virtual void actuate(const Value &value){}
};

#endif