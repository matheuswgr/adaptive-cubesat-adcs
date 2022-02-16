#include "transducer.h"
#include "smartdata.h"
#include "std_msgs/msg/Float32MultiArray.hpp"

#include <iostream>

template <unsigned long UNIT,typename Value>
class Dummy_Transducer: public Transducer<UNIT,Value>
{
    friend ResponsiveSmartData<Dummy_Transducer,Value>;

public:
    static const bool active = false;

public:
    Dummy_Transducer(){}
    
    Value sense()
    {
        value++;
        return value;
    }

    void actuate(Value value)
    {
        this->value = value;
    }

private:
    Value value;
};


int main()
{

    Dummy_Transducer<1,float>* dummy = new Dummy_Transducer<1,float>;
    ResponsiveSmartData<Dummy_Transducer<1,float>,float>* sd = new ResponsiveSmartData<Dummy_Transducer<1,float>,float>(0,1,1,1);
    std::cout << sd->value() << "\n";
    sd->update();
    std::cout << sd->value() << "\n";
}