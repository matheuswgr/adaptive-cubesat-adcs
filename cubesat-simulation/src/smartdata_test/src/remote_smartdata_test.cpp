#include "../../../include/responsiveSmartdata.h"
#include "../../../include/remoteSmartdata.h"
#include "../../../include/transducer.h"
#include "rclcpp/rclcpp.hpp"


template <unsigned long UNIT,typename Value>
class Dummy_Transducer: public Transducer<UNIT,Value>
{

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

int main(int argc, char * argv[])
{ 
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RemoteSmartData<Dummy_Transducer<1,float>,float>>(new Region(0.0,0.0,0.0,0.0,0,0),1,1000,"testing2",new Coordinates(0,0,0),"testing","sensor"));
    rclcpp::shutdown();
    return 0;
}