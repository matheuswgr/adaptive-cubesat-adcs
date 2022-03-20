#ifndef __topic_server_h
#define __topic_server_h

#include "units.h"
#include <string>

class TopicServer
{
    public:
        static std::string serve(unsigned long unit, int dev)
        {
            if(A == unit)
            {
                if(1 == dev)
                {
                    return "reaction_wheel_x_axis_shaft/current";
                }   
                if(2 == dev)
                {
                    return "reaction_wheel_y_axis_shaft/current";
                }
                if(3 == dev)
                {
                    return "reaction_wheel_z_axis_shaft/current";
                }
            }
            else if(D == unit)
            {
                if(1 == dev)
                {
                    return "reaction_wheel_x_axis_shaft/duty_cycle";
                }   
                if(2 == dev)
                {
                    return "reaction_wheel_y_axis_shaft/duty_cycle";
                }
                if(3 == dev)
                {
                    return "reaction_wheel_z_axis_shaft/duty_cycle";
                }
            }

            return "";
        }
};

#endif