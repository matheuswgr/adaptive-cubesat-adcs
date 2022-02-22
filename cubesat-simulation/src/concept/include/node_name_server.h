#ifndef __node_name_server_h
#define __node_name_server_h

#include "../../../include/units.h"
#include <string>

class NodeNameServer
{
    public:
        static std::string serve(unsigned long unit, int dev)
        {
            if(A == unit)
            {
                if(1 == dev)
                {
                    return "reaction_wheel_x_axis_shaft_current_smartdata";
                }   
                if(2 == dev)
                {
                    return "reaction_wheel_y_axis_shaft_current_smartdata";
                }
                if(3 == dev)
                {
                    return "reaction_wheel_z_axis_shaft_current_smartdata";
                }
            }
            else if(D == unit)
            {
                if(1 == dev)
                {
                    return "reaction_wheel_x_axis_shaft_duty_cycle_smartdata";
                }   
                if(2 == dev)
                {
                    return "reaction_wheel_y_axis_shaft_duty_cycle_smartdata";
                }
                if(3 == dev)
                {
                    return "reaction_wheel_z_axis_shaft_duty_cycle_smartdata";
                }
            }

            return "";
        }
};

#endif