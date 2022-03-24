#ifndef __node_name_server_h
#define __node_name_server_h

#include "units.h"
#include <string>

class NodeNameServer
{
    public:
        static std::string serve(unsigned long unit, int dev)
        {
            if(AMP == unit)
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
            else 
            if(DUTY == unit)
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
            else 
            if(RADPS == unit)
            {
                if(1 == dev)
                {
                    return "imu_velocity_x_smartdata";
                }
                if(2 == dev)
                {
                    return "imu_velocity_y_smartdata";
                }
                if(3 == dev)
                {
                    return "imu_velocity_z_smartdata";
                }
                if(4 == dev)
                {
                    return "reaction_wheel_x_axis_shaft_velocity_smartdata";
                }
                if(5 == dev)
                {
                    return "reaction_wheel_y_axis_shaft_velocity_smartdata";
                }
                if(6 == dev)
                {
                    return "reaction_wheel_z_axis_shaft_velocity_smartdata";
                }
            }
            else
            if(QUAT == unit)
            {
                return "imu_orientation_smartdata";
            }

            return "";
        }
};

#endif