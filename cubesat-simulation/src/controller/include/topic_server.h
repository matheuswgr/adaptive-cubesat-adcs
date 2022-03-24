#ifndef __topic_server_h
#define __topic_server_h

#include "units.h"
#include <string>

class TopicServer
{
    public:
        static std::string serve(unsigned long unit, int dev)
        {
            if(AMP == unit)
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
            else 
            if(DUTY == unit)
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
            else 
            if(RADPS == unit)
            {
                if(1 == dev)
                {
                    return "imu_velocity_x";
                }
                if(2 == dev)
                {
                    return "imu_velocity_y";
                }
                if(3 == dev)
                {
                    return "imu_velocity_z";
                }
                if(4 == dev)
                {
                    return "reaction_wheel_x_axis_shaft/velocity";
                }
                if(5 == dev)
                {
                    return "reaction_wheel_y_axis_shaft/velocity";
                }
                if(6 == dev)
                {
                    return "reaction_wheel_z_axis_shaft/velocity";
                }
            }
            else
            if(QUAT == unit)
            {
                return "imu_orientation";
            }

            return "";
        }
};

#endif