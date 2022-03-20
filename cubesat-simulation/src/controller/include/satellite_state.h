#ifndef __satellite_state_h
#define __satellite_state_h

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Geometry>

class SatelliteState
{
    public:
        Eigen::Matrix<float,3,1> satelliteVelocity;
        Eigen::Matrix<float,3,1> reactionWheelVelocities;
        Eigen::Quaternion<float> attitude;
};

#endif