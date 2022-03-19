#ifndef __adaptive_controller_sytem_h
#define __adaptive_controller_sytem_h

#include "control_system.h"
#include "feedback_lineariazation_controller.h"
#include "digital_filter.h"
#include "disturbance_predictor.h"
#include "satellite_model.h"
#include "motor_controller.h"
#include <list>

class AdaptiveControllerControlSystem 
{
    private:
        Eigen::Matrix<float,3,1> controlSignal;
        DisturbancePredictor disturbancePredictor;
        SatelliteModel satelliteModel;
        Eigen::Matrix<float,3,3> controllerGains;
        Eigen::Matrix<float,3,3> slidingVariableGains;
        Eigen::Quaternion<float> referenceAttitude;
        Eigen::Matrix<float,3,1> filteredSatelliteVelocity;
        Eigen::Matrix<float,3,1> filteredReactionWheelVelocities;
        Eigen::Quaternion<float> filteredAttitude;
        FeedbackLinearizationControler controller;
        std::list<DigitalFilter*> filters;

    private:
        Eigen::Matrix<float,3,1> SlidingVariable(Eigen::Matrix<float,3,1> satelliteVelocity,Eigen::Quaternion<float> atitudeError)
        {
            Eigen::Matrix<float,3,1> s = satelliteVelocity;
            Eigen::Matrix<float,3,1> quaterionVector = Eigen::Matrix<float,3,1>(atitudeError.vec());

            if (atitudeError.w < 0)
            {
                s = s - slidingVariableGains*quaterionVector
            }
            else
            {
                s = s + slidingVariableGains*quaterionVector
            }

            return s;
        }

    public:
        AdaptiveControllerControlSystem(DisturbancePredictor disturbancePredictor,SatelliteModel satelliteModel, 
                                        FeedbackLinearizationControler controller, bool adaptive)
        {            
            this->disturbancePredictor = disturbancePredictor;
            this->satelliteModel = satelliteModel;

            this->controllerGains = 3.33e-2*Eigen::Matrix<float,3,3>::Identity();
            this->slidingVariableGains = 30*Eigen::Matrix<float,3,3>::Identity();
            
            float inputGainsMotor[] = {0.1416,0.1416};
            float outputGainsMotor[] = {0.7072};

            float inputGains[] = {0.0799,0.3196,0.4794,0.3196,0.0799};
            float ouputGains[] = {0.1968,-0.4984,0.041579,-0.01836};

            for(int i = 0; i < 10; i++)
            {
                if(i < 3 || i > 5)
                {
                    this->filters.push(new DigitalFilter(inputGains, outputGains,5,4));
                }
                else
                {
                    this->filters.push(new DigitalFilter(inputGainsMotor, outputGainsMotor,2,1));
                }
            }

            this->controller = controller;
        }
        void SetReferenceAttitude(Eigen::Quaternion<float> referenceAttitude)
        {
            this->referenceAttitude = referenceAttitude;
        }

        void UpdateControlSignal(Eigen::Matrix<float,3,1> satelliteVelocity, Eigen::Matrix<float,3,1> reactionWheelVelocities, Eigen::Quaternion<float> attitude)
        {
            int i = 0;
            for (std::list<DigitalFilter*>::iterator it = this->filters.begin(); it != this->filters.end(); ++it)
            {
                if(i < 3)
                {
                    filteredSatelliteVelocity(i) = (*it)->Filter(satelliteVelocity(i));
                }
                else 
                if (i >= 3 && i < 6)
                {
                    filteredReactionWheelVelocities(i-3) = (*it)->Filter(reactionWheelVelocities(i-3));                    
                }
                else
                {
                    filteredAttitude.coefs()(i-6) = (*it)->Filter(attitude.coefs()(i-6)(i-6))
                }
            } 

            filteredAttitude.normalize();

            Eigen::Quaternion<float> atitudeError = referenceAttitude.conjugate()*filteredAttitude;

            Eigen::Matrix<float,3,1> slidingVariable = satelliteVelocity + slidingVariableGains*sign;

            controlSignal = controller.UpdateControlSignal(SlidingVariable(filteredSatelliteVelocity,filteredAttitude), filteredSatelliteVelocity, filteredReactionWheelVelocities, filteredAttitude, atitudeError)
        }   
};

#endif