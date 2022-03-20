#ifndef __signal_monitor_h
#define __signal_monitor_h

#include <eigen3/Eigen/Dense>
#include <math.h>
#include "observed.h"
#include <iostream>

template<typename T>
class SignalMonitor : public Observed<T>
{
    public:
        Eigen::Matrix<float,7,1> trackingErrorAverage;
        Eigen::Matrix<float,7,1> trackingErrorReferenceAverage;
        Eigen::Matrix<float,7,1> trackingErrorVariance;
        Eigen::Matrix<float,7,1> trackingErrorReferenceVariance;
        Eigen::Matrix<float,10,1> sensorMeasurementAverage;
        Eigen::Matrix<float,10,1> sensorMeasurementsVariance;
        Eigen::Matrix<float,10,1> sensorMeasurementsReferenceVariance;
        int learningCounter = 0;
        int errorLearningCounter = 0;
        bool learning = false;
        bool errorLearning = false;
        int counter=0;
        int negativeCounter=0;

    public:
        int lastEvent;

    public:
        SignalMonitor()
        {
            trackingErrorAverage = Eigen::Matrix<float,7,1>::Zero();
            trackingErrorVariance = Eigen::Matrix<float,7,1>::Zero();
            trackingErrorReferenceVariance = Eigen::Matrix<float,7,1>::Constant(0.001);
            trackingErrorReferenceAverage= Eigen::Matrix<float,7,1>::Zero();
            sensorMeasurementAverage = Eigen::Matrix<float,10,1>::Zero();
            sensorMeasurementsVariance = Eigen::Matrix<float,10,1>::Zero();
            sensorMeasurementsReferenceVariance = Eigen::Matrix<float,10,1>::Constant(0.001);
            lastEvent = 0;
        }

        void UpdateTrackingErrorStatisics(Eigen::Matrix<float,7,1> trackingError)
        {
            trackingErrorAverage = (1-0.5)*trackingErrorAverage + 0.5*(trackingError);
            trackingErrorVariance = (1-0.30)*(trackingErrorReferenceVariance) + 0.30*((trackingError-trackingErrorAverage).array() * (trackingError-trackingErrorAverage).array()).matrix();

            bool allSensorsOk = true;
            for(int i=0; i<7;i++)
            {
                if(trackingErrorAverage(i) > trackingErrorReferenceAverage(i) + 3*std::sqrt(trackingErrorReferenceVariance(i)) || trackingErrorAverage(i) < trackingErrorReferenceAverage(i) - 3*std::sqrt(trackingErrorReferenceVariance(i)))
                {
                    allSensorsOk = false;
                }
            }


            if (allSensorsOk)
            {
                counter = 0;
                if(lastEvent != 1 && !learning && !errorLearning)
                {
                    lastEvent = 1;
                    trackingErrorReferenceAverage = trackingErrorAverage;
                    trackingErrorReferenceVariance = trackingErrorVariance;   
                    sensorMeasurementsReferenceVariance = sensorMeasurementsVariance;                     
                    this->notifyObservers();
                }
            }

            if(errorLearningCounter > 50)
            {
                errorLearning = false;
                errorLearningCounter = 0;
                trackingErrorReferenceAverage = trackingErrorAverage;
                trackingErrorReferenceVariance = trackingErrorVariance;
            }

            if(errorLearning)
            {
                errorLearningCounter++;
            }

            bool anySensorNotOk = false;
            for(int i=0; i<7;i++)
            {
                if(trackingErrorAverage(i) > trackingErrorReferenceAverage(i) + std::sqrt(trackingErrorReferenceVariance(i)) || trackingErrorAverage(i) < trackingErrorReferenceAverage(i) - std::sqrt(trackingErrorReferenceVariance(i)))
                {
                    anySensorNotOk = true;
                    if (lastEvent != 2 && counter == 5 && !errorLearning)
                    {
                        lastEvent = 2;
                        sensorMeasurementsReferenceVariance = sensorMeasurementsVariance;
                        this->notifyObservers();
                        counter = 0;
                        errorLearning = true;
                        errorLearningCounter = 0;
                    }
                }
            }
            if(anySensorNotOk)
            {
                counter++;
            }           

        }

        void UpdateSensorMeasurementsStatistics(Eigen::Matrix<float,10,1> measurement)
        {
            float forgettingFactor = 0.115;
            float Fvalue = 2.659; // last 24 samples
            sensorMeasurementAverage = (1-0.85)*sensorMeasurementAverage + 0.85*(measurement);
            sensorMeasurementsVariance = (1-forgettingFactor)*(sensorMeasurementsVariance) + forgettingFactor*((measurement-sensorMeasurementAverage).array() * (measurement-sensorMeasurementAverage).array()).matrix();

            bool allSensorsOk = true;
            for(int i=0; i<10;i++)
            {
                if(sensorMeasurementsReferenceVariance(i) == 0)
                {
                    sensorMeasurementsReferenceVariance(i) = 1e-20;
                }
                if(sensorMeasurementsVariance(i)/sensorMeasurementsReferenceVariance(i)  <= 1.0/Fvalue || sensorMeasurementsVariance(i)/sensorMeasurementsReferenceVariance(i) >= Fvalue)
                {
                    allSensorsOk = false;
                }
            }

            if (allSensorsOk)
            {
                negativeCounter=0;
                if(lastEvent != 1  && !learning && !errorLearning)
                {
                    lastEvent = 1;
                    trackingErrorReferenceAverage = trackingErrorAverage;
                    trackingErrorReferenceVariance = trackingErrorVariance;   
                    sensorMeasurementsReferenceVariance = sensorMeasurementsVariance;                     
                    this->notifyObservers();

                    return;
                }
            }

            if(learningCounter > 150)
            {
                learning = false;
                learningCounter = 0;
                sensorMeasurementsReferenceVariance = sensorMeasurementsVariance; 
            }

            if(learning)
            {
                learningCounter++;
            }


            bool anySensorNotOk = false;
            for(int i=0; i<10;i++)
            {
                if(sensorMeasurementsReferenceVariance(i) == 0)
                {
                    sensorMeasurementsReferenceVariance(i) = 1e-20;
                }
                if(sensorMeasurementsVariance(i)/sensorMeasurementsReferenceVariance(i)  < 1.0/Fvalue || sensorMeasurementsVariance(i)/sensorMeasurementsReferenceVariance(i) > Fvalue)
                {
                    anySensorNotOk = true;
                    if (lastEvent != 3 && negativeCounter == 15 && !learning)
                    {
                        lastEvent = 3;
                        trackingErrorReferenceAverage = trackingErrorAverage;
                        trackingErrorReferenceVariance = trackingErrorVariance;   
                        this->notifyObservers();
                        negativeCounter = 0;
                        learning = true;
                        learningCounter = 0;
                        return;
                    }
                }
            }

            if(anySensorNotOk)
            {
                negativeCounter++;
            }            
        }

};

#endif