#ifndef __signal_monitor_h
#define __signal_monitor_h

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <observed.h>
#include <hybrid_controller.h>

class SignalMonitor : public Observed<HybridController>
{
    private:
        Eigen::Matrix<7,1> trackingErrorAverage;
        Eigen::Matrix<7,1> trackingErrorVariance;
        Eigen::Matrix<7,1> trackingErrorReferenceVariance;
        Eigen::Matrix<10,1> sensorMeasurementsVariance;
        Eigen::Matrix<10,1> sensorMeasurementsReferenceVariance;

    public:
        int lastEvent;

    public:
        SignalMonitor()
        {
            trackingErrorAverage = Eigen::Matrix<7,1>::Zero();
            trackingErrorVariance = Eigen::Matrix<7,1>::Zero();
            trackingErrorReferenceVariance = Eigen::Matrix<7,1>::Zero();
            trackingErrorReferenceAverage= Eigen::Matrix<7,1>::Zero();
            sensorMeasurementsVariance = Eigen::Matrix<10,1>::Zero();
            sensorMeasurementsReferenceVariance = Eigen::Matrix<10,1>::Zero();
            lastEvent = 0;
        }

        void UpdateTrackingErrorStatisics(Eigen::Matrix<7,1> trackingError)
        {
            trackingErrorAverage = 0.998*trackingErrorAverage + (1-0.998)*(trackingError);
            trackingErrorVariance = 0.998*(trackingErrorReferenceVariance) + (1-0.998)*((trackingError-trackingErrorAverage).array() * (trackingError-trackingErrorAverage).array()).matrix()

            for(i=0; i<7;i++)
            {
                if(trackingErrorAverage(i) > trackingErrorReferenceAverage(i) + std::math.sqrt(trackingErrorReferenceVariance(i)))
                {
                    if (lastEvent != 2)
                    {
                        lastEvent = 2;
                        sensorMeasurementsReferenceVariance = sensorMeasurementsVariance;
                        notifyObservers();
                    }
                }
            }

            if (lastEvent != 1)
            {
                lastEvent = 1;
                sensorMeasurementsReferenceVariance = sensorMeasurementsVariance;
                trackingErrorReferenceAverage = trackingErrorAverage;
                trackingErrorReferenceVariance = trackingErrorVariance;
                notifyObservers();
            }

        }

        void UpdateSensorMeasurementsStatistics(Eigen::Matrix<10,1> measurement)
        {
            sensorMeasurementsVariance = 0.998*(sensorMeasurementsVariance) + (1-0.998)*((measurement-sensorMeasurementsVariance).array() * (measurement-sensorMeasurementsVariance).array()).matrix()
            
            for(i=0; i<10;i++)
            {
                if(sensorMeasurementsVariance(i) > 0.95*sensorMeasurementsReferenceVariance(i) || sensorMeasurementsVariance(i) > 1.05*sensorMeasurementsReferenceVariance(i))
                {
                    if (lastEvent != 3)
                    {
                        lastEvent = 3;
                        trackingErrorReferenceAverage = trackingErrorAverage;
                        trackingErrorReferenceVariance = trackingErrorVariance;                        
                        notifyObservers();
                    }
                }
            }
            
            if (lastEvent != 1)
            {
                lastEvent = 1;
                sensorMeasurementsReferenceVariance = sensorMeasurementsVariance;
                trackingErrorReferenceAverage = trackingErrorAverage;
                trackingErrorReferenceVariance = trackingErrorVariance;
                notifyObservers();
            }
        }

};

#endif