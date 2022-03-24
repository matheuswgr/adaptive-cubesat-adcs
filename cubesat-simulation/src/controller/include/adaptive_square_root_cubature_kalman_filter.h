#ifndef __adaptive_square_root_cubature_kalman_filter_h
#define __adaptive_square_root_cubature_kalman_filter_h

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "satellite_model.h"
#include "satellite_state.h"
#include <math.h>
#include <iostream>

class AdaptiveSquareRootCubatureKalmanFilter
{
    private:
        Eigen::Matrix<float,10,10> measurementNoiseCovariance;
        Eigen::Matrix<float,10,10> processNoiseCovariance;
        Eigen::Matrix<float,10,1>  cubaturePoints[20];
        Eigen::Matrix<float,10,1>  propagatedCubaturePoints[20];
        Eigen::Matrix<float,10,1> predictedState;
        Eigen::Matrix<float,10,1> updatedState;
        Eigen::Matrix<float,10,1> predictedMeasurement;
        Eigen::Matrix<float,10,10> priorPredictedErrorCovariance;
        Eigen::Matrix<float,10,10> posteriorPredictedErrorCovariance;
        Eigen::Matrix<float,10,10> innovationCovariance;
        Eigen::Matrix<float,10,10> crossCovariance;
        Eigen::Matrix<float,10,10> kalmanGain;
        Eigen::Matrix<float,10,1> deviation[20];
        Eigen::Matrix<float,10,1> averageInnovation;
        Eigen::Matrix<float,10,1> averageResidual;
        Eigen::Matrix<float,10,10> averageInnovationCovariance;
        Eigen::Matrix<float,10,10> averagePrioPredictedCovariance;

        SatelliteModel satelliteModel;

    private:
        Eigen::Matrix<float,10,10> QRDecomposition(Eigen::Matrix<float,10,30> matrix)
        {
            Eigen::HouseholderQR<Eigen::Matrix<float,30,10>> QRDecomposition;

            Eigen::Matrix<float,30,10> buffer = Eigen::Matrix<float,30,10> (QRDecomposition.compute(matrix.transpose()).matrixQR().triangularView<Eigen::Upper>());

            buffer.diagonal() = -buffer.diagonal();
            
            return Eigen::Matrix<float,10,10>(buffer(Eigen::seq(0,9), Eigen::seq(0,9)));;
        }

        void UpdateAverageInnovationCovariance(Eigen::Matrix<float,10,10> sample)
        {
            averageInnovationCovariance = (1-0.2)*averageInnovationCovariance + 0.2*sample;
        }

        void UpdateAveragePriorCovariance(Eigen::Matrix<float,10,10> sample)
        {
            Eigen::LLT<Eigen::Matrix<float,10,10>> CholeskyDecomposition;

            Eigen::Matrix<float,10,10> bufferTriangular = Eigen::Matrix<float, 10,10>(CholeskyDecomposition.compute(sample).matrixL());

            averagePrioPredictedCovariance = (1-0.2)*averagePrioPredictedCovariance + 0.2*bufferTriangular;
        }

        void UpdateMeasurementCovariance(Eigen::Matrix<float,10,1> sample)
        {
            Eigen::LLT<Eigen::Matrix<float,10,10>> bufferLLT(averageInnovationCovariance);
            bufferLLT.rankUpdate(sample,-1);
            measurementNoiseCovariance = measurementNoiseCovariance*(1-0.25) + 0.25*(Eigen::Matrix<float,10,10>(bufferLLT.matrixL()).transpose());
        }

        void UpdateProcessCovariance(Eigen::Matrix<float,10,1> sample)
        {
            Eigen::LLT<Eigen::Matrix<float,10,10>> bufferLLT(averagePrioPredictedCovariance);
            bufferLLT.rankUpdate(sample,-1);
            processNoiseCovariance = processNoiseCovariance*(1-0.25) + 0.25*(Eigen::Matrix<float,10,10>(bufferLLT.matrixL()).transpose());
        }


    public:
        AdaptiveSquareRootCubatureKalmanFilter(){}
        AdaptiveSquareRootCubatureKalmanFilter(SatelliteModel satelliteModel, Eigen::Matrix<float,10,1> initialState)
        {
            this->satelliteModel = satelliteModel;
            this->updatedState = initialState;

            measurementNoiseCovariance = 1e-5*Eigen::Matrix<float,10,10>::Identity();
            processNoiseCovariance = 1e-5*Eigen::Matrix<float,10,10>::Identity();
            posteriorPredictedErrorCovariance = 1e-3*Eigen::Matrix<float,10,10>::Identity();

            Eigen::LLT<Eigen::Matrix<float,10,10>> CholeskyDecomposition;

            measurementNoiseCovariance = Eigen::Matrix<float, 10,10>(CholeskyDecomposition.compute(measurementNoiseCovariance).matrixL());
            processNoiseCovariance = Eigen::Matrix<float, 10,10>(CholeskyDecomposition.compute(processNoiseCovariance).matrixL());
            posteriorPredictedErrorCovariance = Eigen::Matrix<float, 10,10>(CholeskyDecomposition.compute(posteriorPredictedErrorCovariance).matrixL());

            for(int i = 0; i<10;i++)
            {
                deviation[i] = Eigen::Matrix<float,10,1>::Zero();
                if(i < 6)
                {
                    deviation[i](i) = 1;
                    deviation[i+10](i) = -deviation[i](i);
                }
                else
                {
                    deviation[i](i) = 0.1;
                    deviation[i+10](i) = -deviation[i](i);
                }

            }
        }

        Eigen::Matrix<float,10,1> Filter(Eigen::Matrix<float,10,1> measurement, Eigen::Matrix<float,3,1> controlInput)
        {
            int points = 20; 

            predictedState = Eigen::Matrix<float,10,1>::Zero();

            Eigen::Matrix<float, 10,30> matrixForDecomposition;

            std::cout << "measurement: " << measurement.transpose() << "\n";
            //std::cout << "prior: " << measurement.transpose() << "\n";
            //std::cout << "measurement cov: " << measurementNoiseCovariance.diagonal().transpose() << "\n";
            //std::cout << "process cov: " << processNoiseCovariance.diagonal().transpose() << "\n";
            //std::cout << "innovation avg: " << averageInnovationCovariance.diagonal().transpose() << "\n";
            //std::cout << "prior avg: " << averagePrioPredictedCovariance.diagonal().transpose() << "\n";
            //std::cout << "input: " << controlInput.transpose() << "\n";

            for(int i = 0; i<points;i++)
            {
                
                cubaturePoints[i] = posteriorPredictedErrorCovariance*deviation[i] + updatedState;
                Eigen::Matrix<float,3,1> satelliteVelocity = cubaturePoints[i](Eigen::seq(0,2));
                Eigen::Matrix<float,3,1> reactionWheelVelocities = cubaturePoints[i](Eigen::seq(3,5));
                Eigen::Quaternion<float> attitude(cubaturePoints[i](6),cubaturePoints[i](7),cubaturePoints[i](8),cubaturePoints[i](9));
                
                attitude.normalize();

                SatelliteState state = satelliteModel.Predict(satelliteVelocity,reactionWheelVelocities,attitude, controlInput, 1.0/(900.0));

                for(int j = 0; j<3; j++)
                {
                    cubaturePoints[i](j) = state.satelliteVelocity(j);
                }
                for(int j = 0; j<3; j++)
                {
                    cubaturePoints[i](j+3) = state.reactionWheelVelocities(j);
                }

                cubaturePoints[i](6) = state.attitude.w();
                cubaturePoints[i](7) = state.attitude.x();
                cubaturePoints[i](8) = state.attitude.y();
                cubaturePoints[i](9) = state.attitude.z();

                std::cout << "predicted: " << state.attitude << "\n";
                std::cout << "cubature point: " << cubaturePoints[i].transpose() << "\n";

                predictedState = predictedState + (1.0/points)*cubaturePoints[i];
            }
            std::cout << "prediction: " << predictedState.transpose() << "\n";

            for(int i = 0; i<points; i++)
            {
                matrixForDecomposition.col(i) = (cubaturePoints[i] - predictedState)/(std::sqrt(20.0));
            }

            for(int i = 20; i<30; i++)
            {
                matrixForDecomposition.col(i) = processNoiseCovariance.col(i-20);
            }
            
            priorPredictedErrorCovariance = QRDecomposition(matrixForDecomposition);

            //std::cout << "priorPredictedErrorCovariance: \n" << priorPredictedErrorCovariance << "\n\n";

            matrixForDecomposition = Eigen::Matrix<float, 10,30>::Zero();

            predictedMeasurement = Eigen::Matrix<float,10,1>::Zero();

            for(int i = 0; i<points;i++)
            {
                cubaturePoints[i] = priorPredictedErrorCovariance*deviation[i] + predictedState;

                predictedMeasurement = predictedMeasurement + (1.0/points)*cubaturePoints[i];
            }

            //std::cout << "predictedMeasurement: " << predictedMeasurement.transpose() << "\n";
            //std::cout << "innovation: " << measurement.transpose()-predictedMeasurement.transpose() << "\n";

            /*for(int i = 0; i<points; i++)
            {
                matrixForDecomposition.col(i) = (cubaturePoints[i] - predictedMeasurement)/(std::sqrt(20.0));
            }

            for(int i = 20; i<30; i++)
            {
                matrixForDecomposition.col(i) = measurementNoiseCovariance.col(i-20);
            }*/
            Eigen::LLT<Eigen::Matrix<float,10,10>> CholeskyDecomposition;

            innovationCovariance = Eigen::Matrix<float, 10,10>(CholeskyDecomposition.compute(priorPredictedErrorCovariance*priorPredictedErrorCovariance.transpose() + measurementNoiseCovariance*measurementNoiseCovariance.transpose()).matrixL());

            //std::cout << "innovationCovariance: \n" << innovationCovariance << "\n\n";
            //std::cout << "innovationCovariaceInverse: \n" << innovationCovariance.transpose().inverse() << "\n\n";

        
            /*matrixForDecomposition = Eigen::Matrix<float, 10,30>::Zero();

            for(int i = 0; i<points; i++)
            {
                matrixForDecomposition.col(i) = (cubaturePoints[i] - predictedMeasurement)/(std::sqrt(20.0));
            }*/
            
            UpdateAverageInnovationCovariance(Eigen::Matrix<float, 10,10>(CholeskyDecomposition.compute(priorPredictedErrorCovariance*priorPredictedErrorCovariance.transpose()).matrixL()));

            Eigen::Matrix<float, 10,20> weightedCenteredStateForCrossCovariance;

            for(int i=0; i<20; i++)
            {
                weightedCenteredStateForCrossCovariance.col(i) = priorPredictedErrorCovariance*deviation[i];
            }

            crossCovariance = weightedCenteredStateForCrossCovariance*(weightedCenteredStateForCrossCovariance.transpose());

            //std::cout << "crossCovariance: \n" << crossCovariance << "\n\n";

            //std::cout << "1. : \n" << crossCovariance*(innovationCovariance.inverse()) << "\n\n";

            //std::cout << "2. : \n" << innovationCovariance.inverse()*innovationCovariance.transpose().inverse() << "\n\n";


            kalmanGain = (crossCovariance*(innovationCovariance.inverse()))*innovationCovariance.transpose().inverse();

            //std::cout << "kalmanGain: \n" << kalmanGain << "\n\n";

            updatedState = predictedState + kalmanGain*(measurement - predictedMeasurement);

            matrixForDecomposition << (weightedCenteredStateForCrossCovariance - kalmanGain*weightedCenteredStateForCrossCovariance), kalmanGain*priorPredictedErrorCovariance;
        
            posteriorPredictedErrorCovariance = QRDecomposition(matrixForDecomposition);

            for(int i = 0; i<points; i++)
            {
                matrixForDecomposition.col(i) = (cubaturePoints[i] - predictedState)/(std::sqrt(20.0));
            }
                        
            Eigen::Matrix<float, 10,10> priorPredictedErrorCovarianceNoQ = QRDecomposition(matrixForDecomposition);

            UpdateAveragePriorCovariance(kalmanGain*priorPredictedErrorCovariance*priorPredictedErrorCovariance.transpose());

            Eigen::Quaternion<float> updatedQuaternion(updatedState(6),updatedState(7),updatedState(8),updatedState(9));


            updatedState(6) = updatedQuaternion.w();
            updatedState(7) = updatedQuaternion.x();
            updatedState(8) = updatedQuaternion.y();
            updatedState(9) = updatedQuaternion.z();

            updatedQuaternion.normalize();

            std::cout << "update: " << updatedState.transpose() << "\n\n";

            return updatedState;

        }

        void UpdateCovarianceMatrices(Eigen::Matrix<float,10,1> measurement)
        {
            float forgettingFactor = 0.998;
            
            averageInnovation = averageInnovation*(1-forgettingFactor) + forgettingFactor*(measurement-predictedMeasurement);
            averageResidual = averageResidual*(1-forgettingFactor)  + forgettingFactor*(updatedState-predictedState);

            UpdateMeasurementCovariance(averageInnovation-(measurement-predictedMeasurement));
            UpdateProcessCovariance(averageInnovation-(measurement-predictedMeasurement));
        }
};

#endif