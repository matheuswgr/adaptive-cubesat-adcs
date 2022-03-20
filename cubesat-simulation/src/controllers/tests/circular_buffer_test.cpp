#include "circular_buffer.h"
#include <iostream>
#include <eigen3/Eigen/Dense>

int main()
{
    Eigen::Matrix<float,2,1> x1 {1,1};
    Eigen::Matrix<float,2,1> x2 {2,2};
    Eigen::Matrix<float,2,1> x3 {3,3};
    Eigen::Matrix<float,2,1> x4 {4,4};
    Eigen::Matrix<float,2,1> x5 {5,5};
    Eigen::Matrix<float,2,1> x6 {6,6};

    CircularBuffer<Eigen::Matrix<float,2,1>> circularBuffer(4);

    circularBuffer.push(x1);

    
    Iterator<Eigen::Matrix<float,2,1>>* iterator = circularBuffer.getIterator();
    while(iterator->hasNext())
    {
        Eigen::Matrix<float,2,1>* v = iterator->next();
        std::cout << "[" << (*v)(0) << " , " << (*v)(1) << "]" <<"\n";
    }

    std::cout << "\n";

    circularBuffer.push(x2);

    iterator = circularBuffer.getIterator();
    while(iterator->hasNext())
    {
        Eigen::Matrix<float,2,1>* v = iterator->next();
        std::cout << "[" << (*v)(0) << " , " << (*v)(1) << "]" <<"\n";
    }

    std::cout << "\n";

    circularBuffer.push(x3);

    iterator = circularBuffer.getIterator();
    while(iterator->hasNext())
    {
        Eigen::Matrix<float,2,1>* v = iterator->next();
        std::cout << "[" << (*v)(0) << " , " << (*v)(1) << "]" <<"\n";
    }

    std::cout << "\n";


    circularBuffer.push(x4);

    iterator = circularBuffer.getIterator();
    while(iterator->hasNext())
    {
        Eigen::Matrix<float,2,1>* v = iterator->next();
        std::cout << "[" << (*v)(0) << " , " << (*v)(1) << "]" <<"\n";
    }

    std::cout << "\n";


    circularBuffer.push(x5);

    iterator = circularBuffer.getIterator();
    while(iterator->hasNext())
    {
        Eigen::Matrix<float,2,1>* v = iterator->next();
        std::cout << "[" << (*v)(0) << " , " << (*v)(1) << "]" <<"\n";
    }
    
    std::cout << "\n";


    circularBuffer.push(x6);

    iterator = circularBuffer.getIterator();
    while(iterator->hasNext())
    {
        Eigen::Matrix<float,2,1>* v = iterator->next();
        std::cout << "[" << (*v)(0) << " , " << (*v)(1) << "]" <<"\n";
    }


    std::cout << "\n";

    circularBuffer.push(x1);

    iterator = circularBuffer.getIterator();
    while(iterator->hasNext())
    {
        Eigen::Matrix<float,2,1>* v = iterator->next();
        std::cout << "[" << (*v)(0) << " , " << (*v)(1) << "]" <<"\n";
    }
    std::cout << "\n";


        circularBuffer.push(x2);

    iterator = circularBuffer.getIterator();
    while(iterator->hasNext())
    {
        Eigen::Matrix<float,2,1>* v = iterator->next();
        std::cout << "[" << (*v)(0) << " , " << (*v)(1) << "]" <<"\n";
    }

    std::cout << "\n";


    circularBuffer.push(x3);

    iterator = circularBuffer.getIterator();
    while(iterator->hasNext())
    {
        Eigen::Matrix<float,2,1>* v = iterator->next();
        std::cout << "[" << (*v)(0) << " , " << (*v)(1) << "]" <<"\n";
    }
    return 0;
}