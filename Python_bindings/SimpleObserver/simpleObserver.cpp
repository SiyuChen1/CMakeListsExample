#include "simpleObserver.hpp"
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
/*
Bindings Class for simpleObserver
*/

namespace py = pybind11;

PYBIND11_MODULE(simpleObserver,
                m)
{

    py::class_<simpleObserver>(m, "simpleObserver")
        .def(py::init<>())
        .def(py::init<float&>())
        .def("calculateClosestPoints", &simpleObserver::calculateClosestPoints)
        .def("calculateClosestPointsExpFun", &simpleObserver::calculateClosestPointsExpFun)
        .def("calculateClosestPointsWeighted", &simpleObserver::calculateClosestPointsWeighted);
}

simpleObserver::simpleObserver(float scalingFactor)
{
    mScalingFactor = scalingFactor;
    // std::cout << "Empty Constructor" << std::endl;
}

void simpleObserver::distanceMatrix(Eigen::MatrixXf *distMatrix, Eigen::MatrixXf *avgPoints, Eigen::MatrixXf *bodyNodePositions, Eigen::MatrixXf *pointCloud, int lowerRow, int upperRow, int *numberOfPoints)
{
    for (int s = lowerRow; s < upperRow; s++)
    {
        distMatrix->row(s) =
            (pointCloud->rowwise() - bodyNodePositions->row(s)).matrix().rowwise().norm();
        numberOfPoints[s] = 0;
        (*avgPoints)(s, 0) = 0;
        (*avgPoints)(s, 1) = 0;
        (*avgPoints)(s, 2) = 0;
    }
}

Eigen::MatrixXf simpleObserver::calculateClosestPoints(Eigen::MatrixXf bodyNodePositions, Eigen::MatrixXf pointCloud)
{
    // First Calculate a Distance Matrix, then find at each column of the Distance matrix the lowest number,
    // calculate the average Points on the number and return the set of average points
    // bodyNodePositions(n,3) n = number of Body Nodes
    // pointCloud(l,3) l = number of Points
    // distMatrix(n,l), n rows and l columns
    // ======== rows ==========
    // =
    // c
    // o
    // l
    // s
    // =
    auto distMatrix = Eigen::MatrixXf(bodyNodePositions.rows(), pointCloud.rows());
    auto avgPoints = Eigen::MatrixXf(bodyNodePositions.rows(), 3);
    int* numberOfPoints = new int[bodyNodePositions.rows()];

    // auto time_1 =
    //     std::chrono::system_clock::now();
    if (bodyNodePositions.rows() > 3)
    {
        Eigen::initParallel();
        auto pointsPerThread = std::div((int)distMatrix.rows(), 4);

        std::thread firstDist(&simpleObserver::distanceMatrix, this, &distMatrix, &avgPoints, &bodyNodePositions, &pointCloud, 0 * pointsPerThread.quot, 1 * pointsPerThread.quot, numberOfPoints);
        std::thread secondDist(&simpleObserver::distanceMatrix, this, &distMatrix , &avgPoints, &bodyNodePositions, &pointCloud, 1 * pointsPerThread.quot, 2 * pointsPerThread.quot, numberOfPoints);
        std::thread thirdDist(&simpleObserver::distanceMatrix,this, &distMatrix, &avgPoints, &bodyNodePositions, &pointCloud, 2 * pointsPerThread.quot, 3 * pointsPerThread.quot, numberOfPoints);
        std::thread fourthDist(&simpleObserver::distanceMatrix,this, &distMatrix, &avgPoints, &bodyNodePositions, &pointCloud, 3 * pointsPerThread.quot, distMatrix.rows(), numberOfPoints);
        firstDist.join();
        secondDist.join();
        thirdDist.join();
        fourthDist.join();
    }
    else
    {

        for (int s = 0; s < bodyNodePositions.rows(); s++)
        {
            distMatrix.row(s) =
                (pointCloud.rowwise() - bodyNodePositions.row(s)).matrix().rowwise().norm();
            numberOfPoints[s] = 0;
            avgPoints(s, 0) = 0;
            avgPoints(s, 1) = 0;
            avgPoints(s, 2) = 0;
        }
    }
    // auto time_2 =
    //     std::chrono::system_clock::now();

    // ***** New Try ******
    int i;

    for (int s = 0; s < distMatrix.cols(); s++)
    {
        distMatrix.col(s).minCoeff(&i);
        avgPoints.row(i) += pointCloud.row(s);
        numberOfPoints[i] += 1;
    }
    // auto time_3 =
    //     std::chrono::system_clock::now();

    for (int s = 0; s < bodyNodePositions.rows(); s++)
    {
        if (numberOfPoints[s] == 0)
        {
            avgPoints.row(s) = bodyNodePositions.row(s);
        }
        else
        {
            // std::cout << avgPoints.row(s) << std::endl;
            avgPoints.row(s) = avgPoints.row(s) / numberOfPoints[s];
            // std::cout << avgPoints.row(s) << std::endl;
        };
    }
    // auto time_4 =
    //     std::chrono::system_clock::now();

    // std::cout << "\nTime1-2: " << std::chrono::duration_cast<std::chrono::microseconds>(time_2 - time_1).count()
    //           << " Time2-3: " << std::chrono::duration_cast<std::chrono::microseconds>(time_3 - time_2).count()
    //           << " Time3-4: " << std::chrono::duration_cast<std::chrono::microseconds>(time_4 - time_3).count() << std::endl;

    return avgPoints;
}

Eigen::MatrixXf simpleObserver::calculateClosestPointsWeighted(Eigen::MatrixXf bodyNodePositions, Eigen::MatrixXf pointCloud)
{
    // First Calculate a Distance Matrix, then find at each column of the Distance matrix the lowest number,
    // calculate the average Points on the number and return the set of average points
    // bodyNodePositions(n,3) n = number of Body Nodes
    // pointCloud(l,3) l = number of Points
    // distMatrix(n,l), n rows and l columns
    // ======== rows ==========
    // =
    // c
    // o
    // l
    // s
    // =
    auto distMatrix = Eigen::MatrixXf(bodyNodePositions.rows(), pointCloud.rows());
    auto avgPoints = Eigen::MatrixXf(bodyNodePositions.rows(), 3);
    int* numberOfPoints = new int[bodyNodePositions.rows()];

    // auto time_1 =
    //     std::chrono::system_clock::now();
    if (bodyNodePositions.rows() > 3)
    {
        Eigen::initParallel();
        auto pointsPerThread = std::div((int)distMatrix.rows(), 4);

        std::thread firstDist(&simpleObserver::distanceMatrix, this, &distMatrix, &avgPoints, &bodyNodePositions, &pointCloud, 0 * pointsPerThread.quot, 1 * pointsPerThread.quot, numberOfPoints);
        std::thread secondDist(&simpleObserver::distanceMatrix, this, &distMatrix , &avgPoints, &bodyNodePositions, &pointCloud, 1 * pointsPerThread.quot, 2 * pointsPerThread.quot, numberOfPoints);
        std::thread thirdDist(&simpleObserver::distanceMatrix,this, &distMatrix, &avgPoints, &bodyNodePositions, &pointCloud, 2 * pointsPerThread.quot, 3 * pointsPerThread.quot, numberOfPoints);
        std::thread fourthDist(&simpleObserver::distanceMatrix,this, &distMatrix, &avgPoints, &bodyNodePositions, &pointCloud, 3 * pointsPerThread.quot, distMatrix.rows(), numberOfPoints);
        firstDist.join();
        secondDist.join();
        thirdDist.join();
        fourthDist.join();
    }
    else
    {

        for (int s = 0; s < bodyNodePositions.rows(); s++)
        {
            distMatrix.row(s) =
                (pointCloud.rowwise() - bodyNodePositions.row(s)).matrix().rowwise().norm();
            numberOfPoints[s] = 0;
            avgPoints(s, 0) = 0;
            avgPoints(s, 1) = 0;
            avgPoints(s, 2) = 0;
        }
    }
    // auto time_2 =
    //     std::chrono::system_clock::now();

    // ***** New Try ******
    int i;

    for (int s = 0; s < distMatrix.cols(); s++)
    {
        distMatrix.col(s).minCoeff(&i);
        avgPoints.row(i) += pointCloud.row(s);
        numberOfPoints[i] += 1;
    }
    // auto time_3 =
    //     std::chrono::system_clock::now();

    for (int s = 0; s < bodyNodePositions.rows(); s++)
    {
        if (numberOfPoints[s] == 0)
        {
            avgPoints.row(s) = bodyNodePositions.row(s);
            // if (s != 0 && numberOfPoints[s - 1] != 0) {
            //     avgPoints.row(s) = bodyNodePositions.row(s) + mScalingFactor * numberOfPoints[s - 1] / (pointCloud.rows() / bodyNodePositions.rows()) * (avgPoints.row(s - 1) / numberOfPoints[s - 1] + bodyNodePositions.row(s) - bodyNodePositions.row(s - 1));
            // }
        }
        else
        {
            // std::cout << avgPoints.row(s) << std::endl;
            avgPoints.row(s) = bodyNodePositions.row(s) + mScalingFactor *  numberOfPoints[s] / (pointCloud.rows()/bodyNodePositions.rows()) * (avgPoints.row(s) / numberOfPoints[s] - bodyNodePositions.row(s));
            // std::cout << avgPoints.row(s) << std::endl;
        };
    }
    // auto time_4 =
    //     std::chrono::system_clock::now();

    // std::cout << "\nTime1-2: " << std::chrono::duration_cast<std::chrono::microseconds>(time_2 - time_1).count()
    //           << " Time2-3: " << std::chrono::duration_cast<std::chrono::microseconds>(time_3 - time_2).count()
    //           << " Time3-4: " << std::chrono::duration_cast<std::chrono::microseconds>(time_4 - time_3).count() << std::endl;

    return avgPoints;
}


float simpleObserver::fun(float x) // the functor we want to apply
{
    return 1/x;
}

Eigen::MatrixXf simpleObserver::calculateClosestPointsExpFun(Eigen::MatrixXf bodyNodePositions, Eigen::MatrixXf pointCloud)
{
    // Calculate the e_fun cost
    // bodyNodePositions(n,3) n = number of Body Nodes
    // pointCloud(l,3) l = number of Points
    // distMatrix(n,l), n rows and l columns
    // ======== rows ==========
    // =
    // c
    // o
    // l
    // s
    // =
    auto distMatrix = Eigen::MatrixXf(bodyNodePositions.rows(), pointCloud.rows());
    auto avgPoints = Eigen::MatrixXf(bodyNodePositions.rows(), 3);
    int* numberOfPoints = new int[bodyNodePositions.rows()];

    // auto time_1 =
    //     std::chrono::system_clock::now();
    if (bodyNodePositions.rows() > 3)
    {
        Eigen::initParallel();
        auto pointsPerThread = std::div((int)distMatrix.rows(), 4);

        std::thread firstDist(&simpleObserver::distanceMatrix, this, &distMatrix, &avgPoints, &bodyNodePositions, &pointCloud, 0 * pointsPerThread.quot, 1 * pointsPerThread.quot, numberOfPoints);
        std::thread secondDist(&simpleObserver::distanceMatrix, this, &distMatrix , &avgPoints, &bodyNodePositions, &pointCloud, 1 * pointsPerThread.quot, 2 * pointsPerThread.quot, numberOfPoints);
        std::thread thirdDist(&simpleObserver::distanceMatrix,this, &distMatrix, &avgPoints, &bodyNodePositions, &pointCloud, 2 * pointsPerThread.quot, 3 * pointsPerThread.quot, numberOfPoints);
        std::thread fourthDist(&simpleObserver::distanceMatrix,this, &distMatrix, &avgPoints, &bodyNodePositions, &pointCloud, 3 * pointsPerThread.quot, distMatrix.rows(), numberOfPoints);
        firstDist.join();
        secondDist.join();
        thirdDist.join();
        fourthDist.join();
    }
    else
    {

        for (int s = 0; s < bodyNodePositions.rows(); s++)
        {
            distMatrix.row(s) =
                (pointCloud.rowwise() - bodyNodePositions.row(s)).matrix().rowwise().norm();
            numberOfPoints[s] = 0;
            avgPoints(s, 0) = 0;
            avgPoints(s, 1) = 0;
            avgPoints(s, 2) = 0;
        }
    }
    // auto time_2 =
    //     std::chrono::system_clock::now();

    // use distMatrix(n,l), n bodynode rows and l pointcloud columns with e^-dist
    Eigen::MatrixXf weightMatrix = (-1 * distMatrix).array().exp();
    // std::cout << "weightMatrix has the size of rows: " << weightMatrix.rows() << " and cols: " << weightMatrix.cols() << std::endl;

    Eigen::MatrixXf sumWeightRowWise = weightMatrix.rowwise().sum();
    // std::cout << "sumWeightRowWise has the size of rows: " << sumWeightRowWise.rows() << " and cols: " << sumWeightRowWise.cols() << std::endl;

    // std::cout << "pointCloud has the size of rows: " << pointCloud.rows() << " and cols: " << pointCloud.cols() << std::endl;

    auto weightM = weightMatrix * pointCloud;

    for (int s = 0; s < bodyNodePositions.rows(); s++) {
        avgPoints(s,0) = weightM(s,0) / 1/ sumWeightRowWise(s);
        avgPoints(s,1) = weightM(s,1) / 1/ sumWeightRowWise(s);
        avgPoints(s,2) = weightM(s,2) / 1/ sumWeightRowWise(s);
    }

    // std::cout << "pointWeight has the size of rows: " << pointWeight.rows() << " and cols: " << pointWeight.cols() << std::endl;

    
    // avgPoints = (-1 * distMatrix).array().exp().rowwise().sum();

    return avgPoints;
}