#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <Eigen/LU>
#include <cmath> 

class simpleObserver
{
public:
    simpleObserver(float scalingFactor = 1);
    Eigen::MatrixXf calculateClosestPoints(Eigen::MatrixXf bodyNodePositions, Eigen::MatrixXf pointCloud);
    /* First Calculate a Distance Matrix, then find at each column of the Distance matrix the lowest number,
    // calculate the average Points on the number and return the set of average points
    // bodyNodePositions(n,3) n = number of Body Nodes
    \param pointCloud(l,3) l = number of Points
    \param distMatrix(n,l), n rows and l columns
    // ======== rows ==========
    // =
    // s
    // l
    // o
    // c
    \return Eigen::MatrixXf avgPoints(n,3)
    */
    Eigen::MatrixXf calculateClosestPointsExpFun(Eigen::MatrixXf bodyNodePositions, Eigen::MatrixXf pointCloud);
    Eigen::MatrixXf calculateClosestPointsWeighted(Eigen::MatrixXf bodyNodePositions, Eigen::MatrixXf pointCloud);

protected:
    /*
    \int scaling factor is used to define how much the calculated target point should be trusted
    scaling factor = 1 equals full trust in the target points
    scaling factor = 0 equals no trust in the target points
    target points to send = initial points + scaling * numberOfPoints/avgNumberOfPoints * (target-initial)
    */
    float mScalingFactor;
    void distanceMatrix(Eigen::MatrixXf *distMatrix, Eigen::MatrixXf *avgPoints, Eigen::MatrixXf *bodyNodePositions, Eigen::MatrixXf *pointCloud, int lowerRow, int upperRow,int *numberOfPoints);
    float fun(float x);
};
