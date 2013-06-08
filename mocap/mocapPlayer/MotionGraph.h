#ifndef MOTIONGRAPH_H
#define MOTIONGRAPH_H

#include "MotionLibrary.h"
#include "PointCloud.h"

class MotionGraph
{
public:
    MotionGraph();

    void genCandidates(int motionIdx1, int motionIdx2, int k);

private:
    void getTransformMatrix(double *matrix, double theta, double x0, double z0);
    double distance(PointCloud *pcA, PointCloud *pcB);

private:
    MotionLibrary *library;
};


#endif // MOTIONGRAPH_H
