#ifndef MOTIONGRAPH_H
#define MOTIONGRAPH_H

#include "MotionLibrary.h"
#include "PointCloud.h"

class MotionGraph
{
public:
    MotionGraph();
    MotionGraph(MotionLibrary *lib);

    void genCandidates(int motionIdx1, int motionIdx2, int k);
    double distance(PointCloud *pcA, PointCloud *pcB, double *matrix = NULL);
    double distance(PointCloud *pcA, PointCloud *pcB, double *oTheta, double *oX0, double *oZ0);
    //double distance(PointCloud *pcA, PointCloud *pcB);

private:
    void getTransformMatrix(double *matrix, double theta, double x0, double z0);
    
    

private:
    MotionLibrary *library;
};


#endif // MOTIONGRAPH_H
