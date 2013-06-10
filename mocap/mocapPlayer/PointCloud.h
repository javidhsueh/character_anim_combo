#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "skeleton.h"

void copyMatrix(double dst[], const double src[]);
vector matMultVec3(const double m[], const vector &v);

class PointCloud
{
public:
    PointCloud(Skeleton *skl);                  // generate a point cloud according to the skeleton
    PointCloud(PointCloud **pc, int k);         // construct a point cloud formed over a window of frames of length k
    ~PointCloud();
    int getNumPoints()  { return numPoints; }
    vector *getPoints() { return points; }
    void setTransformMatrix(double matrix[]);
    void draw();

private:
    void traverse(Bone *ptr);
    void genBonePoints(Bone *pBone);

private:
    vector *points;             // points in global coordinates
    int numPoints;
    double transformMatrix[16];
};

#endif // POINTCLOUD_H
