#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "skeleton.h"

class PointCloud
{
public:
    PointCloud(Skeleton *skl);  // generate a point cloud according to the skeleton
    ~PointCloud();
    void draw();

private:
    void traverse(Bone *ptr);
    void genBonePoints(Bone *pBone);

private:
    vector *points;             // points in global coordinates
    int numPoints;
};

#endif // POINTCLOUD_H
