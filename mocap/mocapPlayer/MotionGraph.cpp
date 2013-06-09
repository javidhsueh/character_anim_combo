#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>

#include <FL/gl.h>

#include "MotionGraph.h"

struct WindowPair
{
    int i, j;
    double dist;
    bool operator < (const WindowPair &other) const { return dist < other.dist; }
};

MotionGraph::MotionGraph()
{
}

MotionGraph::MotionGraph(MotionLibrary *lib)
    : library(lib)
{
}

// theta := in radian
void MotionGraph::getTransformMatrix(double *matrix, double theta, double x0, double z0)
{
    glPushMatrix();

    glLoadIdentity();
    
    glTranslated(x0, 0.0, z0);
    glRotated(theta * 180.0 / M_PI, 0.0, 1.0, 0.0);

    glGetDoublev(GL_MODELVIEW_MATRIX, matrix);

    glPopMatrix();
}

// distance between two point clouds
// return := T(matrix), pa ~= T * pb
double MotionGraph::distance(PointCloud *pcA, PointCloud *pcB, double *matrix)
{
    if (pcA->getNumPoints() != pcB->getNumPoints())
    {
        printf("Error: in MotionGraph::getTransformMatrix().\n");
        exit(-1);
    }
    double numPoints = min(pcA->getNumPoints(), pcB->getNumPoints());       // the two numbers should be the same
    vector *pa = pcA->getPoints();
    vector *pb = pcB->getPoints();

    double w = 1.0;                                 // weight
    double sumWeight = w * numPoints;
    
    // calculate x_bar, x'_bar, z_bar, z'_bar
    double xaBar = 0.0, zaBar = 0.0;
    double xbBar = 0.0, zbBar = 0.0;
    for (int i = 0; i < numPoints; i++)
    {
        xaBar += w * pa[i].x();
        zaBar += w * pa[i].z();
    }
    for (int i = 0; i < numPoints; i++)
    {
        xbBar += w * pb[i].x();
        zbBar += w * pb[i].z();
    }

    // calculate theta, x0, z0
    double nom = 0.0;
    double denom = 0.0;
    for (int i = 0; i < numPoints; i++)
    {
        nom += w * (pa[i].x() * pb[i].z() - pb[i].x() * pa[i].z());
        denom += w * (pa[i].x() * pb[i].x() + pa[i].z() * pb[i].z());
    }
    nom -= (xaBar * zbBar - xbBar * zaBar) / sumWeight;
    denom -= (xaBar * xbBar + zaBar * zbBar) / sumWeight;

    double theta = atan(nom / denom);
    double x0 = (xaBar - xbBar * cos(theta) - zbBar * sin(theta)) / sumWeight;
    double z0 = (zaBar + xbBar * sin(theta) - zbBar * cos(theta)) / sumWeight;

    // sometimes you get wrong theta
    // try opposite direction
    double theta2 = theta + M_PI;
    double x02 = (xaBar - xbBar * cos(theta2) - zbBar * sin(theta2)) / sumWeight;
    double z02 = (zaBar + xbBar * sin(theta2) - zbBar * cos(theta2)) / sumWeight;

    double m[16], m2[16];
    getTransformMatrix(m, theta, x0, z0);
    getTransformMatrix(m2, theta2, x02, z02);
    
    // calculate distance
    double dist = 0.0;
    double dist2 = 0.0;
    for (int i = 0; i < numPoints; i++)
    {
        vector tpbi = matMultVec3(m, pb[i]);
        double dx = pa[i].x() - tpbi.x();
        double dy = pa[i].y() - tpbi.y();
        double dz = pa[i].z() - tpbi.z();
        dist += w * (dx * dx + dy * dy + dz * dz);

        vector tpbi2 = matMultVec3(m2, pb[i]);
        double dx2 = pa[i].x() - tpbi2.x();
        double dy2 = pa[i].y() - tpbi2.y();
        double dz2 = pa[i].z() - tpbi2.z();
        dist2 += w * (dx2 * dx2 + dy2 * dy2 + dz2 * dz2);
    }

    if (dist <= dist2)
    {
        if (matrix != NULL)
            copyMatrix(matrix, m);
        return dist;
    }
    else
    {
        if (matrix != NULL)
            copyMatrix(matrix, m2);
        return dist2;
    }
}

// distance between two point clouds
double MotionGraph::distance(PointCloud *pcA, PointCloud *pcB, double *oTheta, double *oX0, double *oZ0)
{
    if (pcA->getNumPoints() != pcB->getNumPoints())
    {
        printf("Error: in MotionGraph::getTransformMatrix().\n");
        exit(-1);
    }
    double numPoints = min(pcA->getNumPoints(), pcB->getNumPoints());       // the two numbers should be the same
    vector *pa = pcA->getPoints();
    vector *pb = pcB->getPoints();

    double w = 1.0;                                 // weight
    double sumWeight = w * numPoints;
    
    // calculate x_bar, x'_bar, z_bar, z'_bar
    double xaBar = 0.0, zaBar = 0.0;
    double xbBar = 0.0, zbBar = 0.0;
    for (int i = 0; i < numPoints; i++)
    {
        xaBar += w * pa[i].x();
        zaBar += w * pa[i].z();
    }
    for (int i = 0; i < numPoints; i++)
    {
        xbBar += w * pb[i].x();
        zbBar += w * pb[i].z();
    }

    // calculate theta, x0, z0
    double nom = 0.0;
    double denom = 0.0;
    for (int i = 0; i < numPoints; i++)
    {
        nom += w * (pa[i].x() * pb[i].z() - pb[i].x() * pa[i].z());
        denom += w * (pa[i].x() * pb[i].x() + pa[i].z() * pb[i].z());
    }
    nom -= (xaBar * zbBar - xbBar * zaBar) / sumWeight;
    denom -= (xaBar * xbBar + zaBar * zbBar) / sumWeight;

    double theta = atan(nom / denom);
    double x0 = (xaBar - xbBar * cos(theta) - zbBar * sin(theta)) / sumWeight;
    double z0 = (zaBar + xbBar * sin(theta) - zbBar * cos(theta)) / sumWeight;

    // sometimes you get wrong theta
    // try opposite direction
    double theta2 = theta + M_PI;
    double x02 = (xaBar - xbBar * cos(theta2) - zbBar * sin(theta2)) / sumWeight;
    double z02 = (zaBar + xbBar * sin(theta2) - zbBar * cos(theta2)) / sumWeight;

    double m[16], m2[16];
    getTransformMatrix(m, theta, x0, z0);
    getTransformMatrix(m2, theta2, x02, z02);
    
    // calculate distance
    double dist = 0.0;
    double dist2 = 0.0;
    for (int i = 0; i < numPoints; i++)
    {
        vector tpbi = matMultVec3(m, pb[i]);
        double dx = pa[i].x() - tpbi.x();
        double dy = pa[i].y() - tpbi.y();
        double dz = pa[i].z() - tpbi.z();
        dist += w * (dx * dx + dy * dy + dz * dz);

        vector tpbi2 = matMultVec3(m2, pb[i]);
        double dx2 = pa[i].x() - tpbi2.x();
        double dy2 = pa[i].y() - tpbi2.y();
        double dz2 = pa[i].z() - tpbi2.z();
        dist2 += w * (dx2 * dx2 + dy2 * dy2 + dz2 * dz2);
    }

    if (dist <= dist2)
    {
        if (oTheta != NULL && oX0 != NULL && oZ0 != NULL)
        {
            *oTheta = theta;
            *oX0 = x0;
            *oZ0 = z0;
        }
        return dist;
    }
    else
    {
        if (oTheta != NULL && oX0 != NULL && oZ0 != NULL)
        {
            *oTheta = theta2;
            *oX0 = x02;
            *oZ0 = z02;
        }
        return dist2;
    }
}

/*
// distance between two point clouds
double MotionGraph::distance(PointCloud *pcA, PointCloud *pcB)
{
    if (pcA->getNumPoints() != pcB->getNumPoints())
    {
        printf("Error: in MotionGraph::distance().\n");
        exit(-1);
    }
    double numPoints = min(pcA->getNumPoints(), pcB->getNumPoints());       // the two numbers should be the same
    vector *pa = pcA->getPoints();
    vector *pb = pcB->getPoints();
    double w = 1.0;                     // weight
    
    double matrix[16];
    getTransformMatrix(matrix, pcA, pcB);

    double dist = 0.0;
    for (int i = 0; i < numPoints; i++)
    {
        vector tpbi = matMultVec3(matrix, pb[i]);
        double dx = pa[i].x() - tpbi.x();
        double dy = pa[i].y() - tpbi.y();
        double dz = pa[i].z() - tpbi.z();
        dist += w * (dx * dx + dy * dy + dz * dz);
    }

    return dist;
}
*/

// k := window size
void MotionGraph::genCandidates(int motionIdxA, int motionIdxB, int k)
{
    Skeleton *skeleton = library->getSkeleton();
    Motion *motionA = library->getMotion(motionIdxA);
    Motion *motionB = library->getMotion(motionIdxB);
    if (skeleton == NULL || motionA == NULL || motionB == NULL)
    {
        printf("Error: in genCandidates().\n");
        exit(-1);
    }

    int numFramesA = motionA->GetNumFrames();
    int numFramesB = motionB->GetNumFrames();

    printf("numFramesA = %d, numFramesB = %d\n", numFramesA, numFramesB);

    PointCloud **pcListA = new PointCloud *[numFramesA];
    PointCloud **pcListB = new PointCloud *[numFramesB];

    for (int i = 0; i < numFramesA; i++)
    {
        skeleton->setPosture(*(motionA->GetPosture(i)));
        pcListA[i] = new PointCloud(skeleton);
    }
    for (int i = 0; i < numFramesB; i++)
    {
        skeleton->setPosture(*(motionB->GetPosture(i)));
        pcListB[i] = new PointCloud(skeleton);
    }

    double *dist = new double[numFramesA * numFramesB]();   // distance of any pair of windows

    int iBegin = 0;
    int iEnd = numFramesA - k;
    int jBegin = k - 1;
    int jEnd = numFramesB - 1;

    printf("Calculating distances...\n");

    for (int i = iBegin; i <= iEnd; i++)             // window of motionA: [i..i+k-1]
    {
        printf("Calculating (%d, *)...\n", i);

        for (int j = jBegin; j <= jEnd; j++)         // window of motionB: [j-k+1..j]
        {
            // form point cloud over k frames
            PointCloud pcA(&pcListA[i], k);
            PointCloud pcB(&pcListB[j - k + 1], k);

            int index = i * numFramesB + j;
            dist[index] = distance(&pcA, &pcB);     // distance
        }
    }

    printf("Calculating distances... done.\n");

    //std::vector< std::pair<int, int> > minList;     // local minimum list
    // local minimum list
    std::vector<WindowPair> minList;
    
    // search local minimums
    //bool *mark = new bool[numFramesA * numFramesB]();   // traversed
    for (int i = iBegin; i <= iEnd; i++)
    {
        for (int j = jBegin; j <= jEnd; j++)
        {
            int index = i * numFramesB + j;
            bool isMin = true;
            for (int ii = i - 2; ii <= i + 2; ii++)
            {
                for (int jj = j - 2; jj <= j + 2; jj++)
                {
                    if (ii >= iBegin && ii <= iEnd &&
                        jj >= jBegin && jj <= jEnd &&
                        dist[index] > dist[ii * numFramesB + jj])
                        isMin = false;
                }
            }
            if (isMin)
            {
                WindowPair pair;
                pair.i = i;
                pair.j = j;
                pair.dist = dist[index];
                minList.push_back(pair);
            }
        }
    }

    printf("Local minimum list:\n");

    std::sort(minList.begin(), minList.end());

    for (size_t i = 0; i < minList.size(); i++)
    {
        double theta, x0, z0;
        PointCloud pcA(&pcListA[minList[i].i], k);
        PointCloud pcB(&pcListB[minList[i].j - k + 1], k);
        distance(&pcA, &pcB, &theta, &x0, &z0);
        printf("%4d %4d %8.4lf   %lf %lf %lf\n", minList[i].i, minList[i].j, minList[i].dist, theta, x0, z0);
    }
    
    delete [] dist;

    for (int i = 0; i < numFramesA; i++)
        delete pcListA[i];
    for (int i = 0; i < numFramesB; i++)
        delete pcListB[i];
    delete [] pcListA;
    delete [] pcListB;
    
    printf("genCandidates(): done.\n");
}
