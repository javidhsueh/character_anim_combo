#include <cstdio>
#include <cstdlib>
#include <algorithm>

#include <FL/gl.h>

#include "MotionGraph.h"

vector matMultVec3(const double m[], const vector &v);

MotionGraph::MotionGraph()
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

double MotionGraph::distance(PointCloud *pcA, PointCloud *pcB)
{
    if (pcA->getNumPoints() != pcB->getNumPoints())
    {
        printf("Error: in MotionGraph::distance().\n");
        exit(-1);
    }
    double numPoints = min(pcA->getNumPoints(), pcB->getNumPoints());       // the two numbers should be the same
    double xaBar = 0.0, zaBar = 0.0;
    double xbBar = 0.0, zbBar = 0.0;
    double w = 1.0;                                 // weight
    double sumWeight = w * numPoints;

    vector *pa = pcA->getPoints();
    vector *pb = pcB->getPoints();

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

    double nom = 0.0;
    double denom = 0.0;
    for (int i = 0; i < numPoints; i++)
    {
        nom += w * (pa[i].x() * pb[i].z() - pb[i].x() * pa[i].z());
        denom += w * (pa[i].x() * pb[i].x() - pa[i].z() * pb[i].z());
    }
    nom -= (xaBar * zbBar - xbBar * zaBar) / sumWeight;
    denom -= (xaBar * xbBar + zaBar * zbBar) / sumWeight;

    double theta = atan(nom / denom);
    double x0 = (xaBar - xbBar * cos(theta) - zbBar * sin(theta)) / sumWeight;
    double z0 = (zaBar + xbBar * sin(theta) - zbBar * cos(theta)) / sumWeight;

    double matrix[16];
    getTransformMatrix(matrix, theta, x0, z0);

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

    for (int i = 0; i <= numFramesA - k; i++)       // window of motionA: [i..i+k-1]
    {
        for (int j = k - 1; j < numFramesB; j++)    // window of motionB: [j-k+1..j]
        {
            PointCloud *pcA = new PointCloud(&pcListA[i], k);
            PointCloud *pcB = new PointCloud(&pcListB[j], k);




            delete pcA;
            delete pcB;
        }
    }

    for (int i = 0; i < numFramesA; i++)
        delete pcListA[i];
    for (int i = 0; i < numFramesB; i++)
        delete pcListB[i];
    delete [] pcListA;
    delete [] pcListB;
    
}
