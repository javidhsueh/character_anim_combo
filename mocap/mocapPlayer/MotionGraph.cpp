#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>

#include <FL/gl.h>

#include "MotionGraph.h"
#include "Transition.h"

struct WindowPair
{
    int i, j;
    double dist;
    double theta, x0, z0;
    bool operator < (const WindowPair &other) const { return dist < other.dist; }
};

MotionGraph::MotionGraph()
    : numBlendingFrames(40),
      restLabel(0)
{
    resetTransformMatrix();
}

MotionGraph::MotionGraph(MotionLibrary *lib)
    : library(lib),
      numBlendingFrames(40),
      restLabel(0)
{
    resetTransformMatrix();
}

MotionGraph::~MotionGraph()
{
    for (size_t i = 0; i < clips.size(); i++)
        delete clips[i];
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
void MotionGraph::genCandidates(std::ofstream &os, int motionIdxA, int motionIdxB, int k)
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

    printf("motionIdxA = %d, motionIdxB = %d\n", motionIdxA, motionIdxB);
    printf("numFramesA = %d, numFramesB = %d\n", numFramesA, numFramesB);
    os << "#motionIndex" << std::endl << motionIdxA << ' ' << motionIdxB << std::endl;
    os << "#motionName" << std::endl << library->getName(motionIdxA) << ' ' << library->getName(motionIdxB) << std::endl;
    os << "#numFrames" << std::endl << numFramesA << ' ' << numFramesB << std::endl;

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
        if (i % 10 == 0)
            printf("Calculating (%d, %d)...\n", i, 0);

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

    std::sort(minList.begin(), minList.end());

    printf("Local minimum list:\n");

    int minListSize = 0;
    for (size_t i = 0; i < minList.size(); i++)
    {
        if (!(motionIdxA == motionIdxB && minList[i].i + 39 == minList[i].j))
            minListSize++;
    }

    os << "#minListSize" << std::endl << minListSize << std::endl;
    os << "#i j distance theta x0 z0" << std::endl;

    for (size_t i = 0; i < minList.size(); i++)
    {
        double theta, x0, z0;
        PointCloud pcA(&pcListA[minList[i].i], k);
        PointCloud pcB(&pcListB[minList[i].j - k + 1], k);
        distance(&pcA, &pcB, &theta, &x0, &z0);
        //printf("%4d %4d %8.4lf   %lf %lf %lf\n", minList[i].i, minList[i].j, minList[i].dist, theta, x0, z0);
        if (!(motionIdxA == motionIdxB && minList[i].i + 39 == minList[i].j))
            os << minList[i].i << ' ' << minList[i].j << ' ' << minList[i].dist << ' ' << theta << ' ' << x0 << ' ' << z0 << std::endl;
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

void MotionGraph::genAllCandidates(int k)
{
    std::ofstream ofs;
    ofs.open("./mocap_data/candidates.txt", std::ios::out);

    int numMotions = library->getNumMotions();

    ofs << "#numMotions" << std::endl;
    ofs << numMotions << std::endl;

    for (int i = 0; i < numMotions; i++)
    //for (int i = numMotions - 1; i >= 0; i--)
    //int i = 7;
    {
        for (int j = 0; j < numMotions; j++)
        {
            genCandidates(ofs, i, j, k);
        }
    }

    ofs.close();
}

void MotionGraph::addEdge(int id, int src, int dst, double theta, double x0, double z0, int length, int rest)
{
    Edge edge;
    edge.id = id;
    edge.src = src;
    edge.dst = dst;
    edge.theta = theta;
    edge.x0 = x0;
    edge.z0 = z0;
    edge.length = length;
    edge.rest = rest;
    edges.push_back(edge);
}

double MotionGraph::distance(int ma, int mb, int i, int j, double *theta, double *x0, double *z0)
{
    int k = numBlendingFrames;

    Skeleton *skeleton = library->getSkeleton();
    Motion *motionA = library->getMotion(ma);
    Motion *motionB = library->getMotion(mb);

    if (skeleton == NULL || motionA == NULL || motionB == NULL)
    {
        printf("Error: in distance().\n");
        exit(-1);
    }

    PointCloud **pcListA = new PointCloud *[k];
    PointCloud **pcListB = new PointCloud *[k];
    for (int f = i, ii = 0; f < i + k; f++, ii++)
    {
        skeleton->setPosture(*(motionA->GetPosture(f)));
        pcListA[ii] = new PointCloud(skeleton);
    }
    for (int f = j - k + 1, ii = 0; f < j + 1; f++, ii++)
    {
        skeleton->setPosture(*(motionB->GetPosture(f)));
        pcListB[ii] = new PointCloud(skeleton);
    }

    PointCloud pcA(&pcListA[0], k);
    PointCloud pcB(&pcListB[0], k);
    double dist = distance(&pcA, &pcB, theta, x0, z0);

    for (int i = 0; i < k; i++)
        delete pcListA[i];
    for (int i = 0; i < k; i++)
        delete pcListB[i];
    delete [] pcListA;
    delete [] pcListB;

    return dist;
}

void MotionGraph::genGraph()
{
    int k = numBlendingFrames;
    int restPoseIdx = 0;                                            // rest pose motion idx
    restLabel = 0;

    int numMotions = library->getNumMotions();

    std::vector< std::vector<int> > nodeId;            // nodeId[motionIdx][frameIdx]
    for (int m = 0; m < numMotions; m++)
    {
        nodeId.push_back(std::vector<int>());
        for (int f = 0; f < library->getMotion(m)->GetNumFrames(); f++)
            nodeId[m].push_back(-1);                                        // -1 := not a node
        nodeId[m][0] = 1;                                                   // first frame
        //nodeId[m][nodeId[m].size() - 1] = 1;                                // last frame

        // transition point to the rest pose
        int f = nodeId[m].size() - k;
        if (f >= 0)
            nodeId[m][f] = 1;
    }
    // rest pose
    if ((int)nodeId[restPoseIdx].size() >= k)
        nodeId[restPoseIdx][k - 1] = 1;

    std::vector< std::vector< std::vector<WindowPair> > > candidates;       // [motionIdxA][motionIdxB][pairIdx]
    for (int ma = 0; ma < numMotions; ma++)
    {
        candidates.push_back(std::vector< std::vector<WindowPair> >());
        for (int mb = 0; mb < numMotions; mb++)
        {
            candidates[ma].push_back(std::vector<WindowPair>());
        }
    }

    // load candidates
    std::ifstream ifs;
    ifs.open("./mocap_data/candidates.txt", std::ios::in);
    char buf[256];
    ifs >> buf;                             // #numMotions
    int nmotions;
    ifs >> nmotions;
    for (int ma = 0; ma < nmotions; ma++) for (int mb = 0; mb < nmotions; mb++)
    {
        ifs >> buf;                         // #motionIndex
        int motionIdxA, motionIdxB;
        ifs >> motionIdxA >> motionIdxB;
        ifs >> buf;                         // #motionName
        ifs >> buf;                         // motionNameA
        ifs >> buf;                         // motionNameB
        ifs >> buf;                         // #numFrames
        int numFramesA, numFramesB;
        ifs >> numFramesA >> numFramesB;
        ifs >> buf;                         // #minListSize
        int minListSize;
        ifs >> minListSize;
        ifs >> buf;                         // #i
        ifs.getline(buf, sizeof(buf));      // j distance theta x0 z0
        for (int ii = 0; ii < minListSize; ii++)
        {
            int i, j;
            double dist;
            double theta, x0, z0;
            ifs >> i >> j >> dist >> theta >> x0 >> z0;

            if (j > (int)nodeId[mb].size() - k)     // later than last transition point
                continue;

            // todo: check threshold

            WindowPair pair;
            pair.i = i;
            pair.j = j;
            pair.dist = dist;
            pair.theta = theta;
            pair.x0 = x0;
            pair.z0 = z0;
            candidates[ma][mb].push_back(pair);

            nodeId[ma][i] = 1;
            nodeId[mb][j] = 1;
        }
    }
    ifs.close();

    /*for (int aa = 0; aa < numMotions; aa++)
        for (int bb = 0; bb < numMotions; bb++)
        {
            printf("motion(%d, %d): %d\n", aa, bb, candidates[aa][bb].size());
            for (size_t ii = 0; ii < candidates[aa][bb].size(); ii++)
            {
                WindowPair &p = candidates[aa][bb][ii];
                printf("%d %d %lf %lf %lf %lf\n", p.i, p.j, p.dist, p.theta, p.x0, p.z0);
            }
        }
    */

    int numNodes = 0;
    int numEdges = 0;

    // construct nodes and edges within motions
    for (int m = 0; m < numMotions; m++)
    {
        int lastNode = -1;
        for (int f = 0; f < (int)nodeId[m].size(); f++)
            if (nodeId[m][f] >= 0)
            {
                // form a new node
                nodeId[m][f] = numNodes++;

                printf("node(%d): %d, %d\n", nodeId[m][f], m, f);

                Node node;
                node.id = nodeId[m][f];
                node.motionIdx = m;
                node.frameIdx = f;
                node.label = m;                 // use motion idx as label
                nodes.push_back(node);

                if (lastNode != -1)             // not the first node of the motion
                {
                    // construct an edge fram last node to current node
                    int id = numEdges++;
                    int src = nodeId[m][lastNode];
                    int dst = nodeId[m][f];
                    addEdge(id, src, dst, 0.0, 0.0, 0.0, f - lastNode);
                    nodes[src].link.push_back(id);
                }
                lastNode = f;
            }
    }
        
    // construct edges crossing two motions
    for (int ma = 0; ma < numMotions; ma++)
    {
        for (int mb = 0; mb < numMotions; mb++)
        {
            for (size_t ii = 0; ii < candidates[ma][mb].size(); ii++)
            {
                WindowPair &pair = candidates[ma][mb][ii];

                // construct an edge from A(i) to B(j)
                int id = numEdges++;
                int src = nodeId[ma][pair.i];
                int dst = nodeId[mb][pair.j];
                addEdge(id, src, dst, pair.theta, pair.x0, pair.z0, k - 1);
                nodes[src].link.push_back(id);
            }
        }
    }

    // construct edges from the end of motions to the rest pose
    if ((int)nodeId[restPoseIdx].size() >= k)
    {
        for (int m = 0; m < numMotions; m++)
        {
            double theta, x0, z0;
            distance(m, restPoseIdx, nodeId[m].size() - k, k - 1, &theta, &x0, &z0);
            
            int id = numEdges++;
            int src = nodeId[m][nodeId[m].size() - k];
            int dst = nodeId[restPoseIdx][k - 1];

            addEdge(id, src, dst, theta, x0, z0, k - 1, 1);
            nodes[src].link.push_back(id);
        }
    }

    // test: dump
    for (size_t eid = 0; eid < edges.size(); eid++)
    {
        Edge &e = edges[eid];
        int srcMotion = nodes[e.src].motionIdx;
        int dstMotion = nodes[e.dst].motionIdx;
        int srcFrame = nodes[e.src].frameIdx;
        int dstFrame = nodes[e.dst].frameIdx;
        printf("edge(%d): %d(%d) --> %d(%d)\n", eid, srcMotion, srcFrame, dstMotion, dstFrame);
    }

    ////////////////////////////////////////////////////////////////////////////

    int numLabels = numMotions;         // can be different

    for (int dstLabel = 0; dstLabel < numLabels; dstLabel++)
    {
        // find shortest path to a certain label
        std::vector<int> dist;
        std::vector<int> next;
        for (size_t i = 0; i < nodes.size(); i++)
        {
            int initDist = 999999999;
            if (nodes[i].label == dstLabel)
                initDist = 0;
            dist.push_back(initDist);
            next.push_back(-1);
        }

        // Bellman-Ford
        for (int i = 0; i < (int)nodes.size() - 1; i++)
        {
            for (int j = 0; j < (int)edges.size(); j++)
            {
                int src = edges[j].src;
                int dst = edges[j].dst;
                if (dist[src] > dist[dst] + edges[j].length)
                {
                    dist[src] = dist[dst] + edges[j].length;
                    next[src] = j;
                }
            }
        }

        for (int i = 0; i < (int)nodes.size(); i++)
        {
            int nextEdge = next[i];

            if (nodes[i].label == dstLabel)
            {
                if (nodes[i].link.size() == 0)
                {
                    printf("Warning: nodes[%d](%d, %d).link.size() == 0\n", i, nodes[i].motionIdx, nodes[i].frameIdx);
                    exit(-1);
                }
                else
                {
                    int ndst0 = edges[nodes[i].link[0]].dst; 
                    if (nodes[ndst0].motionIdx == nodes[i].motionIdx && nodes[ndst0].frameIdx > nodes[i].frameIdx)
                    {
                        // set the next edge to the edge towards the next frame
                        nextEdge = nodes[i].link[0];
                    }
                    else
                    {
                        //printf("%d(%d)\n", nodes[i].motionIdx, nodes[i].frameIdx);
                        nextEdge = nodes[i].link[0];
                        /*for (int j = 0; j < (int)nodes[i].link.size(); j++)
                        {
                            if (edges[nodes[i].link[j]].rest != 0)
                            {
                                nextEdge = nodes[i].link[j];
                                break;
                            }
                        }*/
                        //printf("%d(%d) --> %d(%d)\n", nodes[edges[nextEdge].src].motionIdx, nodes[edges[nextEdge].src].frameIdx,
                        //                              nodes[edges[nextEdge].dst].motionIdx, nodes[edges[nextEdge].dst].frameIdx);
                    }
                    /*for (int j = 0; j < (int)nodes[i].link.size(); j++)
                    {
                        int dst = edges[nodes[i].link[j]].dst;
                        if (nodes[dst].motionIdx == nodes[i].motionIdx &&
                            nodes[dst].frameIdx == nodes[i].frameIdx + 1)
                        {
                            nextEdge = nodes[i].link[j];
                            break;
                        }
                    }*/
                }
            }

            nodes[i].next.push_back(nextEdge);

            if (nextEdge == -1)
            {
                printf("Warning: node[%d](%d, %d) cannot reach label %d.\n", i, nodes[i].motionIdx, nodes[i].frameIdx, dstLabel);
                //exit(-1);
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////

    // construct edge clips
    for (size_t eid = 0; eid < edges.size(); eid++)
    {
        Motion *clip = NULL;

        Edge &e = edges[eid];
        Node &nsrc = nodes[e.src];
        Node &ndst = nodes[e.dst];

        if (nsrc.motionIdx != ndst.motionIdx || e.length != ndst.frameIdx - nsrc.frameIdx)      // transition between two motions
        {
            //printf("src(%d, %d) dst(%d, %d) %d\n", nsrc.motionIdx, nsrc.frameIdx, ndst.motionIdx, ndst.frameIdx, e.length);
            Transition *tran = new Transition(library->getMotion(nsrc.motionIdx), nsrc.frameIdx, library->getMotion(ndst.motionIdx), ndst.frameIdx, e.theta, e.x0, e.z0);
            clip = tran->getBlendedMotion();
            delete tran;
        }
        else                                                                                    // forward edge
        {
            Motion *motion = library->getMotion(nsrc.motionIdx);
            clip = new Motion(e.length + 1, library->getSkeleton());
            if (e.length != ndst.frameIdx - nsrc.frameIdx)
            {
                printf("Error: edges[%d].length != nodes[%d].frameIdx - nodes[%d].frameIdx\n", eid, e.dst, e.src);
            }

            for (int i = 0, j = nsrc.frameIdx; i < e.length + 1; i++, j++)
            {
                clip->SetPosture(i, *motion->GetPosture(j));
            }
        }

        clips.push_back(clip);
    }
        
    reset();
}

void MotionGraph::reset()
{
    targetLabel = restLabel;
    int nodeId = 0;
    currentEdge = nodes[nodeId].next[targetLabel];
    currentFrame = 0;
    resetTransformMatrix();
}

void MotionGraph::advance()
{
    currentFrame++;
    if (currentFrame >= clips[currentEdge]->GetNumFrames() - 1)     // is the last frame
    {
        double matrix[16];
        getTransformMatrix(matrix, edges[currentEdge].theta, edges[currentEdge].x0, edges[currentEdge].z0);
        glPushMatrix();
        glLoadIdentity();
        glMultMatrixd(transformMatrix);
        glMultMatrixd(matrix);
        //glMultMatrixd(transformMatrix);
        glGetDoublev(GL_MODELVIEW_MATRIX, transformMatrix);
        glPopMatrix();

        if (edges[currentEdge].rest != 0 && nodes[edges[currentEdge].src].label == targetLabel)
            targetLabel = restLabel;

        int nodeId = edges[currentEdge].dst;
        currentEdge = nodes[nodeId].next[targetLabel];
        currentFrame = 0;

        printf("edge(%d): %d(%d) --> %d(%d)\n", currentEdge, nodes[edges[currentEdge].src].motionIdx, nodes[edges[currentEdge].src].frameIdx,
                                                             nodes[edges[currentEdge].dst].motionIdx, nodes[edges[currentEdge].dst].frameIdx);
    }
}

void MotionGraph::setTargetLabel(int label)
{
    targetLabel = label;
}

Motion *MotionGraph::getMotion()
{
    return clips[currentEdge];
}

Posture *MotionGraph::getPosture()
{
    return clips[currentEdge]->GetPosture(currentFrame);
}

void MotionGraph::getTransformMatrix(double matrix[])
{
    for (int i = 0; i < 16; i++)
        matrix[i] = transformMatrix[i];
}

void MotionGraph::resetTransformMatrix()
{
    glPushMatrix();
    glLoadIdentity();
    glGetDoublev(GL_MODELVIEW_MATRIX, transformMatrix);
    glPopMatrix();
}
