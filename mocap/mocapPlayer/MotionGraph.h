#ifndef MOTIONGRAPH_H
#define MOTIONGRAPH_H

#include <iostream>
#include <vector>
#include "MotionLibrary.h"
#include "PointCloud.h"

struct Node
{
    Node() : id(0), motionIdx(0), frameIdx(0), label(0), link(), next() {}
    int id;
    int motionIdx;
    int frameIdx;
    int label;
    std::vector<int> link;      // edge ids
    std::vector<int> next;      // next edge in the shortest path to a certain label
};

struct Edge
{
    int id;
    int src;
    int dst;
    double theta;
    double x0, z0;
    int length;
    int rest;       // 1 if this edge is connecting to the beginning of rest pose
};

class MotionGraph
{
public:
    MotionGraph();
    MotionGraph(MotionLibrary *lib);
    ~MotionGraph();

    int getNumNodes() const { return (int)nodes.size(); }
    int getNumEdges() const { return (int)edges.size(); }

    void genCandidates(std::ofstream &os, int motionIdx1, int motionIdx2, int k);
    double distance(PointCloud *pcA, PointCloud *pcB, double *matrix = NULL);
    double distance(PointCloud *pcA, PointCloud *pcB, double *oTheta, double *oX0, double *oZ0);
    //double distance(PointCloud *pcA, PointCloud *pcB);
    void genAllCandidates(int k);                           // k should be 40

    void genGraph();

    // traversal
    void reset();
    void advance();
    void setTargetLabel(int label);
    int getTargetLabel() const { return targetLabel; }
    int getCurrentLabel();
    Motion *getMotion();
    Posture *getPosture();
    void getTransformMatrix(double matrix[]);
    void resetTransformMatrix();

private:
    void getTransformMatrix(double *matrix, double theta, double x0, double z0);
    void addEdge(int id, int src, int dst, double theta, double x0, double z0, int length, int rest = 0);
    double distance(int ma, int mb, int i, int j, double *theta, double *x0, double *z0);

private:
    MotionLibrary *library;

    int numBlendingFrames;

    std::vector<Node> nodes;
    std::vector<Edge> edges;

    std::vector<Motion *> clips;

    int restLabel;
    int targetLabel;
    int currentEdge;
    int currentFrame;

    double transformMatrix[16];
};


#endif // MOTIONGRAPH_H
