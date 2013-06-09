//
//  Floyd.h
//  mocap
//
//  Created by Javid on 6/7/13.
//  Copyright (c) 2013 javid. All rights reserved.
//

#ifndef mocap_Floyd_h
#define mocap_Floyd_h

#define GRAPH_MAX 5000

class Floyd{

public:
    Floyd(char* filename);
    int* getShortestPath(int node1, int node2);    
    
protected:
    int parseFile(char* filename);
    void solve();
    
private:
    //graph
    int node_num;
    double graph[GRAPH_MAX][GRAPH_MAX];
    
};

#endif
