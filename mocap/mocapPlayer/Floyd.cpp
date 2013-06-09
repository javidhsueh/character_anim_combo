//
//  Floyd.cpp
//  mocap
//
//  Created by Javid on 6/7/13.
//  Copyright (c) 2013 javid. All rights reserved.
//

#include "Floyd.h"
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <string>

Floyd::Floyd(char* filename){

    //load graph
    int code = this->parseFile(filename);
    if(code == -1)
        printf("Enable to read graph file");
    else
        this->solve();//solve graph    
}

int Floyd::parseFile(char* filename){

    std::ifstream file( filename, std::ios::in );
    if( file.fail() )
        return -1;
    
    int counter = 0;
    char str[2048];
    while(!file.eof()){
        
        file.getline(str, 2048);
        if(file.eof()) break;
        
        
    }
    return 0;
}

void Floyd::solve(){
    
}

int* Floyd::getShortestPath(int node1, int node2){
    int* path = nullptr;
    
    return path;
}

