//
//  MotionLibrary.cpp
//  mocap
//
//  Created by javid on 6/6/13.
//  Copyright (c) 2013 javid. All rights reserved.
//
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <math.h>
#include <stdlib.h>

#include "MotionLibrary.h"

MotionLibrary::MotionLibrary(char* name, Skeleton* pSkeleton)
    :skeleton(pSkeleton), total_motion_num(0)
{
    int code = this->loadFiles(name);
    if( code == -1)
        printf("load motion files failed");
}

int MotionLibrary::loadFiles(char* name){
    std::ifstream file( name, std::ios::in );
    if( file.fail() )
        return -1;
    
    int counter = 0;
    char str[2048];
    while(!file.eof()){

        file.getline(str, 2048);
        if(file.eof()) break;
        
        std::string filename = PREFIX_MOTION_PATH + std::string(str) + ".amc";
        
        //load motion file
        Motion* pMotion = new Motion( strdup(filename.c_str()) , MOCAP_SCALE, this->skeleton);
        this->table[counter] = name;
        this->container[counter] = pMotion;
        counter++;        
    }
    this->total_motion_num = counter;
    return 0;
}

Motion* MotionLibrary::getMotion(char* name){
    int i = 0;
    while(this->table[i] != name && i++ < total_motion_num );
    if( i < total_motion_num)
        return this->container[i];//return motion
    else{
        printf("ERROR: can't find the motion data: %s", name);
        return 0;
    }
}