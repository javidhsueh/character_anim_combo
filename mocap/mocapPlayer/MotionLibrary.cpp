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
#include <string>
#include "Transition.h"

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

Motion* MotionLibrary::getMotion(int index){
    if( index < total_motion_num)
        return this->container[index];//return motion
    else{
        printf("ERROR: can't find the motion data: %d", index);
        return 0;
    }
}

Motion* MotionLibrary::createTransition(int idx1, int f1, int idx2, int f2){
    
    if(idx1 > total_motion_num || idx2 > total_motion_num ){
        printf("Index out of bound.");
        return 0;
    }
    Motion* m1 = this->container[idx1];
    Motion* m2 = this->container[idx2];
    if(f1 > m1->GetNumFrames() || f2 > m2->GetNumFrames() ){
        printf("Frame index out of bound.");
        return 0;
    }
    Transition* t =  new Transition(m1, f1, m2, f2);
    return t->getBlendedMotion();
    
}
