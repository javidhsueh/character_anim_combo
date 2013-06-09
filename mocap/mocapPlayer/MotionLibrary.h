//
//  MotionLibrary.h
//  mocap
//
//  Created by javid on 6/6/13.
//  Copyright (c) 2013 javid. All rights reserved.
//

#ifndef mocap_MotionLibrary_h
#define mocap_MotionLibrary_h

#define MAX_MOTION_NUM 9999
#define PREFIX_MOTION_PATH "./mocap_data/"


#include "motion.h"
#include "skeleton.h"


class MotionLibrary{
    
public:
    MotionLibrary(char* table,  Skeleton* pSkeleton);

    
    //getter
    Motion* getMotion(char *name);
    
    Motion* getMotion(int index);

    Skeleton *getSkeleton() { return skeleton; }
    
    //method
    Motion* createTransition(int idx1, int f1, int idx2, int f2, double theta, double tx, double tz);
    
private:
    int total_motion_num;
    //loader
    int loadFiles(char *table);
    
    //table
    char* table[MAX_MOTION_NUM];
 
    //container
    Motion* container[ MAX_MOTION_NUM ];
    
    //skeleton
    Skeleton* skeleton;
};

#endif
