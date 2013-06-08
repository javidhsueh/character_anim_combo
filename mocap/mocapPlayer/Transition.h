//
//  Transition.h
//  mocap
//
//  Created by Javid on 6/6/13.
//  Copyright (c) 2013 javid. All rights reserved.
//

#ifndef mocap_Transition_h
#define mocap_Transition_h

#define BLENDING_FRAME_NUM 50

#include "motion.h"
#include "posture.h"

class Transition{
public:
    
    Transition(Motion* m1, int f1, Motion* m2, int f2);

    Motion* getBlendedMotion();
    
    int writeMotionFile(char* filename);
    
protected:
    void blend();
private:
    
    Motion* m1;
    Motion* m2;
    
    int f1, f2;
    
    Posture* interpolated_pos;
};

#endif
