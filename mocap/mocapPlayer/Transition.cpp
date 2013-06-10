//
//  Transition.cpp
//  mocap
//
//  Created by Javid on 6/6/13.
//  Copyright (c) 2013 javid. All rights reserved.
//

#include "Transition.h"
#include <math.h>
#include "vector.h"
#include <stdlib.h>
#include <stdio.h>
#include "transform.h"
#include <FL/gl.h>


inline double radToDegree(double theta)
{
    return theta * 180.0 / M_PI;
}

// rx := psi
// ry := theta
// rz := phi
void matrixToEulerXYZ(const double m[], double *rx, double *ry, double *rz)
{
    if (m[2] != 1.0 && m[2] != -1.0)
    {
        *ry = -asin(m[2]);
        double cosTheta = cos(*ry);
        *rx = atan2(m[6] / cosTheta, m[10] / cosTheta);
        *rz = atan2(m[1] / cosTheta, m[0] / cosTheta);
    }
    else    // Gimbal lock ?
    {
        *rz = 0.0;
        if (m[2] == -1.0)
        {
            *ry = M_PI / 2.0;
            *rx = *rz + atan2(m[4], m[8]);
        }
        else
        {
            *ry = -M_PI / 2.0;
            *rx = -(*rz) + atan2(-m[4], -m[8]);
        }
    }

    *rx = radToDegree(*rx);
    *ry = radToDegree(*ry);
    *rz = radToDegree(*rz);
}

vector Slerp(vector start, vector end, double percent)
{
    // Dot product - the cosine of the angle between 2 vectors.
    double dot = start % end;
    // Clamp it to be in the range of Acos()
    dot = dot > 1.0 ? 1.0 : dot;
    dot = dot < -1.0? -1.0 : dot;
    // Acos(dot) returns the angle between start and end,
    // And multiplying that by percent returns the angle between
    // start and the final result.
    double theta = acos(dot)*percent;
    vector RelativeVec = end - start*dot;
    RelativeVec.normalize();
    // The final result.
    return ((start*cos(theta)) + (RelativeVec*sin(theta)));
}

vector Quarternion2Euler(vector q)
{
    vector euler;
    double w = q[0];
    double x = q[1];
    double y = q[2];
    double z = q[3];
    
    double sqw = w * w;
	double sqx = x * x;
	double sqy = y * y;
	double sqz = z * z;
    
    /*
    euler[0] = atan2(2*((w * x) + (y * z)), 1 - (2 * ((x* x) + (y * y)))) * (180.0f/M_PI);
    euler[1] = asin(2 * ((w * y) - (z * x))) * (180.0f/M_PI);
    euler[2] = atan2(2 * ((w * z) + (x * y)), 1 - (2 * ((y * y) + (z * z)))) * (180.0f/M_PI);    
    euler[3] = 0;
    */
    
    /*
	euler[0] = (double)atan2l(2.0 * ( y*z + x*w ) , ( -sqx - sqy + sqz + sqw )) * (180.0f/M_PI);
	euler[1] = (double)asinl(-2.0 * ( x*z - y*w )) * (180.0f/M_PI);
	euler[2] = (double)atan2l(2.0 * ( x*y + z*w ) , (  sqx - sqy - sqz + sqw )) * (180.0f/M_PI);
    euler[3] = 0;
	*/
    
    
    euler[0] = (double)atan2( (2*(x*w-y*z)), 1-2*sqx-2*sqy) * (180.0f/M_PI);
    euler[1] = (double)asin(2*(x*z+y*w)) * (180.0f/M_PI);
    euler[2] = (double)atan2( ( 2*(z*w-x*y)), 1-2*sqy-2*sqz ) * (180.0f/M_PI);
    euler[3] = 0;
        
    return euler;
}

vector Euler2Quarternion(vector d)
{
    vector quart;
    /*
    double e1 = d[0];//x
    double e2 = d[1];//y
    double e3 = d[2];//z
    double PI = M_PI;
    
    quart[0] = sqrt(cos(e2*PI/180)*cos(e1*PI/180)+cos(e2*PI/180)*cos(e3*PI/180)-sin(e2*PI/180)*sin(e1*PI/180)*sin(e3*PI/180)+cos(e1*PI/180)* cos(e3*PI/180)+1)/2;
    
    quart[1] = (cos(e1*PI/180)*sin(e3*PI/180)+cos(e2*PI/180)*sin(e3*PI/180)+sin(e2*PI/180)*sin(e1*PI/180)*cos(e3*PI/180))/sqrt(cos(e2*PI/180)* cos(e1*PI/180)+cos(e2*PI/180)*cos(e3*PI/180)-sin(e2*PI/180)*sin(e1*PI/180)*sin(e3*PI/180)+cos(e1*PI/180)*cos(e3*PI/180)+1)/2;
    
    quart[2] = (sin(e2*PI/180)*sin(e3*PI/180)-cos(e2*PI/180)*sin(e1*PI/180)*cos(e3*PI/180)-sin(e1*PI/180))/sqrt(cos(e2*PI/180)*cos(e1*PI/180)+ cos(e2*PI/180)*cos(e3*PI/180)-sin(e2*PI/180)*sin(e1*PI/180)*sin(e3*PI/180)+cos(e1*PI/180)*cos(e3*PI/180)+1)/2;
    
    quart[3] = (sin(e2*PI/180)*cos(e1*PI/180)+sin(e2*PI/180)*cos(e3*PI/180)+cos(e2*PI/180)*sin(e1*PI/180)*sin(e3*PI/180))/sqrt(cos(e2*PI/180)* cos(e1*PI/180)+cos(e2*PI/180)*cos(e3*PI/180)-sin(e2*PI/180)*sin(e1*PI/180)*sin(e3*PI/180)+cos(e1*PI/180)*cos(e3*PI/180)+1)/2;
    */
    //double examine = quart[0]*quart[0] + quart[1]*quart[1] + quart[2]*quart[2] + quart[3]*quart[3];
  /*
    double PIOVER180 = M_PI/180;
    float p = d.z() * PIOVER180 / 2.0;
	float y = d.y() * PIOVER180 / 2.0;
	float r = d.x() * PIOVER180 / 2.0;
    
	float sinp = sin(p);
	float siny = sin(y);
	float sinr = sin(r);
	float cosp = cos(p);
	float cosy = cos(y);
	float cosr = cos(r);
    
	quart[0] = sinr * cosp * cosy - cosr * sinp * siny;
	quart[1] = cosr * sinp * cosy + sinr * cosp * siny;
	quart[2] = cosr * cosp * siny - sinr * sinp * cosy;
	quart[3] = cosr * cosp * cosy + sinr * sinp * siny;
    quart.normalize();
   */
    vector q_x;// sin(angx/2) 0 0 cos(angx/2)
    q_x[0] = sin(d.x()/2);
    q_x[1] = q_x[2] = 0;
    q_x[3] = cos(d.x()/2);
    
    vector q_y; //  0 sin(angy/2) 0 cos(angy/2)
    q_y[0] = q_y[2] = 0;
    q_y[1] = sin(d.y()/2);
    q_y[3] = cos(d.y()/2);
    
    vector q_z;  // 0 0 sin(angz/2) cos(angz/2)
    q_z[0] = q_z[1] = 0;
    q_z[2] = sin(d.z()/2);
    q_z[3] = cos(d.z()/2);
    
    vector qe;
    qe[0] = q_x[0] * q_y[0] - q_x[1] * q_y[1] - q_x[2] * q_y[2] - q_x[3] * q_y[3];
    qe[1] = q_x[0] * q_y[1] + q_x[1] * q_y[0] + q_x[2] * q_y[3] - q_x[3] * q_y[2];
    qe[2] = q_x[0] * q_y[2] - q_x[1] * q_y[3] + q_x[2] * q_y[0] + q_x[3] * q_y[1];
    qe[3] = q_x[0] * q_y[3] + q_x[1] * q_y[2] - q_x[2] * q_y[1] + q_x[3] * q_y[0];
    
    vector qe2;
    qe2[0] = qe[0] * q_z[0] - qe[1] * q_z[1] - qe[2] * q_z[2] - qe[3] * q_z[3];
    qe2[1] = qe[0] * q_z[1] + qe[1] * q_z[0] + qe[2] * q_z[3] - qe[3] * q_z[2];
    qe2[2] = qe[0] * q_z[2] - qe[1] * q_z[3] + qe[2] * q_z[0] + qe[3] * q_z[1];
    qe2[3] = qe[0] * q_z[3] + qe[1] * q_z[2] - qe[2] * q_z[1] + qe[3] * q_z[0];
    
    //double examine = qe2[0]*qe2[0] + qe2[1]*qe2[1] + qe2[2]*qe2[2] + qe2[3]*qe2[3];
    return qe2;
}


Transition::Transition(Motion* m1, int f1, Motion* m2, int f2, double theta, double tx, double tz)
:m1(m1), m2(m2), f1(f1), f2(f2), theta(theta), tx(tx), tz(tz)
{
    if(f1 + BLENDING_FRAME_NUM > m1->GetNumFrames() || f2-BLENDING_FRAME_NUM+1 < 0)
        printf("Error: the frame index out of bound.");
    else{
        this->interpolated_pos = new Posture[BLENDING_FRAME_NUM];
        this->blend();
    }    
}

int Transition::writeMotionFile(char* filename)
{
    return this->getBlendedMotion()->writeAMCfile(filename, 1.0);
}

void Transition::blend(){
    
    int num_bones = m1->GetSkeleton()->getNumBonesInAsf();
    
    for(int i= 0 ; i < BLENDING_FRAME_NUM; i++){
        
        double t = (1.0*i + 1 ) / BLENDING_FRAME_NUM;
        double alpha = 2*t*t*t - 3*t*t + 1;
        
        
        Posture* p1 = m1->GetPosture(f1+i);
        Posture* p2 = m2->GetPosture(f2-BLENDING_FRAME_NUM+1+i);
        
        //get root position
        vector root1 = p1->root_pos;
        vector root2 = p2->root_pos;
        vector_rotationXYZ(root2.p, 0.0, radToDegree(theta), 0.0);
        root2 = root2 + vector(tx, 0.0, tz);
        
        //cancel x,z translation
        //root1[0] = root1[2] = root2[0] = root2[2] = 0;
        
        //interpolate root translation
        //vector inerpolate_root =  (root2-root1)*(1.0*i/BLENDING_FRAME_NUM) + root1;
        vector inerpolate_root =  root1*alpha + root2*(1.0-alpha);
        this->interpolated_pos[i].root_pos = inerpolate_root;
        this->interpolated_pos[i].bone_translation[0] = inerpolate_root;
        
        //calculate root rotation
        vector bone1_rotation = p1->bone_rotation[0];
        vector bone2_rotation = p2->bone_rotation[0] ; //* theta
        
        glPushMatrix();
        glLoadIdentity();
        glRotated(radToDegree(theta), 0.0, 1.0, 0.0);
        glRotated(bone2_rotation.z(), 0.0, 0.0, 1.0);
        glRotated(bone2_rotation.y(), 0.0, 1.0, 0.0);
        glRotated(bone2_rotation.x(), 1.0, 0.0, 0.0);
        double matrix[16];
        glGetDoublev(GL_MODELVIEW_MATRIX, matrix);
        double rx, ry, rz;
        matrixToEulerXYZ(matrix, &rx, &ry, &rz);
        bone2_rotation = vector(rx, ry, rz);
        glPopMatrix();

        this->interpolated_pos[i].bone_rotation[0] = bone1_rotation*alpha + bone2_rotation* (1.0-alpha);
        
        //get joint rotation
        for(int j = 1; j < num_bones; j++){
            
            vector bone1_rotation = p1->bone_rotation[j];
            vector bone2_rotation = p2->bone_rotation[j];
            
            ///////////use quarternion/////////////////////////
            //convert to quarternion q1, q2
//            vector q1 = Euler2Quarternion(bone1_rotation);
//            vector q2 = Euler2Quarternion(bone2_rotation);
            
//            vector test = Euler2Quarternion(bone1_rotation);
//            test = Quarternion2Euler(test);
            //interpolate q1, q2 => q
            //vector q = (q2-q1)*(1.0*i/BLENDING_FRAME_NUM) + q1; //linear interpolation
            //vector q = q1*alpha + q2*(1.0-alpha);
            
//            vector q = Slerp(q1, q2, alpha);

            //convert q =>euler
//            vector interpolate_rotation = Quarternion2Euler(q);
            ///////////use quarternion/////////////////////////
            
            
            ///////////use linear interpolation /////////////////////////
            //vector interpolate_rotation = (bone2_rotation-bone1_rotation)*(1.0*i/BLENDING_FRAME_NUM) + bone1_rotation;
            
            ///////////use weighted interpolation /////////////////////////
            //vector interpolate_rotation = Slerp(bone1_rotation, bone2_rotation, alpha);
            vector interpolate_rotation = bone1_rotation*alpha + bone2_rotation* (1.0-alpha);
            
            this->interpolated_pos[i].bone_rotation[j] = interpolate_rotation;
        }
        
    }    
}

Motion* Transition::getBlendedMotion(){

    Motion* m = new Motion(BLENDING_FRAME_NUM, this->m1->GetSkeleton() );
    for(int i = 0 ; i< BLENDING_FRAME_NUM; i++){
        m->SetPosture(i, this->interpolated_pos[i]);
    }
    
    /*
    //concatenate two motions and transition
    Motion* m = new Motion(f1+BLENDING_FRAME_NUM+(m2->GetNumFrames()-f2), this->m1->GetSkeleton() );
    int counter = 0;
    for(int i = 0 ; i< f1; i++){
        m->SetPosture(counter, *this->m1->GetPosture(i));
        counter++;
    }
    for(int i = 0 ; i< BLENDING_FRAME_NUM; i++){
        m->SetPosture(counter, this->interpolated_pos[i]);
        counter++;
    }
    for(int i = f2 ; i< m2->GetNumFrames(); i++){
        m->SetPosture(counter, *this->m2->GetPosture(i));
        counter++;
    }*/
    return m;
    
}