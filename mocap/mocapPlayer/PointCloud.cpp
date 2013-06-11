#include <cmath>

#include <FL/gl.h>
#include <FL/glu.h>
#include <FL/glut.H>

#include "PointCloud.h"
#include "MotionGraph.h"
#include "transform.h"


extern MotionGraph *motion_graph;
extern bool useMotionGraph;


void copyMatrix(double dst[], const double src[])
{
    for (int i = 0; i < 16; i++)
        dst[i] = src[i];
}

// m is column major (OpenGL style)
vector matMultVec3(const double m[], const vector &v)
{
    double x = m[0] * v.x() + m[4] * v.y() + m[ 8] * v.z() + m[12] * 1.0;
    double y = m[1] * v.x() + m[5] * v.y() + m[ 9] * v.z() + m[13] * 1.0;
    double z = m[2] * v.x() + m[6] * v.y() + m[10] * v.z() + m[14] * 1.0;
    double w = m[3] * v.x() + m[7] * v.y() + m[11] * v.z() + m[15] * 1.0;
    return vector(x / w, y / w, z / w);
}

inline double radToDegree(double theta)
{
    return theta * 180.0 / M_PI;
}

// rx := psi
// ry := theta
// rz := phi
void matrixToEulerXYZ_(const double m[], double *rx, double *ry, double *rz)
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

// generate a point cloud according to the skeleton
// skeleton's posture must be set
PointCloud::PointCloud(Skeleton *skl)
{
    // points
    numPoints = skl->getNumBonesInAsf() * 6;
    points = new vector[numPoints];
    numPoints = 0;

    glPushMatrix();
    glLoadIdentity();

    double translation[3];
    skl->GetTranslation(translation);
    double rotationAngle[3];
    skl->GetRotationAngle(rotationAngle);

    glTranslatef(float(MOCAP_SCALE * translation[0]), float(MOCAP_SCALE * translation[1]), float(MOCAP_SCALE * translation[2]));
    glRotatef(float(rotationAngle[0]), 1.0f, 0.0f, 0.0f);
    glRotatef(float(rotationAngle[1]), 0.0f, 1.0f, 0.0f);
    glRotatef(float(rotationAngle[2]), 0.0f, 0.0f, 1.0f);
    traverse(skl->getRoot());

    glPopMatrix();

    // default transform matrix
    glPushMatrix();
    glLoadIdentity();
    glGetDoublev(GL_MODELVIEW_MATRIX, transformMatrix);
    glPopMatrix();
}

PointCloud::PointCloud(PointCloud **pc, int k)
{
    numPoints = 0;
    for (int i = 0; i < k; i++)
        numPoints += pc[i]->getNumPoints();
    
    points = new vector[numPoints];
    
    numPoints = 0;
    for (int i = 0; i < k; i++)
    {
        for (int j = 0; j < pc[i]->getNumPoints(); j++)
        {
            points[numPoints++] = pc[i]->getPoints()[j];
        }
    }

    // default transform matrix
    glPushMatrix();
    glLoadIdentity();
    glGetDoublev(GL_MODELVIEW_MATRIX, transformMatrix);
    glPopMatrix();
}

PointCloud::~PointCloud()
{
    delete [] points;
}

void PointCloud::setTransformMatrix(double matrix[])
{
    for (int i = 0; i < 16; i++)
        transformMatrix[i] = matrix[i];
}

void PointCloud::draw()
{
    float color[3] = {1.0f, 0.0f, 0.0f};
    float color2[3] = {0.0f, 1.0f, 0.0f};
    float shininess = 120.0f;
    float ambient[4] = {color[0], color[1], color[2], 1.0};
    float diffuse[4] = {color[0], color[1], color[2], 1.0};
    float specular[4] = {color[0], color[1], color[2], 1.0};
    float ambient2[4] = {color2[0], color2[1], color2[2], 1.0};
    float diffuse2[4] = {color2[0], color2[1], color2[2], 1.0};
    float specular2[4] = {color2[0], color2[1], color2[2], 1.0};

    GLUquadricObj *qobj;
    qobj = gluNewQuadric();

    gluQuadricDrawStyle(qobj, (GLenum)GLU_FILL);
    gluQuadricNormals(qobj, (GLenum)GLU_SMOOTH);
    
    double radius = 0.01;

    glPushMatrix();

    if (useMotionGraph)
    {
        double matrix[16];
        motion_graph->getTransformMatrix(matrix);
        glMultMatrixd(matrix);
    }
    else
    {
        glMultMatrixd(transformMatrix);
    }

    for (int i = 0; i < numPoints; i++)
    {
        if (i / 6 % 2 == 0)
        {
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
            glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
        }
        else
        {
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient2);
            glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse2);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular2);
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
        }

        glPushMatrix();
        glTranslated(points[i].x(), points[i].y(), points[i].z());
        gluSphere(qobj, radius, 20, 20);
        glPopMatrix();
    }

    glPopMatrix();
    
    gluDeleteQuadric(qobj);
}

void PointCloud::traverse(Bone *ptr)
{
    if (ptr != NULL)
    {
        glPushMatrix();
        genBonePoints(ptr);
        traverse(ptr->child);
        glPopMatrix();
        traverse(ptr->sibling);
    }
}

void PointCloud::genBonePoints(Bone *pBone)
{
    static double z_dir[3] = {0.0, 0.0, 1.0};
    double r_axis[3], theta;

    //Transform (rotate) from the local coordinate system of this bone to it's parent
    //This step corresponds to doing: ModelviewMatrix = M_k * (rot_parent_current)
    glMultMatrixd((double*)&pBone->rot_parent_current);     

    //Draw the local coordinate system for the selected bone.
    /*if((renderMode == BONES_AND_LOCAL_FRAMES) && (pBone->idx == m_SpotJoint))
    {
        GLint lightingStatus;
        glGetIntegerv(GL_LIGHTING, &lightingStatus);
        glDisable(GL_LIGHTING);
        DrawSpotJointAxis();
        if (lightingStatus)
            glEnable(GL_LIGHTING);
    }*/

    //translate AMC (rarely used)
    if(pBone->doftz) 
        glTranslatef(0.0f, 0.0f, float(pBone->tz));
    if(pBone->dofty) 
        glTranslatef(0.0f, float(pBone->ty), 0.0f);
    if(pBone->doftx) 
        glTranslatef(float(pBone->tx), 0.0f, 0.0f);

    //rotate AMC 
    if(pBone->dofrz) 
        glRotatef(float(pBone->rz), 0.0f, 0.0f, 1.0f);
    if(pBone->dofry) 
        glRotatef(float(pBone->ry), 0.0f, 1.0f, 0.0f);
    if(pBone->dofrx) 
        glRotatef(float(pBone->rx), 1.0f, 0.0f, 0.0f);

    //Store the current ModelviewMatrix (before adding the translation part)
    glPushMatrix();

    //Compute tx, ty, tz : translation from pBone to its child (in local coordinate system of pBone)
    double tx = pBone->dir[0] * pBone->length;
    double ty = pBone->dir[1] * pBone->length;
    double tz = pBone->dir[2] * pBone->length;
    
    // Use the current ModelviewMatrix to display the current bone
    // Rotate the bone from its canonical position (elongated sphere 
    // with its major axis parallel to X axis) to its correct orientation
    if(pBone->idx == Skeleton::getRootIndex())
    {
        // glCallList(m_BoneList[skelNum] + pBone->idx);  // no need to draw the root here any more (it is not a bone) 
    }
    else
    { 
        //Compute the angle between the canonical pose and the correct orientation 
        //(specified in pBone->dir) using cross product.
        //Using the formula: r_axis = z_dir x pBone->dir
        v3_cross(z_dir, pBone->dir, r_axis);

        theta =  GetAngle(z_dir, pBone->dir, r_axis);

        glRotatef(float(theta*180./M_PI), float(r_axis[0]), float(r_axis[1]), float(r_axis[2]));

        double matrix[16];
        glGetDoublev(GL_MODELVIEW_MATRIX, matrix);

        // generate points
        //points[numPoints++] = matMultVec3(matrix, vector(0.0, 0.0, 0.0));
        //points[numPoints++] = matMultVec3(matrix, vector(0.0, 0.0, pBone->length));
        double radius = 0.2 * pBone->aspx;
        for (int i = 1; i <= 2; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                double theta = 2.0 * M_PI * ((double)j / 3.0);
                double z = pBone->length * ((double)i / 2.0);
                vector p(radius * cos(theta), radius * sin(theta), z);
                points[numPoints++] = matMultVec3(matrix, p);
            }
        }
    }

    glPopMatrix(); 

    // Finally, translate the bone, depending on its length and direction
    // This step corresponds to doing: M_k+1 = ModelviewMatrix += T_k+1
    glTranslatef(float(tx), float(ty), float(tz));
}
