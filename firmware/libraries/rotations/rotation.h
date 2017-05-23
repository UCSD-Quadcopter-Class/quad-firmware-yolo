#ifndef ROTATIONS_HEADER
#define ROTATIONS_HEADER
#include "vectors.h"


struct rotation_matrix_t {
    union {
        float * raw[9];
        struct {
            float ws[3][3];
        };
    };
};

typedef struct rotation_matrix_t rotation_matrix;
union quaternion_t {
        float components[4];
        struct{
            float x;
            float y;
            float z;
            float w;
        };
        struct{
            union { 
                struct {
                    float q1;
                    float q2;
                    float q3;
                };
                vector3 v;
            };
            float q0;
        };
        quaternion_t ();
        quaternion_t (float w, float x, float y, float z);
        quaternion_t (float w, vector3 v);
};

typedef union quaternion_t quaternion;

void rotate( rotation_matrix R, vector3 & v, vector3 & out ) ;

// Inverse operator
quaternion operator-(quaternion p );
// Complex conjugate operator
quaternion operator*(quaternion p );
quaternion operator*(quaternion p, quaternion q);
quaternion operator*(float k, quaternion q);
quaternion operator/(quaternion q, float k);
quaternion operator+(quaternion p, quaternion q);
vector3 operator*(quaternion q, vector3 v);
float magnitude( quaternion q );
float sqr_magnitude( quaternion q );
quaternion lerp( quaternion p, quaternion q, float t );
quaternion normalize( quaternion q );
quaternion make_quaternion( vector3 u, vector3 v );
quaternion slerp( quaternion p, quaternion q, float t );
float dot_product( quaternion p, quaternion q ) ;

#endif