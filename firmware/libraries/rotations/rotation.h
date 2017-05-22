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

struct quaternion_t {
    union {
        float components[4];
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
        struct{
            float x;
            float y;
            float z;
            float w;
        };
    };
};

typedef struct quaternion_t quaternion;

void rotate( rotation_matrix R, vector3 & v, vector3 & out ) ;

// Complex conjugate operator
quaternion operator*(quaternion p );
quaternion operator*(quaternion p, quaternion q);
quaternion operator*(float k, quaternion q);
quaternion operator+(quaternion p, quaternion q);
vector3 operator*(quaternion q, vector3 v);
float magnitude( quaternion q );
quaternion lerp( quaternion p, quaternion q, float t );

#endif