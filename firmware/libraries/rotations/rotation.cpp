#include "rotation.h"
#include "math.h"
void rotate( rotation_matrix R, vector3 & v, vector3 & out ) {
//void matmul( float ** A, float ** B, float ** C, int m, int n, int o ) 
    float * addrR = (float *)&R.ws;
    float * addrV = (float *)&v.components;
    float * addrOut = (float *)&out.components;
    matmul( &addrR, &addrV, &addrOut , 3, 3, 1 );
}

float magnitude( quaternion q ) {
    float rv = 0;
    for(int i = 0; i < 4; i++ ){
        rv += (q.components[i]*q.components[i]);
    }
    return sqrt(rv);
}

quaternion operator*(quaternion p, quaternion q) {
    quaternion rv;
    rv.q0 = (p.q0*q.q0) - (p.q1*q.q1) - (p.q2*q.q2) - (p.q3*q.q3); 
    rv.q1 = (p.q0*q.q1) + (p.q1*q.q0) + (p.q2*q.q3) - (p.q3*q.q2); 
    rv.q2 = (p.q0*q.q2) - (p.q1*q.q3) + (p.q2*q.q0) + (p.q3*q.q1); 
    rv.q3 = (p.q0*q.q3) + (p.q1*q.q2) - (p.q2*q.q1) + (p.q3*q.q0); 
    return rv;
}
// Complex conjugate operator
quaternion operator*(quaternion p ) {
    p.v = -p.v;
    return p;
}

vector3 operator*(quaternion q, vector3 v) {
    quaternion pure;
    pure.q0 = 0;
    pure.v = v;
    vector3 rv = ((*q) * pure * (q)).v;
    return rv;
}
quaternion operator+(quaternion p, quaternion q) {
    p.w += q.w;
    p.v = p.v + q.v;
    return p;
}
quaternion operator*(float k, quaternion q) {
    q.w *= k;
    q.v = k * q.v;
    return q;
}
quaternion lerp( quaternion p, quaternion q, float t ) {
    p = t*p + (1-t) * q;
    return (1.0f/magnitude(p))*p ;
}
