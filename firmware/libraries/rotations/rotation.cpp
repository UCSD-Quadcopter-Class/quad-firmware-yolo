#include "rotation.h"
#include "math.h"
void rotate( rotation_matrix R, vector3 & v, vector3 & out ) {
//void matmul( float ** A, float ** B, float ** C, int m, int n, int o ) 
    float * addrR = (float *)&R.ws;
    float * addrV = (float *)&v.components;
    float * addrOut = (float *)&out.components;
    matmul( &addrR, &addrV, &addrOut , 3, 3, 1 );
}

float sqr_magnitude( quaternion q ){
    float rv = 0;
    for(int i = 0; i < 4; i++ ){
        rv += (q.components[i]*q.components[i]);
    }
    return rv;
}
float magnitude( quaternion q ) {
    return sqrt(sqr_magnitude(q));
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
quaternion operator/(quaternion q, float k) {
    q.w /= k;
    q.v = q.v/k;
    return q;
}
quaternion lerp( quaternion p, quaternion q, float t ) {
    p = p + t* ( q + (-1 * p));
    return normalize(p);
}
quaternion slerp( quaternion v0, quaternion v1, float t ) {
    float dot = dot_product( v0, v1 );
    if( fabs(dot) > 0.9995 ) {
        return lerp(v0,v1,t);
    }
    if( dot < 0 ) {
        v1 = -1 * v1;
        dot = -dot;
    }
    float theta_0 = acos(dot);
    float theta = theta_0 * t;
    quaternion v2 = normalize( v1 + (-dot * v0 ));
    return cos(theta) * v0 + sin(theta) * v2;
}
float dot_product( quaternion p, quaternion q ) {
    float rv = dot_product(p.v, q.v); 
    return (q.w*p.w) + rv;
}
quaternion normalize( quaternion q ) {
    return q/magnitude(q);
};
// Inverse operator
quaternion operator-(quaternion q ) {
    return (*q)/sqr_magnitude(q);
}

quaternion make_quaternion( vector3 u, vector3 v ) {
    quaternion q, p;
    q.w = p.w = 0;
    q.v = u;
    p.v = v;
    q = (*q) * p;
    q.w += 1;
    return normalize(q);
}

quaternion::quaternion_t () : quaternion::quaternion_t(0,0,0,0) { };
quaternion::quaternion_t (float w, vector3 v) { 
    this->w = w;
    this->v = v;
}
quaternion::quaternion_t (float w, float x, float y, float z) { 
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
}
