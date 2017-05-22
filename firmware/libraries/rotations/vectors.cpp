#include "vectors.h"


float dot_product( float * a, float * b, int n ) {
    float rv = 0;
    for( int i = 0; i < n; i++ ) {
        rv += a[i] * b[i];
    }
    return rv;
}

void matmul( float ** A, float ** B, float ** C, int m, int n, int p ) {
    for( int i = 0; i < m; i++ ) {
        for( int j = 0; j < p; j++ ) {
            float sum = 0;
            for( int k = 0; k < n; k++ ) {
                sum = A[i][k] * B[j][k];
            }
            C[i][j] = sum;
        }
    }
}

vector3 operator+( vector3 v, vector3 u ) {
    v.x += u.x;
    v.y += u.y;
    v.z += u.z;
    return v;
}
void set_vector( vector3 & v, float x, float y, float z) {
    v.x = x;
    v.y = y;
    v.z = z;
}

vector3 operator-( vector3 v ) {
    v.x *= -1;
    v.y *= -1;
    v.z *= -1;
    return v;
}
vector3 operator*( vector3 v, float k ) {
    v.x *= k;
    v.y *= k;
    v.z *= k;
    return v;
}
vector3 operator*( float k, vector3 v ){
    v.x *= k;
    v.y *= k;
    v.z *= k;
    return v;
};


float dot_product( vector3 a, vector3 b ) {
    return dot_product( a.components, b.components, 3 );
}
