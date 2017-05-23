#ifndef VECTORS
#define VECTORS
union vector3_t {
    struct {
            float x;
            float y;
            float z;
        };
    float components[3];
};

typedef union vector3_t vector3;

namespace Vector3 {
    
}

float dot_product( vector3 a, vector3 b );
float dot_product( float * a, float * b, int n );

void matmul( float ** A, float ** B, float ** C, int m, int n, int o );

vector3 operator*( float k, vector3 v );
vector3 operator*( vector3 v, float k );
vector3 operator/( vector3 v, float k );
vector3 operator+( vector3 v, vector3 u );
vector3 operator-( vector3 v, vector3 u );
vector3 operator-( vector3 v );
vector3 cross_product( vector3 a, vector3 b );
vector3 normalize( vector3 v ); 
float sqr_magnitude( vector3 v );
float magnitude( vector3 v ); 
void set_vector( vector3 & v, float x, float y, float z);
vector3 make_vector(  float x, float y, float z);


#endif 
