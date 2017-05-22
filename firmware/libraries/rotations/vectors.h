#ifndef VECTORS
#define VECTORS
struct vector3_t {
    union {
        struct {
            float x;
            float y;
            float z;
        };
        float components[3];
        float ** as_matrix;
    };
};

typedef struct vector3_t vector3;


float dot_product( vector3 a, vector3 b );
float dot_product( float * a, float * b, int n );

void matmul( float ** A, float ** B, float ** C, int m, int n, int o );

vector3 operator*( float k, vector3 v );
vector3 operator*( vector3 v, float k );
vector3 operator+( vector3 v, vector3 u );
vector3 operator-( vector3 v );
void set_vector( vector3 & v, float x, float y, float z);



#endif 
