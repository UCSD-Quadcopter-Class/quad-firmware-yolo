#include "rotation.h"
#include <iostream>
#include "math.h"

using namespace std;
#define DEBUG( x ) cout << "Value of "#x ": " << (x) << endl   
void debug_quaternion_def() {
    quaternion q;
    q.q0 = 1;
    q.q1 = 2;
    q.q2 = 3;
    q.q3 = 4;

    DEBUG( q.q0 );
    DEBUG( q.q1 );
    DEBUG( q.q2 );
    DEBUG( q.q3 );

    DEBUG( q.v.components[0] );
    DEBUG( q.v.components[1] );
    DEBUG( q.v.components[2] );

    DEBUG( q.w );
    DEBUG( q.x );
    DEBUG( q.y );
    DEBUG( q.z );
    DEBUG( sizeof(quaternion) );
}
void debug_vector( ) {
    vector3 a, b;
    a.x = 1;
    a.y = 0;
    a.z = 3;
    b.x = 8;
    b.y = 0;
    b.z = 2;
    DEBUG( dot_product( a , b ));
 
}

void debug_vec() {
   vector3 v,u;
   v.x = 1;
   v.y = 2;
   v.z = 3;
   DEBUG(v.x);
   DEBUG(v.y);
   DEBUG(v.z);
   v = -v;
   DEBUG(v.x);
   DEBUG(v.y);
   DEBUG(v.z);
 
}

void debug_quaternion_ops() {
  quaternion p, q, r;
  p.q0 =  3;
  p.q1 =  1;
  p.q2 = -2;
  p.q3 =  1;

  q.q0 =  2;
  q.q1 = -1;
  q.q2 =  2;
  q.q3 =  3;
  r = p * q;
  DEBUG( r.q0 );
  DEBUG( r.q1 );
  DEBUG( r.q2 );
  DEBUG( r.q3 );
  r = *r;
  DEBUG( r.q0 );
  DEBUG( r.q1 );
  DEBUG( r.q2 );
  DEBUG( r.q3 );
 
}

int main( ) {
  quaternion q, q2, lerped;
  vector3 up, forward, right, whoa;
  set_vector( up, 0, 1, 0);
  set_vector( forward, 0, 0, 1);
  set_vector( right, 1, 0, 0);

  whoa = right;
  DEBUG( whoa.x );
  DEBUG( whoa.y );
  DEBUG( whoa.z );

  float theta = M_PI;
  q.x = 0;
  q.y = sin(theta/2);
  q.z = 0;
  q.q0 = cos(theta/2);
  q2.x = 0;
  q2.y = 0;
  q2.z = sin(theta/2);
  q2.q0 = cos(theta/2);
  lerped = lerp( q, q2, .25); 
  DEBUG( magnitude( q ) );
  DEBUG( magnitude( q2 ) );
  DEBUG( magnitude( lerped));

  DEBUG( lerped.x );
  DEBUG( lerped.y );
  DEBUG( lerped.z );
  DEBUG( lerped.w );

  whoa = lerped * whoa;
  DEBUG( whoa.x );
  DEBUG( whoa.y );
  DEBUG( whoa.z );


  return 0;
}
