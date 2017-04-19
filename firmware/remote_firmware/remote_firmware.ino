#include <radio.h>
#include <serLCD.h>
#include <Gimbal.h>

#define GIMBAL_MAX 1500
#define GIMBAL_MIN 0

//LCD
serLCD mon;

//Gimbal pins
int left_x  = 0; //a0 yaw
int left_y  = 1; //a1 throttle
int right_x = 2; //a2 roll
int right_y = 3; //a3 pitch

//Gimbal objects
Gimbal left_gimbal (left_x, left_y);
Gimbal right_gimbal (right_x, right_y);

void setup() {
  //init gimbal bounds
  right_gimbal.set_x_bounds(GIMBAL_MIN, GIMBAL_MAX);
  right_gimbal.set_y_bounds(GIMBAL_MIN, GIMBAL_MAX);
  left_gimbal.set_x_bounds(GIMBAL_MIN, GIMBAL_MAX);
  left_gimbal.set_y_bounds(GIMBAL_MIN, GIMBAL_MAX);

  //init serLCD mon
  mon.clear();
  mon.display();
  mon.setBrightness(15); // can go from 1-30
  //mon.autoscroll();

  //init serial monitor
  //Serial.begin(115200);
  //Serial.print("Initialization complete");
}

void loop() {
  Vector2 l_g_v = left_gimbal.read();
  Vector2 r_g_v = right_gimbal.read();
  
  //mon.clear();

  mon.setCursor(0, 0);
  mon.print(l_g_v.x);
  mon.setCursor(0, 8);
  mon.print(l_g_v.y);

  mon.setCursor(1, 0);
  mon.print(r_g_v.x);
  mon.setCursor(1, 8);
  mon.print(r_g_v.y);
  
  //print_gimbal_pos(l_g_v, r_g_v);  
}

void print_gimbal_pos(Vector2 l_g_v, Vector2 r_g_v) {
  Serial.print(l_g_v.x);
  Serial.print("\t");
  Serial.print(l_g_v.y);
  Serial.print("\t");
  Serial.print(r_g_v.x);
  Serial.print("\t");
  Serial.print(r_g_v.y);
  Serial.print("\n");
}

