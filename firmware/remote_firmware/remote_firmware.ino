 /*****************************************************************************

                                                       Author: Jason Ma
                                                               Michael Gonzalez
                                                       Date:   Apr 19 2017
                                 remote_firmware

 File Name:       remote_firmware.ino
 Description:     Contains RedBoard code for remote to read from gimbals,
                  display to LCD screen, and transmit radio signals.
 *****************************************************************************/
 
#include <radio.h>
#include <serLCD.h>
#include <Gimbal.h>

#define GIMBAL_MAX 900 //not actually max value, but scales value
#define GIMBAL_MIN 0   //not actually min value, but scales value

//Gimbal pins
int left_x  = 0; //a0 yaw
int left_y  = 1; //a1 throttle
int right_x = 2; //a2 roll
int right_y = 3; //a3 pitch

//Gimbal objects
Gimbal left_gimbal (left_x, left_y);
Gimbal right_gimbal (right_x, right_y);

//LCD object
serLCD mon;

char * space = " ";

 /********************************************************************
 | Routine Name: setup
 | File:         remote_firmware.ino
 | 
 | Description: Init gimbal, lcd display
 ********************************************************************/
void setup() {
  //init gimbal bounds
  left_gimbal.set_x_bounds(GIMBAL_MIN, GIMBAL_MAX);
  left_gimbal.set_y_bounds(GIMBAL_MIN, GIMBAL_MAX);
  right_gimbal.set_x_bounds(GIMBAL_MAX, GIMBAL_MIN);
  right_gimbal.set_y_bounds(GIMBAL_MAX, GIMBAL_MIN);

  //init serLCD mon
  mon.clear();
  mon.display();
  mon.setBrightness(15); // can go from 1-30
  //mon.autoscroll(); //autoscroll is enabled by default it seems

  //init radio
  //Serial1.begin(115200);
  //Serial1.println("[yolo] Transmiting board connected");
  rfBegin(22);
  
  //init serial monitor
  //Serial.begin(115200);
  //Serial.print("Initialization complete");
}

 /********************************************************************
 | Routine Name: loop
 | File:         remote_firmware.ino
 | 
 | Description: Read from gimbals, display info on LCD screen, and
 |              send commands to drone using radio.
 ********************************************************************/
void loop() {
  Vector2 l_g_v = left_gimbal.read();
  Vector2 r_g_v = right_gimbal.read();
  
  display_gimbal_pos(l_g_v, r_g_v);

  //TODO hook up motor and control it using throttle (l_g_v.y)

  //int throttle = l_g_v.y / 6;
  //auto reading = l_g_v.y/6;
  /*
  mon.setCursor(0, 0);
  mon.print("Value: ");
  mon.print(reading);
  analogWrite(3, reading);
  if(reading < 100)
    mon.print(space);
  */
  delay(100);
}

 /********************************************************************
 | Routine Name: display_gimbal_pos
 | File:         remote_firmware.ino
 | 
 | Description: Display gimbal values on LCD screen
 |
 | Parameter Descriptions:
 | name               description
 | ------------------ -----------------------------------------------
 | l_g_v (Vector2)    left gimbal values
 | r_g_v (Vector2)    right gimbal values
 ********************************************************************/
void display_gimbal_pos(Vector2 l_g_v, Vector2 r_g_v) {
  //mon.clear() is slow, don't use it if possible
  mon.setCursor(0, 0);
  mon.print("LX:");
  mon.print(l_g_v.x);

  if(l_g_v.x < 1000)
    mon.print(space);
    
  mon.setCursor(1, 0);
  mon.print("LY:");
  mon.print(l_g_v.y);

  if(l_g_v.y < 1000)
    mon.print(space);

  mon.setCursor(0, 8);
  mon.print("RX:");
  mon.print(r_g_v.x);

  if(r_g_v.x < 1000)
    mon.print(space);
    
  mon.setCursor(1, 8);
  mon.print("RY:");
  mon.print(r_g_v.y);

  if(r_g_v.y < 1000)
    mon.print(space);
}

 /********************************************************************
 | Routine Name: print_gimbal_pos
 | File:         remote_firmware.ino
 | 
 | Description: Print gimbal values to serial monitor
 |
 | Parameter Descriptions:
 | name               description
 | ------------------ -----------------------------------------------
 | l_g_v (Vector2)    left gimbal values
 | r_g_v (Vector2)    right gimbal values
 ********************************************************************/
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

