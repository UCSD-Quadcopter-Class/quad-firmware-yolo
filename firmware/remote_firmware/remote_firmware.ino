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
long last_update_time;
 /********************************************************************
 | Routine Name: setup
 | File:         remote_firmware.ino
 | 
 | Description: Init gimbal, lcd display
 ********************************************************************/
void setup() {
  //init gimbal bounds
  left_gimbal.set_x_bounds(131, 818);
  left_gimbal.set_y_bounds(135, 818);
  right_gimbal.set_x_bounds(818, 118);
  right_gimbal.set_y_bounds(818, 116);

  //init serLCD mon
  mon.clear();
  mon.display();
  mon.setBrightness(15); // can go from 1-30
  //mon.autoscroll(); //autoscroll is enabled by default it seems

  //init radio
  //Serial1.begin(115200);
  //Serial1.println("[yolo] Transmiting board connected");
  rfBegin(22);
  last_update_time = 0;
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
  long t_curr = millis();
  if( t_curr - last_update_time > 50 ) {
      display_gimbal_pos(l_g_v, r_g_v);
      last_update_time = t_curr;
  }
  //get throttle
  auto reading = l_g_v.y;
  rfWrite(reading);

  //prints specifically throttle to screen
  //mon.setCursor(0, 0);
  //mon.print("Value: ");
  //mon.print(reading);

  //for hooking remote directly to motor
  //analogWrite(3, reading);
  
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
  print_val(l_g_v.x);
    
  mon.setCursor(1, 0);
  mon.print("LY:");
  print_val(l_g_v.y);


  mon.setCursor(0, 8);
  mon.print("RX:");
  print_val(r_g_v.x);
  
  mon.setCursor(1, 8);
  mon.print("RY:");
  print_val(r_g_v.y);

}
void print_val( int val ) {
    mon.print(val);
    if(val < 1000)
      mon.print(space);
    if(val < 100 )
      mon.print(space);
    if(val < 10 )
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

