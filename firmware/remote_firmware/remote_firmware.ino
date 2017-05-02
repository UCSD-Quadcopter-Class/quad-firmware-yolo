 /*****************************************************************************

                                                       Author: Jason Ma
                                                               Michael Gonzalez
                                                       Date:   Apr 19 2017
                                 remote_firmware

 File Name:       remote_firmware.ino
 Description:     Contains RedBoard code for remote to read from gimbals,
                  display to LCD screen, and transmit radio signals.
 *****************************************************************************/
#define PIN_BTN1    16    // PG0 (schematic) G0 (red board)
#define PIN_BTN2    17    // PG1 (schematic) G1 (red board)
#define PIN_LED_BLUE  22    // PD6 (schematic) D4 (red board)
#define PIN_LED_GRN   23    // PD5 (schematic) D5 (red board)
#define PIN_LED_RED   24    // PD4 (schematic) D6 (red board)   
#define UPDATE_TIME 50;
#include<Gadgetron.h>

#include <radio.h>
#include <serLCD.h>
#include <Gimbal.h>
#include <quad.h>
#include "FlightController_StateMachine.h"
MomentaryButton btn1(PIN_BTN1);
MomentaryButton btn2(PIN_BTN2);
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
long t_curr;
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
  btn1.setup();
  btn2.setup();
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
  Serial.begin(115200);
  Serial.print("Initialization complete");
}

 /********************************************************************
 | Routine Name: loop
 | File:         remote_firmware.ino
 | 
 | Description: Read from gimbals, display info on LCD screen, and
 |              send commands to drone using radio.
 ********************************************************************/
void loop() {
	t_curr = millis();
	update_states();
  //prints specifically throttle to screen
  //mon.setCursor(0, 0);
  //mon.print("Value: ");
  //mon.print(reading);

  //for hooking remote directly to motor
  //analogWrite(3, reading);
  
}
bool should_update_display() {
	bool rv = t_curr - last_update_time > UPDATE_TIME;
	if(rv)
		last_update_time = t_curr;
	return rv;
}

extern "C" {
	void do_default() {
		Vector2 l_g_v = left_gimbal.read();
		Vector2 r_g_v = right_gimbal.read();
		if (should_update_display) {
			display_gimbal_pos(l_g_v, r_g_v);
		}
		//get throttle
		int16_t buf[5] = { SECRET_NUMBER, l_g_v.x, l_g_v.y, r_g_v.x, r_g_v.y };
		rfWrite((uint8_t*)buf, 10);
	}
	void do_set_PID() {

	}
	boolean on_buttons() {
		return (!btn1.isPressed()) && (!btn2.isPressed());
	}
	void on_any_transition() {

	}
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

