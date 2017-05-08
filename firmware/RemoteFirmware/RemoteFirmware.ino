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

#define P_ADDR  0
#define I_ADDR  1
#define D_ADDR  2


#include <EEPROM.h>
#include <Gadgetron.h>	
#include <radio.h>
#include <serLCD.h>
#include <Gimbal.h>
#include <quad.h>
#include <checksum.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "FlightController_StateMachine.h"
const long SHIFT_DELAY = 250;
MomentaryButton btn1(PIN_BTN1);
MomentaryButton btn2(PIN_BTN2);
Buzzer buzzer(7);

const int PID_VALUE_LENGTH = 3;
bool view_debug = false;

//Gimbal pins
int left_x = 0; //a0 yaw
int left_y = 1; //a1 throttle
int right_x = 2; //a2 roll
int right_y = 3; //a3 pitch

const int deadzone = 15;
Adafruit_BMP280 bmp; // I2C
//Gimbal objects
Gimbal left_gimbal(left_x, left_y);
Gimbal right_gimbal(right_x, right_y);

int major_notes[7] = { 0, 2, 4, 5, 7, 9, 11 };
pid_controller pid_controllers[PID_VALUE_LENGTH];

bool should_write_to_eeprom;

Vector2 l_g_v, r_g_v;
//LCD object
serLCD mon;
bool should_display_barometer;
char * space = " ";
long last_update_time;
long t_curr;
int pid_idx;
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
	left_gimbal.set_midpoint(508, 501);
	right_gimbal.set_x_bounds(126, 818);
	right_gimbal.set_y_bounds(116, 817);
	right_gimbal.set_midpoint(494, 469);
	btn1.setup();
	btn2.setup();
	buzzer.setup();
	// load pid values from EEPROM
	for (int i = 0; i < PID_VALUE_LENGTH; i++) {
		pid_controllers[i].p = EEPROM.read(sizeof(pid_controller) * i + P_ADDR);
		pid_controllers[i].i = EEPROM.read(sizeof(pid_controller) * i + I_ADDR);
		pid_controllers[i].d = EEPROM.read(sizeof(pid_controller) * i + D_ADDR);
	}
	pid_idx = 0;
	//init serLCD mon
	mon.clear();
	mon.display();
	mon.setBrightness(15); // can go from 1-30
	should_display_barometer = bmp.begin();
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
	if (view_debug) {
		l_g_v = left_gimbal.read_raw();
		r_g_v = right_gimbal.read_raw();
	}
	else {
		l_g_v = left_gimbal.read();
		r_g_v = right_gimbal.read();
		r_g_v.x = _GIMBAL_MAX - r_g_v.x;
		r_g_v.y = _GIMBAL_MAX - r_g_v.y;
		clamp_at_deadzone(r_g_v.x);
		clamp_at_deadzone(r_g_v.y);
		clamp_at_deadzone(l_g_v.x);
	}
	t_curr = millis();
	update_states();
}

void clamp_at_deadzone(int16_t & val) {
	if (abs(_GIMBAL_MID - val) < deadzone) {
		val = _GIMBAL_MID;
	}
}
bool should_update_display() {
	bool rv = t_curr - last_update_time > UPDATE_TIME;
	if (rv) {
		last_update_time = t_curr;
	}
	return rv;
}

void set_pid_value(const char * name, uint8_t & value) {
	value = map(l_g_v.y, _GIMBAL_MIN, _GIMBAL_MAX, 0, 255);
	should_write_to_eeprom = true;
	if (should_update_display()) {
		mon.setCursor(0, 0);
		mon.print("Setting ");
		mon.print(name);
		mon.print(':');
		mon.setCursor(1, 0);
		print_val(value);
	}
}

extern "C" {
	void do_default() {
		int16_t buf[4] = { l_g_v.x, l_g_v.y, r_g_v.x, r_g_v.y };
		if (should_update_display()) {
			display_gimbal_pos(l_g_v, r_g_v);
			/*
			uint16_t checksum = fletcher16((uint8_t *)buf, 8);
			mon.setCursor(0, 0);
			mon.print(checksum);
			*/
		}
		if (!view_debug)
		{
			rfWrite((uint8_t*)buf, 8);
		}
	}
	void do_music_mode() {
		int base_note = map(l_g_v.x, _GIMBAL_MIN, _GIMBAL_MAX, -7, 7);
		int octave = map(l_g_v.y, _GIMBAL_MIN, _GIMBAL_MAX, -1, 8);
		int final_freq = 0;
		if (octave >= 0) {
			int note = major_notes[(base_note+7)%7] - 60;
			note += ((octave) * 12);
			int freq = buzzer.calculateSemitone(note);
			const int k = 2;
			const int dk = 16;
			int modulation_frequency = map(r_g_v.y, _GIMBAL_MIN, _GIMBAL_MAX, -freq / k, freq / k);
			int depth_of_mod = map(r_g_v.x, _GIMBAL_MIN, _GIMBAL_MAX, -freq / dk, freq / dk);
			float fm = depth_of_mod * sin(millis() * modulation_frequency);
			final_freq = freq + fm;
			buzzer.playNote(final_freq);
		}
		else {
			buzzer.turnOff();
		}
		if (should_update_display()) {
			mon.setCursor(0, 0);
			mon.print("Music Mode!");
			mon.setCursor(1, 0);
			print_val(final_freq);
		}
	}
	void do_read_barometer(void) {
		if (should_update_display()) {
			mon.setCursor(0, 0) ;
			mon.print("Altitude: ");
			print_val(bmp.readAltitude(1013.25)*100);
		}

	}
	void do_set_midpoint(void) {
		left_gimbal.set_midpoint();
		right_gimbal.set_midpoint();
	}
	void do_toggle_debug(void) {
		view_debug = !view_debug;
	}
	void do_set_PID() {
		//int kP = 
		if (should_update_display()) {
			mon.setCursor(0, 0);
			switch (pid_idx)
			{
			case YAW_IDX:
				mon.print("Yaw");
				break;
			case PITCH_IDX:
				mon.print("Pitch");
				break;
			case ROLL_IDX:
				mon.print("Roll");
				break;
			}
			mon.print(" PID:");
			mon.setCursor(1, 0);
			mon.print("P");
			print_val(pid_controllers[pid_idx].p);
			mon.print("I");
			print_val(pid_controllers[pid_idx].i);
			mon.print("D");
			print_val(pid_controllers[pid_idx].d);
		}
	}
	void do_set_P() {
		set_pid_value("P", pid_controllers[pid_idx].p);
	}
	void do_set_I() {
		set_pid_value("I", pid_controllers[pid_idx].i);
	}
	void do_set_D() {
		set_pid_value("D", pid_controllers[pid_idx].d);
	}
	void do_prep_write_pid() {
		should_write_to_eeprom = false;
	}
	void do_write_pid_vals() {
		if (should_write_to_eeprom) {
			for (int i = 0; i < PID_VALUE_LENGTH; i++) {
				EEPROM.write(sizeof(pid_controller) * i + P_ADDR, pid_controllers[i].p);
				EEPROM.write(sizeof(pid_controller) * i + I_ADDR, pid_controllers[i].i);
				EEPROM.write(sizeof(pid_controller) * i + D_ADDR, pid_controllers[i].d);
			}
		}
		rfWrite((uint8_t*)pid_controllers, sizeof(pid_controller) * PID_VALUE_LENGTH);
	}
	void shift_control(int dirr) {
		pid_idx += dirr + 3;
		pid_idx %= 3;
		delay(SHIFT_DELAY);
	}
	void do_shift_control_left() {
		shift_control(-1);
	}
	void do_shift_control_right() {
		shift_control(1);
	}
	bool should_set_midpoint() {
		return time_in_state() > 1000;
	}
	bool debug_button() {
		return (btn1.isPressed() && !btn2.isPressed());
	}
	bool on_music_input() {
		return (btn1.isPressed() && btn2.isPressed() && l_g_v.y < GIMBAL_TILT_RIGHT);
	}
	bool pid_button() {
		return (!btn1.isPressed() && btn2.isPressed());
	}
	bool on_set_button() {
		return (btn1.isPressed() || r_g_v.y < GIMBAL_TILT_LEFT);
	}
	bool should_set_P() {
		return r_g_v.x > GIMBAL_TILT_RIGHT ;
	}
	bool should_set_I() {
		return r_g_v.y > GIMBAL_TILT_RIGHT;
	}
	bool should_set_D() {
		return r_g_v.x < GIMBAL_TILT_LEFT;
	}
	bool should_shift_ctl_left() {
		return l_g_v.x < GIMBAL_TILT_LEFT;
	}
	bool should_shift_ctl_right() {
		return l_g_v.x > GIMBAL_TILT_RIGHT;
	}
	bool pid_value_request() {
		rf_get_msg_length() == 1 && is_radio_valid();
	}
	bool should_read_barometer() {
		return should_display_barometer;
	}
	void on_any_transition() {
		mon.clear();
		buzzer.turnOff();
		buzzer.playSemitone(major_notes[state % 7], 100);
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
void print_val(int val) {
	mon.print(val);
	if (val < 1000)
		mon.print(space);
	if (val < 100)
		mon.print(space);
	if (val < 10)
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

