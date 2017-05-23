/*
 Name:		FlightController.ino
 Created:	4/27/2017 10:44:43 AM
 Author:	Michael
*/
#include<radio.h>
#include<quad.h>
#include<Gimbal.h>
#include "rotation.h"
#include "vectors.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_Sensor_Set.h>

sensors_vec_t operator+(sensors_vec_t  src, sensors_vec_t   rhs) {
	src.x += rhs.x;
	src.y += rhs.y;
	src.z += rhs.z;
	return src;
}

sensors_vec_t operator*(float scalar, sensors_vec_t vec) {
	vec.x *= scalar;
	vec.y *= scalar;
	vec.z *= scalar;
	return vec;
}
sensors_vec_t operator/(sensors_vec_t vec, float scalar) {
	vec.x /= scalar;
	vec.y /= scalar;
	vec.z /= scalar;
	return vec;
}

sensors_vec_t operator *= (sensors_vec_t vec, float scalar) {
	return (scalar * vec);
}

sensors_vec_t operator+=(sensors_vec_t a, sensors_vec_t b) {
	return a + b;
}

// Sensors
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();  // i2c sensor
Adafruit_Simple_AHRS ahrs(&(lsm._accelSensor), &(lsm._magSensor) );
sensors_vec_t angle;
quaternion orientation;
Adafruit_LSM9DS1::Sensor gyroscope;
// Magic Numbers~
const float complementary_gain = .96f;
const float PID_RADIO_SCALAR = 4;
// PID Variables
PID controllers[3];
PID * pitch_controller; 
PID * roll_controller;
PID * yaw_controller; 

int16_t pitch_ctl;
int16_t roll_ctl;
int16_t yaw_ctl;
sensors_event_t a, m, g, temp;

float const deg2rad = PI / 180;
bool has_set;
vector3 forward, right, up, one, zero, reference;

vector3 accel_acc, error, error_old, delta_error, error_acc;


bool pid_setup = false;
// Radio Variables
extern uint8_t radio_buffer[];
int16_t* radio_addr;
int16_t gimbal_vec[4];

int16_t throttle;
const float max_yaw_speed = 4;
const float max_pitch_angle = .3;
const float max_roll_angle = max_pitch_angle;
uint8_t motor_1_pin = 4;
uint8_t motor_2_pin = 3;
uint8_t motor_3_pin = 8;
uint8_t motor_4_pin = 5;

// Time Variables
long t_last = 0;
long t_curr = 0;
float delta_time = 0;

vector3 target_vector;

const int WINDOW_SIZE = (1 << 3);
int window_index = 0;
sensors_vec_t orientation_window_vec_old;
sensors_vec_t previous_error;
sensors_vec_t error_acc_vec;
float i_damp = .99f;

void setup() {
	forward = make_vector(0, -1, 0);
	up = make_vector(0, 0, -1);
	right = make_vector(1, 0, 0);
	one = make_vector(1, 1, 1);
	zero = make_vector(0, 1, 0);
	reference = up;
	rfBegin(22);
	Serial.begin(115200);
	radio_addr = (int16_t*) &TRXFBST;
	setup_imu();
	angle *= 0;
	previous_error *= 0;
	error_acc_vec *= 0;
	error_old = zero;
	accel_acc = zero;
	orientation_window_vec_old = 0 * orientation_window_vec_old ;
	for (int i = 0; i < 3; i++) {
		controllers[i].p = 50;
	}
}

void setup_imu() {
	if (!lsm.begin())
	{
		/* There was a problem detecting the LSM9DS1 ... check your connections */
		Serial.print(F("Ooops, no LSM9DS1 detected ... Check your wiring!"));
		while (1);
	}
	lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
	lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
	lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
	gyroscope = lsm.getGyro();
}

void loop() {
	t_curr = millis(); 
	delta_time = (t_curr - t_last) / 1000.0f;// +micros() / 1000000.0f;
	radio_update();
	debug_imu();
	pid_update();
	motor_update();
	t_last = t_curr;
	//graph(1.0f / delta_time);
}

void pid_update() {
	// Roll
	vector3 curr = orientation * reference;
	curr.z = g.gyro.z/(1.0f* (1<<9));
	error = target_vector - curr;
	delta_error = (error - error_old) / delta_time;
	error_acc = error + (error );
	print(error);
	pid_update(0, pitch_ctl, true);
	pid_update(1, roll_ctl, false);
	pid_update(2, yaw_ctl, false);
	//yaw_ctl = target_vector.heading;
	error_acc = i_damp * error_acc;
	error_old = error;
}

void pid_update(int idx, int16_t & val, bool debug) {
	auto pid = controllers[idx];
	if (debug) {
		graph(target_vector.components[idx]);
	}
	auto new_ctl = pid.p * error.components[idx] + pid.i * error_acc.components[idx] + pid.d * delta_error.components[idx];
	val = constrain(new_ctl, -255, 255);
	if (debug) {
		graph(val);
		graph(error_acc.components[idx]);
	}
}

void motor_update() {
	int m1_ctl = 0;
	int m2_ctl = 0; 
	int m3_ctl = 0; 
	int m4_ctl = 0; 
	if (throttle > 0) {
		m1_ctl = constrain(throttle - pitch_ctl + roll_ctl + yaw_ctl, 0,255);
		m2_ctl = constrain(throttle - pitch_ctl - roll_ctl - yaw_ctl, 0,255);
		m3_ctl = constrain(throttle + pitch_ctl - roll_ctl + yaw_ctl, 0,255);
		m4_ctl = constrain(throttle + pitch_ctl + roll_ctl - yaw_ctl, 0,255);
	}
	//graph(m1_ctl);
	analogWrite(motor_1_pin, m1_ctl);
	analogWrite(motor_2_pin, m2_ctl);
	analogWrite(motor_3_pin, m3_ctl);
	analogWrite(motor_4_pin, m4_ctl);
}

void debug_radio() {
  for (int i = 0; i < 4; i++)
  {
    auto value = gimbal_vec[i];
    Serial.print(value);
    Serial.print(' ');
  }
  
}

inline void integrate(sensors_vec_t & acc, sensors_vec_t & new_value) {
	new_value = delta_time * new_value;
	acc = acc + new_value;
}

void debug_imu() {
	lsm.read();  /* ask it to read in the data */
				 /* Get a new sensor event */
	sensors_vec_t adafruit_orientation;

	lsm.getEvent(&a, &m, &g, &temp);
	vector3 accel = normalize(vec_from_sensor(a.acceleration));
	vector3 gyro = deg2rad * delta_time * vec_from_sensor(g.gyro);
	if (!has_set) {
		orientation = make_quaternion(reference, accel);
		has_set = true;
	}
	else {
		rotate_by_gyro(gyro.y, forward);
		rotate_by_gyro(gyro.z, up);
			//graph(orientation_window_vec.y / window_index);
		rotate_by_gyro(gyro.x, right);
	}
	{
		orientation_window_vec_old = adafruit_orientation + orientation_window_vec_old;
		accel_acc = accel_acc + accel;
		window_index++;
		if (window_index > WINDOW_SIZE) {
			window_index = 0;
			accel_acc = accel_acc / WINDOW_SIZE;
			// Do Complementary filter here
			orientation = slerp(orientation, make_quaternion(reference, accel_acc), 1-complementary_gain);
			orientation_window_vec_old *= 0;
			accel_acc = zero;
		}
		//print(accel);
		//print(orientation * reference);
		angle.z = g.gyro.z;
		Serial.println();
	}
}

inline void graph(int val) {
	Serial.print(val);
	Serial.print("\t");
}

inline void graph(sensors_vec_t v) {
	graph(v.x);
	graph(v.y);
	graph(v.z);
}

void radio_update() {
 if(is_radio_valid()) {
  switch( rf_get_msg_length()) {
    case 8: // Recieved 8 bytes
      gimbal_radio();
      break;
    case 9:
      pid_radio();
      break;
  }
  //Serial.println( IS_RADIO_VALID );
 }
}

void pid_radio() {
  pid_controller ctrl_in[3];
  error_acc_vec = 0 * error_acc_vec;
  for( int i = 0; i < 3; i++ ) {
    auto & temp = controllers[i];
    temp.p = radio_buffer[ i * 3 + 0]/PID_RADIO_SCALAR;
    temp.i = radio_buffer[ i * 3 + 1]/PID_RADIO_SCALAR;
    temp.d = radio_buffer[ i * 3 + 2]/PID_RADIO_SCALAR;
    Serial.print( temp.p );
    Serial.print( " " );
    Serial.print( temp.i );
    Serial.print( " " );
    Serial.println( temp.d );
  }
}
void set_target( float & target, float reading, float max_angle ) {
    target = map(reading, _GIMBAL_MIN, _GIMBAL_MAX, -max_angle * 100, max_angle * 100);
    target = constrain(target, -max_angle * 100, max_angle * 100)/100.f;
}
void gimbal_radio() {
	for (int i = 0; i < 4; i++) {
			int16_t value = ((int16_t*)radio_buffer)[i];
			//Serial.print(value);
			//Serial.print(" ");
			gimbal_vec[i] = value;
	}
	throttle = map( gimbal_vec[1], _GIMBAL_MIN, _GIMBAL_MAX, 0, 200);
    set_target( target_vector.z, gimbal_vec[0], max_yaw_speed);
    set_target( target_vector.x, gimbal_vec[2], max_pitch_angle );
    set_target( target_vector.y, gimbal_vec[3], max_roll_angle);
	//Serial.println();
}
void rotate_by_gyro(float rads, vector3 temp) {
	quaternion q(cos(rads / 2), sin(rads / 2) * temp);
	orientation = q * orientation;
}
void print(vector3 val) {
	print(val.x);
	print(val.y);
	print(val.z);
}
void print(float val) {
	Serial.print(val);
	Serial.print("\t");
}

vector3 vec_from_sensor(sensors_vec_t v) {
	vector3 rv;
	rv.x = v.x;
	rv.y = v.y;
	rv.z = v.z;
	return rv;
}