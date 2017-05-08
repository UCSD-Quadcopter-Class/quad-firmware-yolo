/*
 Name:		FlightController.ino
 Created:	4/27/2017 10:44:43 AM
 Author:	Michael
*/
#include<radio.h>
#include<quad.h>
#include<Gimbal.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_Sensor_Set.h>
#include <Mahony.h>
#include <Madgwick.h>

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
sensors_event_t accel, mag, gyro, temp;
sensors_vec_t angle;
Adafruit_LSM9DS1::Sensor gyroscope;
// Magic Numbers~
const float complementary_gain = .96f;
const float PID_RADIO_SCALAR = 64;
// PID Variables
PID controllers[3];
PID * pitch_controller; 
PID * roll_controller;
PID * yaw_controller; 

int8_t pitch_ctl;

bool pid_setup = false;
// Radio Variables
extern uint8_t radio_buffer[];
int16_t* radio_addr;
int16_t gimbal_vec[4];

int16_t throttle;
int8_t target_roll_angle;
const int max_pitch_angle = 20;
uint8_t motor_1_pin = 4;
uint8_t motor_2_pin = 3;
uint8_t motor_3_pin = 8;
uint8_t motor_4_pin = 5;

// Time Variables
long t_last = 0;
long t_curr = 0;
float delta_time = 0;


const int WINDOW_SIZE = (1 << 5);
int window_index = 0;
sensors_vec_t orientation_window_vec;
sensors_vec_t target_vector;
sensors_vec_t previous_error;
sensors_vec_t error_acc_vec;
float i_damp = .90f;

void setup() {
	rfBegin(22);
	Serial.begin(115200);
	radio_addr = (int16_t*) &TRXFBST;
	setup_imu();
	angle *= 0;
	previous_error *= 0;
	error_acc_vec *= 0;
	orientation_window_vec = 0 * orientation_window_vec ;
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
	graph(1.0f / delta_time);
}

void pid_update() {
	// Roll
	pid_update(PITCH_IDX, pitch_ctl);
	error_acc_vec = i_damp * error_acc_vec;
}

void pid_update(int idx, int8_t & val) {
	auto pid = controllers[idx];
	auto error = target_vector.v[idx] - angle.v[idx];
	auto dE_dT = (error - previous_error.v[idx])/delta_time;
	auto & error_acc = error_acc_vec.v[idx];
	graph(target_vector.v[idx]);
	graph(angle.v[idx]);
	graph(error);
	error_acc += (error * delta_time);
	auto new_ctl = pid.p * error + pid.i * error_acc + pid.d * dE_dT;
	val = constrain(new_ctl, -255, 255);
	previous_error.v[idx] = error;
	graph(val);
}

void motor_update() {
	float m1_ctl = 0;
	float m2_ctl = 0; 
	float m3_ctl = 0; 
	float m4_ctl = 0; 
	if (throttle > 0) {
		m1_ctl = max(throttle + pitch_ctl, 0);
		m2_ctl = max(throttle + pitch_ctl, 0);
		m3_ctl = max(throttle - pitch_ctl, 0);
		m4_ctl = max(throttle - pitch_ctl, 0);
	}
	graph(m1_ctl);
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
	sensors_event_t a, m, g, temp;
	sensors_vec_t orientation;
	gyroscope.getEvent(&g);
	angle = angle + (delta_time * g.gyro);
	//graph(angle.y);
	if (ahrs.getOrientation(&orientation))
	{
		//graph(orientation.y);
		orientation_window_vec = orientation + orientation_window_vec;
		window_index++;
		if (window_index > WINDOW_SIZE) {
			window_index = 0;
			orientation_window_vec = orientation_window_vec / WINDOW_SIZE;
			//graph(orientation_window_vec.y / window_index);
			// Do Complementary filter here
			angle = (complementary_gain * angle) + ((1.0f - complementary_gain) * orientation_window_vec);
			orientation_window_vec *= 0;
		}
		//delay(100);
		//graph(angle);
		Serial.println();
	}
	//Serial.println(delta_time);
	//lsm.getEvent(&a, &m, &g, &temp);
	//graph_vec(a.acceleration);
	//graph_vec(g.gyro);
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
void gimbal_radio() {
	for (int i = 0; i < 4; i++) {
			int16_t value = ((int16_t*)radio_buffer)[i];
			//Serial.print(value);
			//Serial.print(" ");
			gimbal_vec[i] = value;
	}
	throttle = map( gimbal_vec[1], _GIMBAL_MIN, _GIMBAL_MAX, 0, 75);
	target_vector.pitch= map(gimbal_vec[2], _GIMBAL_MIN, _GIMBAL_MAX, -max_pitch_angle, max_pitch_angle);
	target_vector.pitch= constrain(target_vector.pitch, -max_pitch_angle, max_pitch_angle);
	//Serial.println();
}
