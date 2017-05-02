// Gimbal.h

#ifndef _GIMBAL_h
#define _GIMBAL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define _GIMBAL_MIN 0
#define _GIMBAL_MAX 1500

const long _GIMBAL_MID = (_GIMBAL_MAX - _GIMBAL_MIN) / 2 + _GIMBAL_MIN;
const float GIMBAL_TILT_RIGHT = .8f * _GIMBAL_MAX;
const float GIMBAL_TILT_LEFT  = .2f * _GIMBAL_MAX;

struct _Vector2 {
	int16_t x;
	int16_t y;
};
typedef struct _Vector2 Vector2;
class Gimbal {
public:
	Gimbal(uint8_t pinX, uint8_t pinY);
	void set_x_bounds(uint16_t lower_bound, uint16_t upper_bound);
	void set_y_bounds(uint16_t lower_bound, uint16_t upper_bound);
	Vector2 read();
	Vector2 read_raw();
	Vector2 set_midpoint();
	void set_midpoint(Vector2 bounds);
	void set_midpoint(uint16_t x, uint16_t y);
private:
	uint8_t _pinX, _pinY;
	uint16_t _x_lower_bound, _x_upper_bound;
	uint16_t _y_lower_bound, _y_upper_bound;
	uint16_t _mid_x, _mid_y;
};


#endif

