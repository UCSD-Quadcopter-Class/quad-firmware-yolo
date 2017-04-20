// 
// 
// 

#include "Gimbal.h"

Gimbal::Gimbal(uint8_t pinX, uint8_t pinY)
{
	_pinX = pinX;
	_pinY = pinY;
}

void Gimbal::set_x_bounds(uint16_t lower_bound, uint16_t upper_bound)
{
	_x_lower_bound = lower_bound;
	_x_upper_bound = upper_bound;
}

void Gimbal::set_y_bounds(uint16_t lower_bound, uint16_t upper_bound)
{
	_y_lower_bound = lower_bound;
	_y_upper_bound = upper_bound;
}

Vector2 Gimbal::read()
{
	auto rv = Vector2();
	auto _x = analogRead(_pinX);
	auto _y = analogRead(_pinY);
	rv.x = map(_x, _x_lower_bound, _x_upper_bound, _GIMBAL_MIN, _GIMBAL_MAX);
	rv.y = map(_y, _y_lower_bound, _y_upper_bound, _GIMBAL_MIN, _GIMBAL_MAX);
    rv.x = constrain(rv.x, _GIMBAL_MIN, _GIMBAL_MAX);
    rv.y = constrain(rv.y, _GIMBAL_MIN, _GIMBAL_MAX);
	return rv;
}
Vector2 Gimbal::read_raw()
{
	auto rv = Vector2();
	rv.x = analogRead(_pinX);
	rv.y = analogRead(_pinY);
	return rv;
}
