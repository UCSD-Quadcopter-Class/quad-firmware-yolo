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
Vector2 Gimbal::set_midpoint() {
	auto rv = read_raw();
	this->set_midpoint(rv);
	return rv;
}

void Gimbal::set_midpoint(Vector2 bounds) {
	this->set_midpoint(bounds.x, bounds.y);
}
void Gimbal::set_midpoint(uint16_t x, uint16_t y) {
	this->_mid_x = x;
	this->_mid_y = y;
}

Vector2 Gimbal::read()
{
	auto rv = Vector2();
	auto _x = analogRead(_pinX);
	auto _y = analogRead(_pinY);
	if(_x < _mid_x)
	    rv.x = map(_x, _x_lower_bound, _mid_x, _GIMBAL_MIN, _GIMBAL_MID);
	else
	    rv.x = map(_x, _mid_x, _x_upper_bound, _GIMBAL_MID, _GIMBAL_MAX);
	if(_y < _mid_y)
	    rv.y = map(_y, _y_lower_bound, _mid_y, _GIMBAL_MIN, _GIMBAL_MID);
	else
	    rv.y = map(_y, _mid_y, _y_upper_bound, _GIMBAL_MID, _GIMBAL_MAX);
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
