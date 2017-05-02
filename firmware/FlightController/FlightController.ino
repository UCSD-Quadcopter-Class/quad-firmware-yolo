/*
 Name:		FlightController.ino
 Created:	4/27/2017 10:44:43 AM
 Author:	Michael
*/



#include<radio.h>
#include<quad.h>
#include<Gimbal.h>

int16_t* radio_addr;
int16_t gimbal_vec[4];

// the setup function runs once when you press reset or power the board
void setup() {
	rfBegin(22);
	Serial.begin(9600);
	radio_addr = (int16_t*) &TRXFBST;
}

// the loop function runs over and over again until power down or reset
void loop() {
	radio_update();
	auto value = map( gimbal_vec[1], 0, 1500, 0, 255 );
  analogWrite( 3, value);
  Serial.println( value );
}

void debug_radio() {
  for (int i = 0; i < 4; i++)
  {
    auto value = gimbal_vec[i];
    Serial.print(value);
    Serial.print(' ');
  }
  Serial.println();
}
void radio_update() {
	if (radio_addr[0] != SECRET_NUMBER)
		return;
	for (int i = 1; i < 5; i++) {
			int16_t value = (int16_t)radio_addr[i];
			if (value > _GIMBAL_MAX || value < _GIMBAL_MIN)
				continue;
			gimbal_vec[i - 1] = value;
	}
}
