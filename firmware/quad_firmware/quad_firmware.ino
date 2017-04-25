 /*****************************************************************************

                                                       Author: Jason Ma
                                                               Michael Gonzalez
                                                       Date:   Apr 19 2017
                                  quad_firmware

 File Name:       quad_firmware.ino
 Description:     Contains RedBoard code for remote to read from gimbals,
                  display to LCD screen, and transmit radio signals.
 *****************************************************************************/

#include <radio.h>
#define MOTOR_PIN_1 3
void setup() {
  rfBegin(22);
}

void loop() {
  while(rfAvailable() ) {
    auto throttle = rfRead();
    analogWrite(MOTOR_PIN_1, throttle);
  }
}
