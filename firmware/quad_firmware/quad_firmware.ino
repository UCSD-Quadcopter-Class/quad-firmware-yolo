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

void setup() {
  Serial1.begin(115200);
  Serial1.println("[yolo] Receiving board connected");
}

void loop() {
  
  
}
