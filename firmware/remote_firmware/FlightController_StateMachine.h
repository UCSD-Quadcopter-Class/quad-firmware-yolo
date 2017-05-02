#ifdef __cplusplus
extern "C" {
#endif
#ifndef FLIGHTCONTROLLER_STATE_STATE_MACHINE
#define FLIGHTCONTROLLER_STATE_STATE_MACHINE

#define DEFAULT_STATE 0
#define SET_PID_STATE 1
#define BUTTON_PRESSED_STATE 2
#define BUTTON_PRESSED2_STATE 3
#include "Arduino.h"
long transitioned_at = 0;
short state = DEFAULT_STATE;
// State Logic Functions
extern void do_default(void);
extern void do_set_PID(void);
// Transitional Logic Functions
extern boolean on_buttons(void);
extern void on_any_transition(void);
void update_states(void) {
	short prev_state = state;
	// The following switch statement handles the state machine's action logic
	switch (state) {
	case DEFAULT_STATE:
			do_default();
			break;
	case SET_PID_STATE:
			do_set_PID();
			break;
	}
	// The following switch statement handles the HLSM's state transition logic
	switch (state) {
	case DEFAULT_STATE:
			if (on_buttons())
				state = BUTTON_PRESSED_STATE;
			break;
	case SET_PID_STATE:
			if (on_buttons())
				state = BUTTON_PRESSED2_STATE;
			break;
	case BUTTON_PRESSED_STATE:
			if (!on_buttons())
				state = SET_PID_STATE;
			break;
	case BUTTON_PRESSED2_STATE:
			if (!on_buttons())
				state = DEFAULT_STATE;
			break;
	}
	if (prev_state != state) {
		transitioned_at = millis();
		on_any_transition();
	}
}

long time_in_state() {
	return millis() - transitioned_at;
}
boolean test_and_set(boolean * variable, boolean val) {
	boolean rv = *variable;
	*variable = val;
	return rv;
}
#endif
#ifdef __cplusplus
}
#endif