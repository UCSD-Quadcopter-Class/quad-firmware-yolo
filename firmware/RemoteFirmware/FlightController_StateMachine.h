#ifdef __cplusplus
extern "C" {
#endif
#ifndef FLIGHTCONTROLLER_STATE_STATE_MACHINE
#define FLIGHTCONTROLLER_STATE_STATE_MACHINE

#define DEFAULT_STATE 0
#define SET_PID_STATE 1
#define PREP_WRITE_PID_STATE 2
#define WRITE_PID_VALS_STATE 3
#define SET_P_STATE 4
#define SET_I_STATE 5
#define SET_D_STATE 6
#define DEAD_STATE_STATE 7
#define Q8_STATE 8
#define MUSIC_MODE_STATE 9
#define Q10_STATE 10
#define SHIFT_CONTROL_LEFT_STATE 11
#define SHIFT_CONTROL_RIGHT_STATE 12
#define SET_MIDPOINT_STATE 13
#define Q14_STATE 14
#define TOGGLE_DEBUG_STATE 15
#define READ_BAROMETER_STATE 16
#include "Arduino.h"
long transitioned_at = 0;
short state = DEFAULT_STATE;
// State Logic Functions
extern void do_default(void);
extern void do_set_PID(void);
extern void do_prep_write_pid(void);
extern void do_write_pid_vals(void);
extern void do_set_P(void);
extern void do_set_I(void);
extern void do_set_D(void);
extern void do_music_mode(void);
extern void do_shift_control_left(void);
extern void do_shift_control_right(void);
extern void do_set_midpoint(void);
extern void do_toggle_debug(void);
extern void do_read_barometer(void);
// Transitional Logic Functions
extern bool should_shift_ctl_right();
extern bool on_set_button();
extern bool pid_value_request();
extern bool should_set_D();
extern bool pid_button();
extern bool on_music_input();
extern bool should_read_barometer();
extern bool should_set_I();
extern bool should_set_midpoint();
extern bool should_set_P();
extern bool should_shift_ctl_left();
extern bool debug_button();
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
	case PREP_WRITE_PID_STATE:
			do_prep_write_pid();
			break;
	case WRITE_PID_VALS_STATE:
			do_write_pid_vals();
			break;
	case SET_P_STATE:
			do_set_P();
			break;
	case SET_I_STATE:
			do_set_I();
			break;
	case SET_D_STATE:
			do_set_D();
			break;
	case MUSIC_MODE_STATE:
			do_music_mode();
			break;
	case SHIFT_CONTROL_LEFT_STATE:
			do_shift_control_left();
			break;
	case SHIFT_CONTROL_RIGHT_STATE:
			do_shift_control_right();
			break;
	case SET_MIDPOINT_STATE:
			do_set_midpoint();
			break;
	case TOGGLE_DEBUG_STATE:
			do_toggle_debug();
			break;
	case READ_BAROMETER_STATE:
			do_read_barometer();
			break;
	}
	// The following switch statement handles the HLSM's state transition logic
	switch (state) {
	case DEFAULT_STATE:
			if (pid_value_request())
				state = WRITE_PID_VALS_STATE;
			if (should_read_barometer())
				state = READ_BAROMETER_STATE;
			if (pid_button())
				state = PREP_WRITE_PID_STATE;
			if (on_music_input())
				state = Q8_STATE;
			if (debug_button())
				state = Q14_STATE;
			break;
	case SET_PID_STATE:
			if (should_shift_ctl_right())
				state = SHIFT_CONTROL_RIGHT_STATE;
			if (should_shift_ctl_left())
				state = SHIFT_CONTROL_LEFT_STATE;
			if (pid_button())
				state = WRITE_PID_VALS_STATE;
			if (should_set_D())
				state = SET_D_STATE;
			if (should_set_P())
				state = SET_P_STATE;
			if (should_set_I())
				state = SET_I_STATE;
			break;
	case PREP_WRITE_PID_STATE:
			if (!pid_button())
				state = SET_PID_STATE;
			break;
	case WRITE_PID_VALS_STATE:
			state = DEAD_STATE_STATE;
			break;
	case SET_P_STATE:
			if (on_set_button())
				state = SET_PID_STATE;
			break;
	case SET_I_STATE:
			if (on_set_button())
				state = SET_PID_STATE;
			break;
	case SET_D_STATE:
			if (on_set_button())
				state = SET_PID_STATE;
			break;
	case DEAD_STATE_STATE:
			if (!pid_button())
				state = DEFAULT_STATE;
			break;
	case Q8_STATE:
			if (!on_music_input())
				state = MUSIC_MODE_STATE;
			break;
	case MUSIC_MODE_STATE:
			if (pid_button())
				state = Q10_STATE;
			break;
	case Q10_STATE:
			if (!pid_button())
				state = DEFAULT_STATE;
			break;
	case SHIFT_CONTROL_LEFT_STATE:
			state = SET_PID_STATE;
			break;
	case SHIFT_CONTROL_RIGHT_STATE:
			state = SET_PID_STATE;
			break;
	case SET_MIDPOINT_STATE:
			state = DEFAULT_STATE;
			break;
	case Q14_STATE:
			if (!debug_button())
				state = TOGGLE_DEBUG_STATE;
			if (should_set_midpoint())
				state = SET_MIDPOINT_STATE;
			break;
	case TOGGLE_DEBUG_STATE:
			state = DEFAULT_STATE;
			break;
	case READ_BAROMETER_STATE:
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
bool test_and_set(bool * variable, bool val) {
	bool rv = *variable;
	*variable = val;
	return rv;
}
#endif
#ifdef __cplusplus
}
#endif