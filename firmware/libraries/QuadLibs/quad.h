#ifndef QUAD_LIBS
#define QUAD_LIBS

#define ROLL_IDX 0
#define PITCH_IDX 1
#define YAW_IDX 2

#define SECRET_NUMBER 7890
struct controller {
	uint8_t p;
	uint8_t i;
	uint8_t d;
};
typedef struct controller pid_controller;
struct quad_pid {
	float p;
	float i;
	float d;
};

typedef struct quad_pid PID;


#endif