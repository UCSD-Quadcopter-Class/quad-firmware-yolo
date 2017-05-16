#ifndef REMOTE
#define REMOTE

#define PIN_BTN1    16    // PG0 (schematic) G0 (red board)
#define PIN_BTN2    17    // PG1 (schematic) G1 (red board)
#define PIN_LED_BLUE  22    // PD6 (schematic) D4 (red board)
#define PIN_LED_GRN   23    // PD5 (schematic) D5 (red board)
#define PIN_LED_RED   24    // PD4 (schematic) D6 (red board)   
#define UPDATE_TIME 50;

MomentaryButton btn1(PIN_BTN1);
MomentaryButton btn2(PIN_BTN2);
Buzzer buzzer(8);
const int PID_VALUE_LENGTH = 3;

//LCD object
serLCD mon;

//Gimbal pins
const int left_x = 0; //a0 yaw
const int left_y = 1; //a1 throttle
const int right_x = 2; //a2 roll
const int right_y = 3; //a3 pitch

//Gimbal objects
Gimbal left_gimbal(left_x, left_y);
Gimbal right_gimbal(right_x, right_y);
Vector2 l_g_v, r_g_v;

Adafruit_BMP280 bmp; // I2C

int major_notes[7] = { 0, 2, 4, 5, 7, 9, 11 };
pid_controller pid_controllers[PID_VALUE_LENGTH];

bool should_write_to_eeprom;
bool should_display_barometer;
long last_update_time;
long t_curr;
int pid_idx;

char * space = " ";

const long SHIFT_DELAY = 250;
bool view_debug = false;
const int deadzone = 40;

bool should_update_display();
void set_pid_value(const char * name, uint8_t & value);
void clamp_at_deadzone(int16_t & val);
/********************************************************************
| Routine Name: print_gimbal_pos
| File:         remote_firmware.ino
|
| Description: Print gimbal values to serial monitor
|
| Parameter Descriptions:
| name               description
| ------------------ -----------------------------------------------
| l_g_v (Vector2)    left gimbal values
| r_g_v (Vector2)    right gimbal values
********************************************************************/
void print_gimbal_pos(Vector2 l_g_v, Vector2 r_g_v);
void print_val(int val);
/********************************************************************
| Routine Name: display_gimbal_pos
| File:         remote_firmware.ino
|
| Description: Display gimbal values on LCD screen
|
| Parameter Descriptions:
| name               description
| ------------------ -----------------------------------------------
| l_g_v (Vector2)    left gimbal values
| r_g_v (Vector2)    right gimbal values
********************************************************************/
void display_gimbal_pos(Vector2 l_g_v, Vector2 r_g_v);
#endif