#pragma once

// Settings about the bot

//Enable to activate debug mode
//#define DEBUG_BOT

//Enable Serial Comms
//#define SERIAL_BOT

// Pivot turn helper
#define RIGHT_WHEEL 1
#define LEFT_WHEEL 0

// Wheel OD for calculating move dist from rotation (mm)
#define WHEEL_OD 116 //100mm diameter wheel with 4mm thick tyre thing.

// Lateral distance between wheel centres. (mm)
#define TRACK_WIDTH 362 //367

// Distance from turning centre to front edge of scoop. (mm)
#define SCOOP_LENGTH 270

// Distance between the axle and the tips of the spikes. (mm)
#define BOT_LENGTH 330

// Total width of bot. Wheel outside to wheel outside.
#define BOT_WIDTH 383

// Length of arms. (mm) 
#define ARM_LENGTH 290

// 23hs5628 step angle (degrees)
#define STEP_ANGLE 1.8

// 23hs5628 Speed (mm/s)
#define MOTOR_SPEED 480.0f //500.0f

// 23hs5628 Edge climb speed (mm/s)
#define CRAWL_SPEED 18.0f //18

// 23hs5628 Turning speed (mm/s)
#define TURN_SPEED 250.0f

// 23hs5628 Full send speed (mm/s)
#define SEND_SPEED 1500.0f

// 23hs5628 Default Acceleration (mm/s/s)
#define MOTOR_ACCEL 600.0f //1000

// 23hs5628 Full send accel (mm/s/s)
#define SEND_ACCEL 12000.0f

// Microstep multiplier. E.g. 1, 2, 4, 8, etc.
#define MS_STEPS 4 //8

// Rate to move arms (degs per second)
#define ARM_SPEED 120 //60

// Rate to move ramp (degs per second)
#define RAMP_SPEED 20 //40

// Tone frequency (hz)
#define TONE_FREQ 1000

// Frequency of ABS pulses. Roughly 5-15Hz range. (Hz)
#define ABS_FREQ 15

/*
Pre-determined Servo Positions (degrees, 0-180)
*/

// Ramp neutral angle (aka flat)
#define FROG_ANGLE_NEUTRAL 20 //0
#define FROG_ANGLE_LIFT 100 //90

// Arms open
#define ARMS_OPEN 50 //40

// Arms closed
#define ARMS_CLOSED 150 //150

// Special distances

// Arms open to collect balls
#define TRIGGER_ARMS_CLOSE_CAPTURE 530 //420

// Distance from arena west edge to axle at start. Set by positioning jig
#define INITIAL_POSITION_LAT 109.2