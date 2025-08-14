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
#define TRACK_WIDTH 373 //380

// Distance from turning centre to front edge of scoop. (mm)
#define SCOOP_LENGTH 270

// Length of arms. (mm) 
#define ARM_LENGTH 290

// 23hs5628 step angle (degrees)
#define STEP_ANGLE 1.8

// 23hs5628 Speed (mm/s)
#define MOTOR_SPEED 500.0f //500.0f

// 23hs5628 Edge climb speed (mm/s)
#define CLIMB_SPEED 18.0f //20

// 23hs5628 Turning speed (mm/s)
#define TURN_SPEED 250.0f

// 23hs5628 Full send speed (mm/s)
#define SEND_SPEED 1500.0f

// 23hs5628 Acceleration (mm/s/s)
#define MOTOR_ACCEL 600.0f //1000

// Microstep multiplier. E.g. 1, 2, 4, 8, etc.
#define MS_STEPS 4 //8

// Rate to move arms (degs per second)
#define ARM_SPEED 70 //60

// Rate to move ramp (degs per second)
#define RAMP_SPEED 40

// Tone frequency (hz)
#define TONE_FREQ 1000

/*
Pre-determined Servo Positions (degrees, 0-180)
*/

// Ramp neutral angle (aka flat)
#define RAMP_ANGLE_NEUTRAL 110
#define RAMP_ANGLE_DUMP 5

// Arms open
#define ARMS_OPEN 150 //150

// Arms closed
#define ARMS_CLOSED 40 //20

// Special distances

// Arms open to collect balls
#define TRIGGER_ARMS_CLOSE_CAPTURE 450