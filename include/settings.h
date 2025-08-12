#pragma once

// Settings about the bot

//Enable to activate debug mode
#define DEBUG_BOT

//Enable Serial Comms
//#define SERIAL_BOT

// Pivot turn helper
#define RIGHT_WHEEL 1
#define LEFT_WHEEL 0

// Wheel OD for calculating move dist from rotation (mm)
#define WHEEL_OD 112 //100mm diameter wheel with 2mm thick tyre thing.

// Lateral distance between wheel centres. (mm)
#define TRACK_WIDTH 335

// Distance from turning centre to front edge of scoop. (mm)
#define SCOOP_LENGTH 270

// Length of arms. (mm) 
#define ARM_LENGTH 290

// 23hs5628 step angle (degrees)
#define STEP_ANGLE 1.8

// 23hs5628 Speed (mm/s)
#define MOTOR_SPEED 500.0f

// 23hs5628 Edge climb speed (mm/s)
#define CLIMB_SPEED 50.0f

// 23hs5628 Acceleration (mm/s/s)
#define MOTOR_ACCEL 200.0f

// Microstep multiplier. E.g. 1, 2, 4, 8, etc.
#define MICROSTEPS 8

// Rate to move arms (degs per second)
#define ARM_SPEED 50 //20 is safe speed

// Rate to move ramp (degs per second)
#define RAMP_SPEED 40

// Tone frequency (hz)
#define TONE_FREQ 1000

/*
Pre-determined Servo Positions (degrees, 0-180)
*/

// Ramp neutral angle (aka flat)
#define RAMP_ANGLE_NEUTRAL 110
#define RAMP_ANGLE_DUMP 0

// Arms open
#define ARMS_OPEN 150 //150

// Arms closed
#define ARMS_CLOSED 40 //20

// Special distances

// Arms open to collect balls
#define TRIGGER_ARMS_CLOSE_CAPTURE 450