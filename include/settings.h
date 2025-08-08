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
#define WHEEL_OD 102

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

// 23hs5628 Acceleration (mm/s/s)
#define MOTOR_ACCEL 50.0f

// Microstep multiplier. E.g. 1, 2, 4, 8, etc.
#define MICROSTEPS 8

// Rate to move arms (degs per second)
#define ARM_SPEED 20

// Rate to move ramp (degs per second)
#define RAMP_SPEED 12

/*
Pre-determined Servo Positions (degrees, 0-180)
*/

// Ramp neutral angle (aka flat)
#define RAMP_ANGLE_NEUTRAL 140
#define RAMP_ANGLE_DUMP 0

// Arms open
#define ARMS_OPEN 150

// Arms closed
#define ARMS_CLOSED 20