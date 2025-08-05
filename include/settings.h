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
#define WHEEL_OD 100

// Lateral distance between wheel centres. (mm)
#define TRACK_WIDTH 345

// Distance from turning centre to front edge of scoop. (mm)
#define SCOOP_LENGTH 310

// 23hs5628 step angle (degrees)
#define STEP_ANGLE 1.8

// 23hs5628 Speed (mm/s)
#define MOTOR_SPEED 20.0f

// 23hs5628 Acceleration (mm/s/s)
#define MOTOR_ACCEL 20.0f

// Microstep multiplier. E.g. 1, 2, 4, 8, etc.
#define MICROSTEPS 2

// Rate to move arms (degs per second)
#define ARM_SPEED 25

// Rate to move ramp (degs per second)
#define RAMP_SPEED 12

/*
Pre-determined Servo Positions (degrees, 0-180)
*/

// Ramp neutral angle (aka flat)
#define RAMP_ANGLE_NEUTRAL 90
#define RAMP_ANGLE_DUMP 120

// Arms open
#define ARMS_OPEN 0

// Arms closed
#define ARMS_CLOSED 80