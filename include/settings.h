#pragma once

// Settings about the bot

// Wheel OD for calculating move dist from rotation (mm)
#define WHEEL_OD 100

// Lateral distance between wheel centres. (mm)
#define TRACK_WIDTH 345

// Distance from turning centre to front edge of scoop. (mm)
#define SCOOP_LENGTH 310

// 23hs5628 step angle (degrees)
#define STEP_ANGLE 1.8

// 23hs5628 Max RPM
#define SPEED_LIMIT 1500


/*
Pre-determined Servo Positions (degrees, 0-180)
*/

// Ramp neutral angle (aka flat)
#define RAMP_ANGLE_NEUTRAL 90

// Arms open
#define ARMS_OPEN 100

// Arms closed
#define ARMS_CLOSED 0


/*
Error Flags
*/

// Value Error
#define VALUE_ERROR -1