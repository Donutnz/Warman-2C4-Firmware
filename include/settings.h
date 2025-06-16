#pragma once

// Settings about the bot

// Wheel OD for calculating move dist from rotation (mm)
#define WHEEL_OD 110

// Lateral distance between wheel centres. (mm)
#define TRACK_WIDTH 380

// Distance from turning centre to front edge of scoop. (mm)
#define SCOOP_LENGTH 300

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
#define ARM_OPEN 100

// Arms closed
#define ARM_CLOSED 0


/*
Error Flags
*/

// Value Error
#define VALUE_ERROR -1