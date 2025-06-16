#pragma once

// Pin definitions
// Makes life easier

// Steppers
#define STEPPER_L_PULSE 8
#define STEPPER_L_DIR 7
#define STEPPER_L_EN 4

#define STEPPER_R_PULSE 2
#define STEPPER_R_DIR 1
#define STEPPER_R_EN 0

// Servos
#define SERVO_R_ARM 9
#define SERVO_L_ARM 10
#define SERVO_RAMP_L 6
#define SERVO_RAMP_R 5
#define SERVO_AUX 11


// Rx Inputs
#define RX_IN_0 A0
#define RX_IN_1 A1
#define RX_IN_2 A2
#define RX_IN_3 A3

// Other
#define RUN_SWITCH 12 //Active LOW

//Status LED -> LED_BUILTIN