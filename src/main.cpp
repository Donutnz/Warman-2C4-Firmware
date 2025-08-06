#include <Arduino.h>
#include <SpeedyStepper.h>
#include <ServoEasing.hpp>

#include "pins.h"
#include "settings.h"

/* TODO

- everything

Might end up doing a "moves are objects, iterate thru list of them" approach if moves get too complicated".
*/

/*
Simple fwd or back move.

@param distance distance to travel in mm.
*/
void moveLinear(float distance);

/*
Rotate around centre.

Both wheels driven, one fwd and one back.

@param targetAngle Angle in degrees. Positive for CW, negative for CCW.
*/
void turnNeutral(float targetAngle);

/**
Pivot on one wheel.

One wheel braked, the other driven.

@param finalAngle The angle to rotate to
@param turnRight Stationary wheel. Use RIGHT_WHEEL and LEFT_WHEEL I.e. right wheel is braked, left wheel is driven.
*/
void turnPivot(float targetAngle, int aroundRight);

/*
Move arms to angle.

@param armsAngle Set angle of grabber arms. 0-180 degrees.
*/
void moveArms(float armsAngle);

/*
Set ramp angle.

@param rampAngle Set ramp angle. 0-180 degrees. 
*/
void tiltRamp(float rampAngle);

/*
Wait for button to be pressed. Also handles ignoring button held.

Blocking
*/
void awaitButton();

// Declare Steppers
SpeedyStepper wheelR;
SpeedyStepper wheelL;

// Neutral mm per degree
float neutralMMperDeg;

// Pivot mm per degree. This is just double neutralMMperDeg but I thought seperating them could be helpful for tweaking.
float pivotMMperDeg;

// Arm Servos
ServoEasing armL;
ServoEasing armR;

// Ramp Servos
ServoEasing rampL;
ServoEasing rampR;

// Keeps track of current task.
int taskCounter;

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(RUN_SWITCH, INPUT_PULLUP);

	digitalWrite(LED_BUILTIN, LOW); //Start low

#ifdef SERIAL_BOT
	//Serial stuff
	Serial.begin(9600);

	while(!Serial){}

	Serial.println("Starting...");

	Serial.println("Init Servos...");
#endif

	// Init Servos
	armL.attach(SERVO_L_ARM, ARMS_CLOSED);
	armR.attach(SERVO_R_ARM, ARMS_CLOSED);

	armL.setSpeed(ARM_SPEED);
	armR.setSpeed(ARM_SPEED);

	rampL.attach(SERVO_RAMP_L, RAMP_ANGLE_NEUTRAL);
	rampR.attach(SERVO_RAMP_R, RAMP_ANGLE_NEUTRAL);

	rampL.setSpeed(RAMP_SPEED);
	rampR.setSpeed(RAMP_SPEED);

	armL.setReverseOperation(true); //!!! CHECK THESE BEFORE RUNNING !!!
	rampL.setReverseOperation(true);

	setEasingTypeForAllServos(EASE_QUADRATIC_IN_OUT); //Is this best curve?


#ifdef SERIAL_BOT
	Serial.println("Init steppers...");
#endif

	// Init Steppers
	pinMode(STEPPER_R_EN, OUTPUT); //May not need this. Leave floating? Disconnect?
	pinMode(STEPPER_L_EN, OUTPUT);
	digitalWrite(STEPPER_R_EN, LOW);
	digitalWrite(STEPPER_L_EN, LOW);

	pinMode(STEPPER_R_PULSE, OUTPUT);
	pinMode(STEPPER_R_DIR, OUTPUT);

	pinMode(STEPPER_L_PULSE, OUTPUT);
	pinMode(STEPPER_L_DIR, OUTPUT);

	digitalWrite(STEPPER_L_DIR, HIGH);

/*
	while(true){
		digitalWrite(STEPPER_L_PULSE, HIGH);
		delayMicroseconds(500);
		digitalWrite(STEPPER_L_PULSE, LOW);
		delayMicroseconds(500);
	}
*/

	wheelR.connectToPins(STEPPER_R_PULSE, STEPPER_R_DIR);
	wheelL.connectToPins(STEPPER_L_PULSE, STEPPER_L_DIR);

	float stepsPerMM = (360 / STEP_ANGLE) / (2 * PI * (WHEEL_OD / 2)); // steps per mm. Dbl check this maths.
	wheelR.setStepsPerMillimeter(stepsPerMM*MICROSTEPS);
	wheelL.setStepsPerMillimeter(stepsPerMM*MICROSTEPS);

	wheelR.setSpeedInMillimetersPerSecond(MOTOR_SPEED*MICROSTEPS); //Tweak this in testing.
	wheelL.setSpeedInMillimetersPerSecond(MOTOR_SPEED*MICROSTEPS); 
	wheelR.setAccelerationInMillimetersPerSecondPerSecond(MOTOR_ACCEL*MICROSTEPS);
	wheelL.setAccelerationInMillimetersPerSecondPerSecond(MOTOR_ACCEL*MICROSTEPS);

	//wheelR.setSpeedInStepsPerSecond(400);
	//wheelL.setSpeedInStepsPerSecond(400);
	//wheelR.setAccelerationInStepsPerSecondPerSecond(200);
	//wheelL.setAccelerationInStepsPerSecondPerSecond(200);

	//wheelR.moveRelativeInMillimeters(500);
	//wheelL.moveRelativeInMillimeters(500);

	neutralMMperDeg = (2 * PI * (TRACK_WIDTH / 2)) / 360;
	pivotMMperDeg = neutralMMperDeg * 2;

	// Init task counter
	taskCounter = 0;

#ifdef SERIAL_BOT
	Serial.println("Sorting out servos...");
#endif

	//Get into start position
	moveArms(ARMS_CLOSED);
	tiltRamp(RAMP_ANGLE_NEUTRAL);

	// Wait for servos to get to their start positions
	synchronizeAllServosStartAndWaitForAllServosToStop();

	// Set servo operation to be non-blocking
	synchronizeAllServosAndStartInterrupt(false);
	disableServoEasingInterrupt(); // Might need this not sure.

#ifdef SERIAL_BOT
	Serial.println("Ready to go...");
#endif

	//Wait for GO!
	awaitButton();

	digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
	/*
	Go through list of tasks to run. Position in list is handled by the task counter.

	Core of everything.
	*/

	if(wheelL.motionComplete() && wheelR.motionComplete() && !isOneServoMoving()){ //Make sure current task is complete before going to next one
		
#if defined(DEBUG_BOT)
		//Step thru tasks
		awaitButton();
#endif

#ifdef SERIAL_BOT
		Serial.print("Task Num: ");
		Serial.prinln(taskCounter);
#endif

		switch (taskCounter){
		case 0: //Open arms and drive to ball collection
			moveArms(ARMS_OPEN);
			moveLinear(450);
			break;
		case 1: //Close arms to capture balls
			moveArms(ARMS_CLOSED);
			delay(2000); //Let balls settle.
			break;
		case 2: //Turn so rear faces ramp edge.
			turnPivot(-90, LEFT_WHEEL); //Turn to be perpendicular to edge of ramp. Easier to climb the 16mm curb?
			break;
		case 3:
			moveLinear(-100); //Climb onto seesaw.
			break;
		case 4: //Drive over pivot and pause for seesaw to drop
			moveLinear(-700);
			delay(3000);
			break;
		case 5: //Slightly tweak position to avoid hitting dump zone edge on ramp dismount 
			turnNeutral(45);
			break;
		case 6:
			moveLinear(-200);
			break;
		case 7: //Rotate to be perpendicular to seesaw edge.
			turnNeutral(-45); 
		case 8: //Dismount seesaw
			moveLinear(-100);
			break;
		case 9: //Align to dump edge
			turnNeutral(-90);
			break;
		case 10: //Drive up to drop zone edge
			moveLinear(-300);
			break;
		case 11: //Dump balls
			tiltRamp(RAMP_ANGLE_DUMP);
			delay(3000);
			break;
		case 12: //Stow ramp and drive to end zone. Might set deccel value to 0 for a hard brake stop. Buzzer?
			tiltRamp(RAMP_ANGLE_NEUTRAL);
			wheelR.setAccelerationInMillimetersPerSecondPerSecond(10000); //Hard brake
			wheelL.setAccelerationInMillimetersPerSecondPerSecond(10000);
			moveLinear(400);
			break;
		case 13: //End. Celebratory LED flashes
			while(1){
				digitalWrite(LED_BUILTIN, HIGH);
				delay(500);
				digitalWrite(LED_BUILTIN, LOW);
				delay(500);
			}
			break;
		default:
			break;
		}

		taskCounter += 1; // Increment task
	}

	// Execute next update functions.
	updateAllServos();
	wheelL.processMovement();
	wheelR.processMovement();
}

void moveLinear(float distance){
	wheelL.setupRelativeMoveInMillimeters(distance*-1);
	wheelR.setupRelativeMoveInMillimeters(distance);
}

void turnNeutral(float targetAngle){
	wheelL.setupRelativeMoveInMillimeters((targetAngle*-1)*neutralMMperDeg);
	wheelR.setupRelativeMoveInMillimeters(targetAngle*neutralMMperDeg);
}

void turnPivot(float targetAngle, int aroundRight){
	if(aroundRight){
		wheelL.setupRelativeMoveInMillimeters((targetAngle*-1)*pivotMMperDeg);
	}
	else{
		wheelR.setupRelativeMoveInMillimeters(targetAngle*pivotMMperDeg);
	}
}

void moveArms(float armsAngle){
	armR.setEaseTo(armsAngle);
	armL.setEaseTo(armsAngle);

	return;
}

void tiltRamp(float rampAngle){
	rampL.setEaseTo(rampAngle);
	rampR.setEaseTo(rampAngle);

	return;
}

void awaitButton(){
#ifdef SERIAL_BOT
	Serial.println("Button start...");
#endif

	while(!digitalRead(RUN_SWITCH)){} //Wait for already pressed button to be released. Prevents fast fwd.

	unsigned long previousMillis = 0;
	int ledState = LOW;

	while(digitalRead(RUN_SWITCH)){ //Wait for button to be pressed again. And flash LED to show impatience.
		unsigned long currentMillis = millis();

		if(currentMillis - previousMillis >= 500){
			previousMillis = currentMillis;

			if(ledState == LOW){
				ledState = HIGH;
			}
			else{
				ledState = LOW;
			}

			digitalWrite(LED_BUILTIN, ledState);
		}
	}
	
	digitalWrite(LED_BUILTIN, LOW);

#ifdef SERIAL_BOT
	Serial.println("Button done");
#endif
}