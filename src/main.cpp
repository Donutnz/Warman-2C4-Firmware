#include <Arduino.h>
#include <SpeedyStepper.h>
#include <ServoEasing.hpp>

#include "pins.h"
#include "settings.h"

/*
Simple fwd or back move. Relative.

@param distance distance to travel in mm.
*/
void driveRel(float distance);

/*
Absolute fwd and back movement.

@param distance Distance to move in mm.
*/
void driveAbs(float distance);

/*
Rotate around centre.

Both wheels driven, one fwd and one back.

@param targetAngle Angle in degrees. Positive for CW, negative for CCW.
*/
void turnNeutral(float targetAngle);

/**
Pivot on one wheel.

One wheel braked, the other driven.

@param targetAngle The angle to rotate to
@param aroundRight Stationary wheel. Use RIGHT_WHEEL and LEFT_WHEEL I.e. right wheel is braked, left wheel is driven.
*/
void turnPivot(float targetAngle, int aroundRight);

/*
Move arms to angle.

@param armsAngle Set angle of grabber arms. 0-180 degrees.
*/
void moveArms(float armsAngle);

/*
Move arms to angle with actuation duration used to determine speed.

@param armsAngle Set angle of grabber arms. 0-180 degrees.
@param actTime Time in ms for actuation to complete. 
*/
void moveArms(float armsAngle, float actTime);

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

/*
Reset current stepper position to 0. Does not move steppers.
*/
void resetPosition();

/*
Set speed of drive movements in mm/s

@param groundSpeed speed in mm/s
*/
void setGroundSpeed(float groundSpeed);

/*
Set speed of drive movements in mm/s

@param accelRate acceleration in mm/s/s
*/
void setAccelRate(float accelRate);

/*
Activate Anti-Lock braking system.

@param actDuration How long the pulsing lasts.
*/
void fireABS(unsigned long actDuration);

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

	pinMode(TONE_PIN, OUTPUT);

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
	pinMode(STEPPER_R_EN, OUTPUT); //May not need this. Can leave floating or hardware disconnect.
	pinMode(STEPPER_L_EN, OUTPUT);
	
	// Enable the stepper motors
	digitalWrite(STEPPER_R_EN, LOW);
	digitalWrite(STEPPER_L_EN, LOW);

	wheelR.connectToPins(STEPPER_R_PULSE, STEPPER_R_DIR);
	wheelL.connectToPins(STEPPER_L_PULSE, STEPPER_L_DIR);

	float stepsPerMM = (360 / (STEP_ANGLE/MS_STEPS)) / (2 * PI * (WHEEL_OD / 2)); // steps per mm. Dbl check this maths.
	wheelR.setStepsPerMillimeter(stepsPerMM);
	wheelL.setStepsPerMillimeter(stepsPerMM);

	setGroundSpeed(MOTOR_SPEED);
	wheelR.setAccelerationInMillimetersPerSecondPerSecond(MOTOR_ACCEL*MS_STEPS);
	wheelL.setAccelerationInMillimetersPerSecondPerSecond(MOTOR_ACCEL*MS_STEPS);

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

	tone(TONE_PIN, TONE_FREQ, 250); //Ready beep

	//Wait for GO!
	awaitButton();

	tone(TONE_PIN, 500, 250); //About to go beep

	delay(1000); // 2sec delay to get hands away

	tone(TONE_PIN, TONE_FREQ, 250); //Go beep

	digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {

	// Handle opening and closing arms without stopping the steppers for ball capture.
	if(!isOneServoMoving()){
		float curPos=wheelL.getCurrentPositionInMillimeters();

		if(abs(abs(curPos) - TRIGGER_ARMS_CLOSE_CAPTURE) < 20){
			moveArms(ARMS_CLOSED);
		}
	}

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
		Serial.println(taskCounter);
#endif

		switch (taskCounter){
		case 0:{ //Open arms and drive to ball collection. Todo: close arms with "time to open" calculated from bot speed. Open just in time.
			float moveTime = 80 / (MOTOR_SPEED*MS_STEPS); //180mm is max dist that the bot can be towards balls before arms will hit balls //180

			//driveAbs(339.292); //360deg
			driveAbs(630); //650
			
			moveArms(ARMS_OPEN, moveTime); //Arms should be open 20mm before 
			break;
		}
		case 1: //Come back to align w drop box
			driveAbs(270); //270. 475 to smash pivot
			break;
		case 2: //Turn so rear faces ramp edge.
			setGroundSpeed(TURN_SPEED);
			turnNeutral(-90);
			break;
		case 3: //Scoot up to ramp
			setGroundSpeed(MOTOR_SPEED);
			driveRel(-350); //-350
			break;
		case 4: // Clamber onto seesaw and slowly climb up. Slide down and smash box.
			setGroundSpeed(CLIMB_SPEED);
			driveRel(-(760 + 390)); //Just over center 340
			break;
		case 5:
			tiltRamp(RAMP_ANGLE_DUMP);
			break;
/*
		case 6: // Come off box
			setGroundSpeed(ESCAPE_SPEED);
			driveRel(100);
			break;
		case 7: // Turn to be parallel w box.
			setGroundSpeed(TURN_SPEED);
			turnPivot(90, LEFT_WHEEL);
			break;
		case 8: // Drive past to clear box
			setGroundSpeed(MOTOR_SPEED);
			driveRel(300);
			break;
		case 9: // Turn to aim for end zone.
			setGroundSpeed(TURN_SPEED);
			turnPivot(90, LEFT_WHEEL);
			break;
		case 10: // Park.
			setGroundSpeed(MOTOR_SPEED);
			driveRel(100);
			break;
*/
		case 6: // Party. Celebratory LED flashes and beeping. 11
			while(1){
				digitalWrite(LED_BUILTIN, LOW);
				tone(TONE_PIN, TONE_FREQ);
				delay(250);
				digitalWrite(LED_BUILTIN, HIGH);
				noTone(TONE_PIN);
				delay(250);
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


// Function Defs

void driveRel(float distance){
#ifdef SERIAL_BOT
	Serial.print("Rel Drive: ");
	Serial.println(distance);
#endif

	wheelL.setupRelativeMoveInMillimeters(distance*-1);
	wheelR.setupRelativeMoveInMillimeters(distance);
}

void driveAbs(float distance){
#ifdef SERIAL_BOT
	Serial.print("Abs Drive: ");
	Serial.println(distance);
#endif

	wheelL.setupMoveInMillimeters(distance*-1);
	wheelR.setupMoveInMillimeters(distance);
}

void turnNeutral(float targetAngle){
#ifdef SERIAL_BOT
	Serial.print("Neutral Ang: ");
	Serial.println(targetAngle);
#endif

	wheelL.setupRelativeMoveInMillimeters(targetAngle*neutralMMperDeg);
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

void moveArms(float targetAngle){
	armR.setEaseTo(targetAngle);
	armL.setEaseTo(targetAngle);

	return;
}

void moveArms(float targetAngle, float actTime){
	float delta_Angle = abs(armR.getCurrentAngle() - targetAngle) * (PI/180); //Delta angle in radians
	uint_fast16_t angVelocity = (delta_Angle / actTime) * (180/PI); // Angular velocity in degrees per second
	
	armR.setEaseTo(targetAngle, angVelocity);
	armL.setEaseTo(targetAngle, angVelocity);

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

void resetPosition(){
	wheelR.setCurrentPositionInMillimeters(0);
	wheelL.setCurrentPositionInMillimeters(0);
}

void setGroundSpeed(float groundSpeed){
	wheelR.setSpeedInMillimetersPerSecond(groundSpeed*MS_STEPS);
	wheelL.setSpeedInMillimetersPerSecond(groundSpeed*MS_STEPS); 
}

void setAccelRate(float accelRate){
	wheelR.setAccelerationInMillimetersPerSecondPerSecond(accelRate*MS_STEPS);
	wheelL.setAccelerationInMillimetersPerSecondPerSecond(accelRate*MS_STEPS);
}

void fireABS(unsigned long actDuration){

	unsigned long endT = millis() + actDuration;

	while(millis() <= endT){
		digitalWrite(STEPPER_R_EN, HIGH);
		digitalWrite(STEPPER_L_EN, HIGH);
		delay(ABS_FREQ/2);
		digitalWrite(STEPPER_R_EN, LOW);
		digitalWrite(STEPPER_L_EN, LOW);
		delay(ABS_FREQ/2);
	}
}