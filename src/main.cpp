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

@param targetAngle The angle to rotate to. Positive for CW, negative for CCW.
@param pivotWheel Stationary wheel. Use RIGHT_WHEEL and LEFT_WHEEL I.e. right wheel is braked, left wheel is driven.
*/
void turnPivot(float targetAngle, int pivotWheel);

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
Set lift angle for legs.

@param liftAngle Set ramp angle. 0-180 degrees. 
*/
void liftBot(float liftAngle);

/*
Wait for button to be pressed. Also handles ignoring button held.

Blocking
*/
void awaitButton();

/*
Reset current stepper position to 0. Does not move steppers.
*/
void setPosition();

/*
Set current position to a value. Does not move steppers.

@param newPos New position in mm
*/
void setPosition(float newPos);

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
	armL.attach(SERVO_L_ARM, 180 - ARMS_CLOSED); // Coz reverse opertion doesn't kick in on start up.
	armR.attach(SERVO_R_ARM, ARMS_CLOSED);
	
	armL.setSpeed(ARM_SPEED);
	armR.setSpeed(ARM_SPEED);
	
	rampL.attach(SERVO_RAMP_L, 180 - FROG_ANGLE_NEUTRAL);
	rampR.attach(SERVO_RAMP_R, FROG_ANGLE_NEUTRAL);
	
	rampL.setSpeed(RAMP_SPEED);
	rampR.setSpeed(RAMP_SPEED);
	
	armL.setReverseOperation(true); //!!! CHECK THIS BEFORE RUNNING !!!
	rampL.setReverseOperation(true); //!!! CHECK THIS BEFORE RUNNING !!!

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
	//moveArms(ARMS_CLOSED);
	//liftBot(FROG_ANGLE_NEUTRAL);

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

	delay(500); // 0.5sec delay to get hands away

	tone(TONE_PIN, TONE_FREQ, 250); //Go beep

	digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {

	// Position triggers. Used for moving arms out of sync with usual tasks.
	// Use target task + 1
	float curPos=wheelL.getCurrentPositionInMillimeters();

	if((abs(abs(curPos) - TRIGGER_ARMS_CLOSE_CAPTURE) < 10) && taskCounter == 1 && !isOneServoMoving()){ // Close arms to capture balls
		moveArms(ARMS_CLOSED);
	}
	
	if((abs(abs(curPos) - 180) < 10) && taskCounter == 9){ // Open arms to dump balls
		moveArms(ARMS_OPEN);
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
			float moveTime = 50 / (MOTOR_SPEED*MS_STEPS); //180mm is max dist that the bot can be towards balls before arms will hit balls //180
			
			// Set initial east-west position to make abs moves easier re ball pick up.
			setPosition(INITIAL_POSITION_LAT);
			
			setGroundSpeed(MOTOR_SPEED);
			//driveAbs(339.292); //360deg
			driveAbs(740); //630
			
			moveArms(ARMS_OPEN, moveTime); //Arms should be open 20mm before 
			break;
		}
		case 1: //Come back to align w drop box
			setGroundSpeed(MOTOR_SPEED);
			driveAbs(600); //Axle centered below ramp.
			break;
		case 2: //Turn so forks faces ramp edge.
			//Might need a better turn for this coz spikes long.
			setGroundSpeed(TURN_SPEED);
			turnNeutral(-90);
			break;
		case 3: //Slowly shove forks under ramp edge
			setPosition(250); //Reset position to center dist from arena edge. Allows absolute moves under ramp.
			setGroundSpeed(CRAWL_SPEED);
			driveAbs(420); //500 //Engage forks under ramp and begin lifting
			break;
		case 4: // Zoom under ramp 
			setGroundSpeed(MOTOR_SPEED);
			driveAbs(1500); //1345 // Drive up between ramp supports and overshoot with clearance for neutral turn.
			break;
		case 5: // Pivot right to face arena north-western corner ish.
			setGroundSpeed(TURN_SPEED);
			turnNeutral(21.39); // Angle from SolidWorks plan.
			break;
		case 6: // Pootle towards north-east corner of finish zone. 
			setGroundSpeed(MOTOR_SPEED);
			driveRel(826.94); //789.83 // Distance calc'ed from solidworks plan.
			break;
		case 7: // Face drop box
			setGroundSpeed(TURN_SPEED);
			turnNeutral(-111.39); // Angle from SolidWorks plan.
			liftBot(FROG_ANGLE_LIFT);
			break;
		case 8: // Stand up and advance on drop box.
			setPosition(); // Reset pos to 0. For arms open to dump timing.
			// Arms open on a position trigger.
			setGroundSpeed(MOTOR_SPEED);
			driveAbs(241); // Over lip of box.
			break;
		case 9: // Shoot backwards off the box edge and settle ur bits.
			delay(500); // Wait for balls to drop
			moveArms(ARMS_CLOSED);
			liftBot(FROG_ANGLE_NEUTRAL);
			setGroundSpeed(SEND_SPEED);
			setAccelRate(SEND_ACCEL);
			driveAbs(-10);
			break;
		default: // Party. Celebratory LED flashes and beeping. End of tasks or something went wrong.
			while(1){
				digitalWrite(LED_BUILTIN, LOW);
				tone(TONE_PIN, 3000);
				delay(150);
				digitalWrite(LED_BUILTIN, HIGH);
				noTone(TONE_PIN);
				delay(150);
			}
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

	wheelL.setupRelativeMoveInMillimeters(distance);
	wheelR.setupRelativeMoveInMillimeters(distance);
}

void driveAbs(float distance){
#ifdef SERIAL_BOT
	Serial.print("Abs Drive: ");
	Serial.println(distance);
#endif

	wheelL.setupMoveInMillimeters(distance);
	wheelR.setupMoveInMillimeters(distance);
}

void turnNeutral(float targetAngle){
#ifdef SERIAL_BOT
	Serial.print("Neutral Ang: ");
	Serial.println(targetAngle);
#endif

	wheelL.setupRelativeMoveInMillimeters((targetAngle * -1)*neutralMMperDeg);
	wheelR.setupRelativeMoveInMillimeters(targetAngle*neutralMMperDeg);
}

void turnPivot(float targetAngle, int pivotWheel){
	if(pivotWheel){
		wheelL.setupRelativeMoveInMillimeters(targetAngle*pivotMMperDeg);
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

void liftBot(float liftAngle){
	rampL.setEaseTo(liftAngle);
	rampR.setEaseTo(liftAngle);

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

void setPosition(){
	wheelR.setCurrentPositionInMillimeters(0);
	wheelL.setCurrentPositionInMillimeters(0);
}

void setPosition(float newPos){
	wheelR.setCurrentPositionInMillimeters(newPos);
	wheelL.setCurrentPositionInMillimeters(newPos);
}

void setGroundSpeed(float groundSpeed){
	wheelR.setSpeedInMillimetersPerSecond(groundSpeed*MS_STEPS);
	wheelL.setSpeedInMillimetersPerSecond(groundSpeed*MS_STEPS); 
}

void setAccelRate(float accelRate){
	wheelR.setAccelerationInMillimetersPerSecondPerSecond(accelRate*MS_STEPS);
	wheelL.setAccelerationInMillimetersPerSecondPerSecond(accelRate*MS_STEPS);
}