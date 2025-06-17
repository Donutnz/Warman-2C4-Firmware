#include <Arduino.h>
#include <SpeedyStepper.h>
#include <Servo.h>

#include "pins.h"
#include "settings.h"

/* TODO

- everything

Might end up doing a "moves are objects, iterate thru list of them" approach if moves get too complicated".
*/

/*
Simple fwd or back move.

float distance = distance to travel in mm.
*/
void moveFwd(float distance);

/*
Rotate around centre.

Both wheels driven, one fwd and one back.

int targetAngle = angle in degrees. Positive for CW, negative for CCW.
*/
void turnNeutral(float targetAngle);

/*
Pivot on one wheel.

One wheel braked, the other driven.

int finalAngle = The angle to rotate to
bool turnRight = If True, pivot around rh wheel. I.e. right wheel is braked, left wheel is driven.
*/
void turnPivot(float targetAngle, bool aroundRight);

/*
Move arms to angle.

float armsAngle = Set angle of grabber arms. 0-180 degrees.
*/
void moveArms(float armsAngle);

/*
Set ramp angle.

float rampAngle = Set ramp angle. 0-180 degrees. 
*/
void tiltRamp(float rampAngle);

/*
Might do more indepth error handling. E.g. flash codes or attempt to connect to serial or something.
*/
void handleError();

// Declare Steppers
SpeedyStepper wheelR;
SpeedyStepper wheelL;

// Neutral mm per degree
float neutralMMperDeg;

// Pivot mm per degree. This is just double neutralMMperDeg but I thought seperating them could be helpful for tweaking.
float pivotMMperDeg;

// Arm Servos
Servo armL;
Servo armR;

// Ramp Servos
Servo rampL;
Servo rampR;

// Keeps track of current task.
int taskCounter;

// Set if something happens.
int errorFlag;

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(RUN_SWITCH, INPUT);

	// Init Servos
	armL.attach(SERVO_L_ARM);
	armR.attach(SERVO_R_ARM);

	rampL.attach(SERVO_RAMP_L);
	rampR.attach(SERVO_RAMP_R);

	// Init Steppers
	pinMode(STEPPER_R_EN, OUTPUT); //May not need this. Leave floating? Disconnect?
	pinMode(STEPPER_L_EN, OUTPUT);

	wheelR.connectToPins(STEPPER_R_PULSE, STEPPER_R_DIR);
	wheelL.connectToPins(STEPPER_L_PULSE, STEPPER_L_DIR);

	float stepsPerMM = (360 / STEP_ANGLE) / (2 * PI * (WHEEL_OD / 2)); // steps per mm. Dbl check this maths.
	wheelR.setStepsPerMillimeter(stepsPerMM);
	wheelL.setStepsPerMillimeter(stepsPerMM);

	wheelR.setSpeedInRevolutionsPerSecond(SPEED_LIMIT); //Tweak this in testing.
	wheelL.setSpeedInRevolutionsPerSecond(SPEED_LIMIT);

	neutralMMperDeg = (2 * PI * (TRACK_WIDTH / 2)) / 360;
	pivotMMperDeg = (2 * PI * TRACK_WIDTH) / 360;

	// Init task counter
	taskCounter = 0;

	// Make sure the go switch isn't turned on at start up. Prevents surprises.
	while(digitalRead(RUN_SWITCH)){
		digitalWrite(LED_BUILTIN, HIGH); 
		delay(2000);
		digitalWrite(LED_BUILTIN, LOW);
		delay(2000);
	}

	moveArms(ARMS_CLOSED);
	tiltRamp(RAMP_ANGLE_NEUTRAL);
}

void loop() {
	// Skip everything if run switch is off.
	if(digitalRead(RUN_SWITCH)){
		return;
	}

	// If error flag is set, crash out.
	if(errorFlag < 0){
		while(1){
			digitalWrite(LED_BUILTIN, HIGH);
			delay(3000);
			digitalWrite(LED_BUILTIN, LOW);
			delay(1000);
		}

		return;
	}

	/*
	Go through list of tasks to run. Position in list is handled by the task counter.

	Core of everything.
	*/
	if(wheelL.motionComplete() && wheelR.motionComplete()){
		taskCounter += 1; // Increment task

		switch (taskCounter){
		case 0:
			moveArms(ARMS_OPEN);
			moveFwd(450);
			break;
		case 1:
			moveArms(ARMS_CLOSED);
			delay(2000); //Let balls settle.
			break;
		case 2:
			turnPivot(-90, false); //Turn to be perpendicular to edge of ramp. Easier to climb the 16mm curb?
			break;
		case 3:
			moveFwd(-100); //Climb onto seesaw.
			break;
		case 4:
			turnPivot(45, true);
			break;
		case 5:
			moveFwd(-700); //Drive to just over pivot
			delay(3000); //Wait 3 seconds for seesaw to flip and stabilise.
			break;
		case 6:
			moveFwd(-300); //Move to be inline with Deposit Zone edge.
			break;
		case 7:
			turnNeutral(45); //Rotate to be perpendicular to deposit zone. Note: still on seesaw.
			break;
		case 8:
			moveFwd(100); //Drive up to edge of deposit zone.
			break;
		default:
			break;
		}
	}

	wheelL.processMovement();
	wheelR.processMovement();
}

void moveFwd(float distance){
	wheelL.setupRelativeMoveInMillimeters(distance);
	wheelR.setupRelativeMoveInMillimeters(distance);
}

void turnNeutral(float targetAngle){
	wheelL.setupRelativeMoveInMillimeters(targetAngle*neutralMMperDeg);
	wheelR.setupRelativeMoveInMillimeters((targetAngle*-1)*neutralMMperDeg);
}

void turnPivot(float targetAngle, bool aroundRight){
	if(aroundRight){
		wheelL.setupRelativeMoveInMillimeters(targetAngle*pivotMMperDeg);
	}
	else{
		wheelR.setupRelativeMoveInMillimeters(targetAngle*pivotMMperDeg);
	}
}

void moveArms(float armsAngle){
	// Prevent error
	if((armsAngle > 180) || (armsAngle < 0)){
		errorFlag = VALUE_ERROR; // Set error occured flag. 
		return;
	}

	armL.write(armsAngle);
	armR.write(armsAngle*-1); // Mirror angle

	return;
}

void tiltRamp(float rampAngle){
	// Prevent Error
	if((rampAngle > 180) || (rampAngle < 0)){
		errorFlag = VALUE_ERROR; // Set error occured flag. 
		return;
	}

	rampL.write(rampAngle);
	rampR.write(rampAngle*-1); // Mirror for the other side.
	return;
}