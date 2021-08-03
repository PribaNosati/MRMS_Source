#include <mrm-8x8a.h>
#include <mrm-bldc2x50.h>
#include <mrm-imu.h>
#include <mrm-lid-can-b2.h>
//#include <mrm-ir-finder2.h>
#include <mrm-ir-finder3.h>
#include <mrm-mot2x50.h>
#include <mrm-mot4x10.h>
#include <mrm-mot4x3.6can.h>
#include "mrm-robot-soccer.h"

/** Constructor
@param name - it is also used for Bluetooth so a Bluetooth client (like a phone) will list the device using this name.
*/
RobotSoccer::RobotSoccer(char name[]) : Robot(name) {
	motorGroup = new MotorGroupStar(this, mrm_mot4x3_6can, 0, mrm_mot4x3_6can, 1, mrm_mot4x3_6can, 2, mrm_mot4x3_6can, 3);

	pidXY = new Mrm_pid(10.0, 100, 0); // PID controller, regulates motors' speeds for linear motion in the x-y plane
	pidRotation = new Mrm_pid(10.0, 100, 0); // PID controller, regulates rotation around z axis
	actionPlay = new ActionSoccerPlay(this);
	actionBounce = new ActionSoccerBounce(this);
	actionCalibrate = new ActionSoccerCalibrate(this);
	actionCatch = new ActionSoccerCatch(this);
	actionIdle = new ActionSoccerIdle(this);
	actionLineAvoid = new ActionSoccerLineAvoid(this);

	// The actions that should be displayed in menus must be added to menu-callable actions. You can use action-objects defined
	// right above, or can create new objects. In the latter case, the inline-created objects will have no pointer and cannot be
	// called in the code, but only through menus. For example, ActionWallsTest test is only manu-based, and it is all right.
	// This test is not supposed to be called in code.
	actionAdd(actionBounce);
	actionAdd(actionCalibrate);
	actionAdd(actionPlay);

	// mrm_mot4x3_6can->directionChange(0); // Uncomment to change 1st wheel's rotation direction
	// mrm_mot4x3_6can->directionChange(1); // Uncomment to change 2nd wheel's rotation direction
	// mrm_mot4x3_6can->directionChange(2); // Uncomment to change 3rd wheel's rotation direction
	// mrm_mot4x3_6can->directionChange(3); // Uncomment to change 4th wheel's rotation direction

	mrm_8x8a->actionSet(_actionMenuMain, 0); // Button 0 stops the robot and prints main manu
	mrm_8x8a->actionSet(actionPlay, 1); // Button 1 starts the play
	mrm_8x8a->actionSet(_actionLoop, 2); // Button 2 starts user defined loop() function
	mrm_8x8a->actionSet(actionBounce, 3); // Button 3 starts user defined bounce() function
}

/** Rear distance to wall
@return - in mm
*/
uint16_t RobotSoccer::back() {
	return mrm_lid_can_b2->reading(0, 2); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Ball's direction
@return - robot's front is 0�, positive angles clockwise, negative anti-clockwise. Back of the robot is 180�.
*/
int16_t RobotSoccer::ballAngle() {
	return mrm_ir_finder3->angle();
}

/** Read barrier
@return - true if interrupted
*/
bool RobotSoccer::barrier() {
	return analogRead(35) < 300; // Adjust this value
}

/** Store bitmaps in mrm-led8x8a.
*/
void RobotSoccer::bitmapsSet() {
	uint8_t red[8] = { 0b00000000, 0b01100110, 0b11111111, 0b11111111, 0b11111111, 0b01111110, 0b00111100, 0b00011000 };
	uint8_t green[8] = { 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000 };
	mrm_8x8a->bitmapCustomStore(red, green, 7);
}

/** Bouncing off the lines
*/
void RobotSoccer::bounce(){
	static int directionCurrent = 45;
	const int VARIATION = 40;
	const int SPEED = 60;
	if (setup()){
		headingToMaintain = heading();
	}
	go(SPEED, directionCurrent, pidRotation->calculate(heading() - headingToMaintain));
	if (left() < 200){
		directionCurrent = 90 + (2 * (rand() % VARIATION) - VARIATION);
		// print("Left %i\n\r", directionCurrent);
	}
	if (right() < 200){
		directionCurrent = -90 + (2 * (rand() % VARIATION) - VARIATION);
		// print("Right %i\n\r", directionCurrent);
	}
	if (front() < 200){
		directionCurrent = 180 + (2 * (rand() % VARIATION) - VARIATION);
		// print("Front %i\n\r", directionCurrent);
	}
	if (back() < 200){
		directionCurrent = 0 + (2 * (rand() % VARIATION) - VARIATION);
		// print("Back %i\n\r", directionCurrent);
	}
	// for (uint8_t i = 0; i < 4; i++)

	static uint32_t ms = 0;
	if (millis() - ms > 1000){
		print("Target: %i, current: %i\n\r", (int)headingToMaintain, (int)heading());
		ms = millis();
	}
}

/** Reads push button switch
@number - 0 to 3, push button's ordinal number
@return - true if pressed
*/
bool RobotSoccer::button(uint8_t number) {
	return mrm_8x8a->switchRead(number);
}

/** Line sensor - brightness of the surface
@param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - brightness as an analog value.
*/
uint16_t RobotSoccer::brightness(uint8_t transistorNumber, uint8_t deviceNumber) {
	return mrm_ref_can->reading(transistorNumber, deviceNumber);
}

/** Calibrate all line sensors
 */
void RobotSoccer::calibrate(){
	go(0, 0, 25, 100);
	mrm_ref_can->calibrate(0);
	mrm_ref_can->calibrate(1);
	mrm_ref_can->calibrate(2);
	mrm_ref_can->calibrate(3);
	go(0, 0, 0, 0);
	end();
}

/** Go around the ball and approach it.
*/
void RobotSoccer::catchBall() {
	if (lineAny())
		actionSet(actionLineAvoid);
	else if (mrm_ir_finder3->distance() > 100) {
		go(30, mrm_ir_finder3->angle(), pidRotation->calculate(headingToMaintain - heading()), 100);
	}
	else
		actionSet(actionIdle);
}

/** Front distance to wall
@return - in mm
*/
uint16_t RobotSoccer::front() {
	return mrm_lid_can_b2->reading(0); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Control of a robot with axles connected in a star formation, like in a RCJ soccer robot with omni wheels. Motor 0 is at 45 degrees, 1 at 135, 2 at -135, 3 at -45.
@param speed - 0 to 100.
@param angleDegrees - Movement direction in a robot's coordinate system, in degrees. 0 degree is the front of the robot and positive angles are to the right.
Values between -180 and 180.
@param rotation - Rotation speed (around the vertical axis), -100 to 100. Positive numbers turn the robot to the right. It makes sense to use smaller
numbers because a value 100 turns on all the motors at maximal speed.
@param speedLimit - Speed limit, 0 to 127. For example, 80 will limit all the speeds to 80/127%. 0 will turn the motors off.
*/
void RobotSoccer::go(float speed, float angleDegrees, float rotation, uint8_t speedLimit) {
	motorGroup->go(speed, angleDegrees, rotation, speedLimit);
}

/** Test - go straight ahead.
*/
void RobotSoccer::goAhead() {
	const uint8_t speed = 60;
	motorGroup->go(speed);
	end();
}

/**Compass
@return - North is 0�, clockwise are positive angles, values 0 - 360.
*/
float RobotSoccer::heading() {
	return mrm_imu->heading();
}

/** No ball detected - return to Your goal.
*/
void RobotSoccer::idle() {
	// if (setup())
	// 	headingToMaintain = mrm_imu->heading();
	if (false && lineAny())
		actionSet(actionLineAvoid);
	else if (mrm_ir_finder3->distance() > 100)
		actionSet(actionCatch);
	else {
		//print("\n\rError: %i = %i - 300\n\r", (int)(300 - robot->mrm_lid_can_b2->reading(3)), robot->mrm_lid_can_b2->reading(3));
		float errorL = 900 - left();
		float errorR = right() - 900;
		motorGroup->goToEliminateErrors(errorL > errorR ? errorL : errorR, 0/*200 - back()*/, heading() - headingToMaintain, pidXY, pidRotation, true);
		//delay(500);
	}
}

/** Left distance to wall
@return - in mm
*/
uint16_t RobotSoccer::left() {
	return mrm_lid_can_b2->reading(0, 3); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Line sensor
@param transistorNumber - starts from 0 and end value depends on sensor. Usually 7 (for mrm-ref-can8) or 8 (for mrm-ref-can9).
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - true if white line found
*/
bool RobotSoccer::line(uint8_t transistorNumber, uint8_t deviceNumber) {
	return !mrm_ref_can->dark(transistorNumber, deviceNumber);
}

bool RobotSoccer::lineAny(){
	for (uint8_t i = 0; i < 4; i++)
		if (mrm_ref_can->any(false, i))
			return true;
	return false;
}

void RobotSoccer::lineAvoid(){
	static uint8_t inboundDirection[4];
	static int8_t lastInbound = -1;
	static int8_t escapeDirection;
	if (setup()){
		for (uint8_t i = 0; i < 4; i++)
			if (mrm_ref_can->any(false, i))
				inboundDirection[++lastInbound] = i;

		int8_t x = 0;
		int8_t y = 0;
		for (uint8_t i = 0; i < lastInbound; i++)
			switch(inboundDirection[i]){
				case 0:
					y++;
					break;
				case 1:
					x++;
					break;
				case 2:
					y--;
					break;
				case 3:
					x--;
					break;
			}
		escapeDirection = atan2(x, y) / PI * 180;
	}
	go(30, escapeDirection, headingToMaintain - mrm_imu->heading(), 100);
	if (!lineAny())
		actionSet(actionIdle);
}

/** Custom test.
*/
void RobotSoccer::loop() {
	motorGroup->go(100, 0);
	//static int initialDirection;
	//if (setup()) {
	//	initialDirection = mrm_imu->heading();
	//}

	//int rotationalError = mrm_imu->heading() - initialDirection;

	//int positionError = 0;
	//if (mrm_lid_can_b2->reading(1) > 80)
	//	positionError = 91 - mrm_lid_can_b2->reading(1);
	//else if (mrm_lid_can_b2->reading(3) > 80)
	//	positionError = mrm_lid_can_b2->reading(1) - 91;

	//motorGroup->go(abs(positionError), positionError < 0 ? -90 : 90, rotationalError);
}

/** Starts robot
*/
void RobotSoccer::play() {
	if (motorGroup == NULL) {
		print("Define robot->motorGroupStar first.\n\r");
		return;
	}
	headingToMaintain = mrm_imu->heading();
	actionSet(actionIdle);
}

/**Pitch
@return - Pitch in degrees. Inclination forwards or backwards. Leveled robot shows 0�.
*/
float RobotSoccer::pitch() {
	return mrm_imu->pitch();
}

/** Right distance to wall
@return - in mm
*/
uint16_t RobotSoccer::right() {
	return mrm_lid_can_b2->reading(0, 1); // Correct all sensors so that they return the same value for the same physical distance.
}

/** Roll
@return - Roll in degrees. Inclination to the left or right. Values -90 - 90. Leveled robot shows 0�.
*/
float RobotSoccer::roll() {
	return mrm_imu->roll();
}

/** Display fixed sign stored in sensor
@image - sign's number
*/
void RobotSoccer::sign(uint8_t number) {
	mrm_8x8a->bitmapDisplay(number);
}
