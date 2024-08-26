#include "mrm-fet-can.h"
#include <mrm-robot.h>

std::vector<uint8_t>* commandIndexes_mrm_fet_can =  new std::vector<uint8_t>(); // C++ 17 enables static variables without global initialization, but no C++ 17 here
std::vector<String>* commandNames_mrm_fet_can =  new std::vector<String>();

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_fet_can::Mrm_fet_can(Robot* robot, uint8_t maxNumberOfBoards) : 
	MotorBoard(robot, 1, "FET", maxNumberOfBoards, ID_MRM_FET_CAN) {

	if (commandIndexes_mrm_fet_can->empty()){
		commandIndexes_mrm_fet_can->push_back(COMMAND_TURN_ON);
		commandNames_mrm_fet_can->push_back("Turn on");
		commandIndexes_mrm_fet_can->push_back(COMMAND_TURN_OFF);
		commandNames_mrm_fet_can->push_back("Turn off");
	}
}

Mrm_fet_can::~Mrm_fet_can()
{
}

/** Add a mrm-therm-b-can sensor
@param deviceName - device's name
*/
void Mrm_fet_can::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_FET0_IN;
		canOut = CAN_ID_FET0_OUT;
		break;
	case 1:
		canIn = CAN_ID_FET1_IN;
		canOut = CAN_ID_FET1_OUT;
		break;
	case 2:
		canIn = CAN_ID_FET2_IN;
		canOut = CAN_ID_FET2_OUT;
		break;
	case 3:
		canIn = CAN_ID_FET3_IN;
		canOut = CAN_ID_FET3_OUT;
		break;
	case 4:
		canIn = CAN_ID_FET4_IN;
		canOut = CAN_ID_FET4_OUT;
		break;
	case 5:
		canIn = CAN_ID_FET5_IN;
		canOut = CAN_ID_FET5_OUT;
		break;
	case 6:
		canIn = CAN_ID_FET6_IN;
		canOut = CAN_ID_FET6_OUT;
		break;
	case 7:
		canIn = CAN_ID_FET7_IN;
		canOut = CAN_ID_FET7_OUT;
		break;
	default:
		sprintf(errorMessage, "Too many %s: %i.", _boardsName, nextFree);
		return;
	}
	MotorBoard::add(deviceName, canIn, canOut);
}

/** Turn output on
@outputNumber - 0 or 1
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_fet_can::turnOn(uint8_t outputNumber, uint8_t deviceNumber) {
	if (outputNumber > 1) {
		sprintf(errorMessage, "%s %i not found.", _boardsName, nextFree);
		return;
	}
	canData[0] = COMMAND_TURN_ON;
	canData[1] = outputNumber;

	robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 2, canData);
}

/** Turn output off
@outputNumber - 0 or 1
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_fet_can::turnOff(uint8_t outputNumber, uint8_t deviceNumber) {
	if (outputNumber > 1) {
		sprintf(errorMessage, "%s %i not found.", _boardsName, nextFree);
		return;
	}
	canData[0] = COMMAND_TURN_OFF;
	canData[1] = outputNumber;

	robotContainer->mrm_can_bus->messageSend((*idIn)[deviceNumber], 2, canData);
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
@return - true if canId for this class
*/
bool Mrm_fet_can::messageDecode(uint32_t canId, uint8_t data[8], uint8_t length){
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++)
		if (isForMe(canId, deviceNumber)){
			if (!messageDecodeCommon(canId, data, deviceNumber)) {
				switch (data[0]) {
				default:
					print("Unknown command. ");
					messagePrint(canId, length, data, false);
					errorCode = 205;
					errorInDeviceNumber = deviceNumber;
				}
			}
			return true;
		}
	return false;
}


/**Test
*/
void Mrm_fet_can::test()
{
	static uint32_t lastMs = 0;
	static bool isOn = false;
	static uint8_t fet = 0;

	if ((millis() - lastMs > 200 && isOn) || (millis() - lastMs > 2000 && !isOn)) {
		// uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				isOn = !isOn;
				if (isOn)
					turnOn(fet);
				else
					turnOff(fet);

				print(isOn ? "On %i\n\r" : "Off %i\n\r", fet);
				if (!isOn)
					fet = 1 - fet;
			}
		}
		lastMs = millis();
	}
}

