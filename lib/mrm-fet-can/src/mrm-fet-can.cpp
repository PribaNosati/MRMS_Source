#include "mrm-fet-can.h"
#include <mrm-robot.h>

std::map<int, std::string>* Mrm_fet_can::commandNamesSpecific = NULL;

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_fet_can::Mrm_fet_can(uint8_t maxNumberOfBoards) : 
	MotorBoard(1, "FET", maxNumberOfBoards, ID_MRM_FET_CAN) {

	if (commandNamesSpecific == NULL){
		commandNamesSpecific = new std::map<int, std::string>();
		commandNamesSpecific->insert({COMMAND_TURN_ON, 	"Turn on"});
		commandNamesSpecific->insert({COMMAND_TURN_OFF, 	"Turn off"});
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
		sprintf(errorMessage, "Too many %s: %i.", _boardsName.c_str(), nextFree);
		return;
	}
	MotorBoard::add(deviceName, canIn, canOut);
}

std::string Mrm_fet_can::commandName(uint8_t byte){
	auto it = commandNamesSpecific->find(byte);
	if (it == commandNamesSpecific->end())
		return "Warning: no command found for key " + (int)byte;
	else
		return it->second;//commandNamesSpecific->at(byte);
}

/** Turn output on
@outputNumber - 0 or 1
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_fet_can::turnOn(uint8_t outputNumber, uint8_t deviceNumber) {
	if (outputNumber > 1) {
		sprintf(errorMessage, "%s %i not found.", _boardsName.c_str(), nextFree);
		return;
	}
	canData[0] = COMMAND_TURN_ON;
	canData[1] = outputNumber;

	messageSend(canData, 2, deviceNumber);
}

/** Turn output off
@outputNumber - 0 or 1
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_fet_can::turnOff(uint8_t outputNumber, uint8_t deviceNumber) {
	if (outputNumber > 1) {
		sprintf(errorMessage, "%s %i not found.", _boardsName.c_str(), nextFree);
		return;
	}
	canData[0] = COMMAND_TURN_OFF;
	canData[1] = outputNumber;

	messageSend(canData, 2, deviceNumber);
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
@return - true if canId for this class
*/
bool Mrm_fet_can::messageDecode(CANMessage& message) {
	for (Device& device : devices)
		if (isForMe(message.id, device)) {
			if (!messageDecodeCommon(message, device)) {
				switch (message.data[0]) {
				default:
					errorAdd(message, ERROR_COMMAND_UNKNOWN, false, true);
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
		for (Device& device: devices) {
			if (device.alive) {
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

