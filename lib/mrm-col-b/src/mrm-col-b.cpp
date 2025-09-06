#include "mrm-col-b.h"
#include <mrm-common.h>
#include <mrm-robot.h>

std::map<int, std::string>* Mrm_col_b::commandNamesSpecific = NULL;

/** Constructor
@param robot - robot containing this board
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_col_b::Mrm_col_b(Robot* robot, uint8_t maxNumberOfBoards) : 
	SensorBoard(robot, 1, "Color", maxNumberOfBoards, ID_MRM_COL_B, MRM_COL_B_COLORS) {
	readings = new std::vector<uint16_t[MRM_COL_B_COLORS]>(maxNumberOfBoards);
	_hsv = new std::vector<bool>(maxNumberOfBoards);
	_hue = new std::vector<uint8_t>(maxNumberOfBoards);
	_saturation = new std::vector<uint8_t>(maxNumberOfBoards);
	_value = new std::vector<uint8_t>(maxNumberOfBoards);
	_patternByHSV = new std::vector<uint8_t>(maxNumberOfBoards);
	_patternBy8Colors = new std::vector<uint8_t>(maxNumberOfBoards);
	_patternRecognizedAtMs = new std::vector<uint32_t>(maxNumberOfBoards);

	if (commandNamesSpecific == NULL){
		commandNamesSpecific = new std::map<int, std::string>();
		commandNamesSpecific->insert({MRM_COL_B_SENDING_COLORS_1_TO_3, 	"Send 1-3"});
		commandNamesSpecific->insert({MRM_COL_B_SENDING_COLORS_4_TO_6, 	"Send 4-6"});
		commandNamesSpecific->insert({MRM_COL_B_SENDING_COLORS_7_TO_9, 	"Send 7-9"});
		commandNamesSpecific->insert({MRM_COL_B_SENDING_COLORS_10_TO_11,	"Send10-11"});
		commandNamesSpecific->insert({MRM_COL_B_ILLUMINATION_CURRENT, 	"Illu curr"});
		commandNamesSpecific->insert({MRM_COL_B_SWITCH_TO_HSV, 			"To HS"});
		commandNamesSpecific->insert({MRM_COL_B_SWITCH_TO_8_COLORS, 		"To 8 colo"});
		commandNamesSpecific->insert({MRM_COL_B_SENDING_HSV, 			"Send HSV"});
		commandNamesSpecific->insert({MRM_COL_B_INTEGRATION_TIME, 		"Inte time"});
		commandNamesSpecific->insert({MRM_COL_B_GAIN, 					"Gain"});
		commandNamesSpecific->insert({MRM_COL_B_PATTERN_RECORD, 			"Patt reco"});
		commandNamesSpecific->insert({MRM_COL_B_PATTERN_SENDING, 		"Patt send"});
		commandNamesSpecific->insert({MRM_COL_B_PATTERN_REQUEST, 		"Patt requ"});
		commandNamesSpecific->insert({MRM_COL_B_PATTERN_ERASE, 			"Patt eras"});
	}
}

Mrm_col_b::~Mrm_col_b()
{
}

/** Add a mrm-col-b sensor
@param deviceName - device's name
*/
void Mrm_col_b::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_COL_B0_IN;
		canOut = CAN_ID_COL_B0_OUT;
		break;
	case 1:
		canIn = CAN_ID_COL_B1_IN;
		canOut = CAN_ID_COL_B1_OUT;
		break;
	case 2:
		canIn = CAN_ID_COL_B2_IN;
		canOut = CAN_ID_COL_B2_OUT;
		break;
	case 3:
		canIn = CAN_ID_COL_B3_IN;
		canOut = CAN_ID_COL_B3_OUT;
		break;
	case 4:
		canIn = CAN_ID_COL_B4_IN;
		canOut = CAN_ID_COL_B4_OUT;
		break;
	case 5:
		canIn = CAN_ID_COL_B5_IN;
		canOut = CAN_ID_COL_B5_OUT;
		break;
	case 6:
		canIn = CAN_ID_COL_B6_IN;
		canOut = CAN_ID_COL_B6_OUT;
		break;
	case 7:
		canIn = CAN_ID_COL_B7_IN;
		canOut = CAN_ID_COL_B7_OUT;
		break;
	default:
		sprintf(errorMessage, "Too many %s: %i.", _boardsName, nextFree);
		return;
	}

	SensorBoard::add(deviceName, canIn, canOut);
}

/** Blue
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorBlue(uint8_t deviceNumber) { 
	if (colorsStarted(deviceNumber))
		return(*readings)[deviceNumber][2];
	else
		return 0;
}

/** Blue greenish
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorBlueGreenish(uint8_t deviceNumber){
	if (colorsStarted(deviceNumber))
		return(*readings)[deviceNumber][3];
	else
		return 0;
}

/** Blue violetish
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorBlueVioletish(uint8_t deviceNumber){
	if (colorsStarted(deviceNumber))
		return(*readings)[deviceNumber][1];
	else
		return 0;
}

/** Green
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorGreen(uint8_t deviceNumber) { 
	if (colorsStarted(deviceNumber))
		return(*readings)[deviceNumber][4];
	else
		return 0;
}

/** Near IR
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t  Mrm_col_b::colorNearIR(uint8_t deviceNumber){
	if (colorsStarted(deviceNumber))
		return(*readings)[deviceNumber][8];
	else
		return 0;
}

/** Orange
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorOrange(uint8_t deviceNumber) { 
	if (colorsStarted(deviceNumber))
		return(*readings)[deviceNumber][6];
	else
		return 0;
}

/** Red
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorRed(uint8_t deviceNumber) { 
	if (colorsStarted(deviceNumber))
		return(*readings)[deviceNumber][7];
	else
		return 0;
}

/** If 10-colors mode not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_col_b::colorsStarted(uint8_t deviceNumber) {
	if ((*_hsv)[deviceNumber] || millis() - devices[deviceNumber].lastReadingsMs > MRM_COL_B_INACTIVITY_ALLOWED_MS || devices[deviceNumber].lastReadingsMs == 0) {
		//print("Switch to 8 col. %i %i \n\r", (*_hsv)[deviceNumber], (*_last8ColorsMs)[deviceNumber]); 
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			switchTo8Colors(&devices[deviceNumber]);
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - devices[deviceNumber].lastReadingsMs < 100) {
					//print("6co confirmed\n\r");
					return true;
				}
				delayMs(1);
			}
		}
		sprintf(errorMessage, "%s %i dead.", _boardsName.c_str(), deviceNumber);
		return false;
	}
	else
		return true;
}

/** Violet
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorViolet(uint8_t deviceNumber) {
	if (colorsStarted(deviceNumber))
		return(*readings)[deviceNumber][0];
	else
		return 0;
}

/** White
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t  Mrm_col_b::colorWhite(uint8_t deviceNumber){
	if (colorsStarted(deviceNumber))
		return(*readings)[deviceNumber][9];
	else
		return 0;
}

/** Yellow
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorYellow(uint8_t deviceNumber) {
	if (colorsStarted(deviceNumber))
		return(*readings)[deviceNumber][5];
	else
		return 0;
}

std::string Mrm_col_b::commandName(uint8_t byte){
	auto it = commandNamesSpecific->find(byte);
	if (it == commandNamesSpecific->end())
		return "Warning: no command found for key " + (int)byte;
	else
		return it->second;//commandNamesSpecific->at(byte);
}

/** Set gain
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
@param gainValue:
	0	0.5x
	1	1x
 	2	2x
	3	4x
	4	8x
	5	16x
	6	32x
	7	64x
	8	128x
	9	256x (default)
	10	512x
*/
void Mrm_col_b::gain(Device * device, uint8_t gainValue) {
	if (device == nullptr)
		for (Device& dev : devices)
			gain(&dev, gainValue);
	else {
		canData[0] = MRM_COL_B_GAIN;
		canData[1] = gainValue;
		messageSend(canData, 2, device->number);
	}
}

/** If HSV not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_col_b::hsvStarted(uint8_t deviceNumber) {
	if (!(*_hsv)[deviceNumber] || millis() - devices[deviceNumber].lastReadingsMs > MRM_COL_B_INACTIVITY_ALLOWED_MS || devices[deviceNumber].lastReadingsMs == 0) {
		//print("Switch to HSV.\n\r"); 

		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			switchToHSV(&devices[deviceNumber]);
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - devices[deviceNumber].lastReadingsMs < 100) {
					//print("HSV confirmed\n\r"); 
					return true;
				}
				delayMs(1);
			}
		}
		sprintf(errorMessage, "%s %i dead.", _boardsName.c_str(), deviceNumber);
		return false;
	}
	else
		return true;
}

/** Hue
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - Hue
*/
uint8_t Mrm_col_b::hue(uint8_t deviceNumber) {
	if (hsvStarted(deviceNumber))
		return (*_hue)[deviceNumber];
	else
		return 0;
}

/** Set illumination intensity
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
@param current - 0 - 3
*/
void Mrm_col_b::illumination(Device* device, uint8_t current) {
	if (device == nullptr)
		for (Device& dev : devices)
			illumination(&dev, current);
	else {
		canData[0] = MRM_COL_B_ILLUMINATION_CURRENT;
		canData[1] = current;
		messageSend(canData, 2, device->number);
	}
}

/** Set integration time
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
@param time - sets the ATIME parameter for integration time from 0 to 255, integration time = (ATIME + 1) * (ASTEP + 1) * 2.78µS.
@param step - sets STEP.
*/
void Mrm_col_b::integrationTime(Device * device, uint8_t time, uint16_t step) {
	if (device == nullptr)
		for (Device& dev : devices)
			integrationTime(&dev, time, step);
	else {
		canData[0] = MRM_COL_B_INTEGRATION_TIME;
		canData[1] = time;
		canData[2] = step >> 8;
		canData[3] = step & 0xFF;
		messageSend(canData, 4, device->number);
	}
}

/** Read CAN Bus message into local variables
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
*/
bool Mrm_col_b::messageDecode(CANMessage message) {
	for (Device& device : devices)
		if (isForMe(message.id, device)) {
			if (!messageDecodeCommon(message, device)) {
				switch (message.data[0]) {
				case MRM_COL_B_PATTERN_SENDING:
					print("Sensor %i, pattern %i: %i/%i/%i (H/S/V)\n\r", device.number, message.data[1], message.data[2], message.data[3], message.data[4]);
					break;
				case COMMAND_SENSORS_MEASURE_SENDING:
					break;
				case MRM_COL_B_SENDING_COLORS_1_TO_3:
					(*readings)[device.number][0] = (message.data[1] << 8) | message.data[2]; // violet
					// print("Data1: %i %i %i\n\r",(int)message.data[0], (int)message.data[1], (int)message.data[2]);
					(*readings)[device.number][1] = (message.data[3] << 8) | message.data[4]; // blue violetish
					(*readings)[device.number][2] = (message.data[5] << 8) | message.data[6]; // blue
					device.lastReadingsMs = millis();
					break;
				case MRM_COL_B_SENDING_COLORS_4_TO_6:
					(*readings)[device.number][3] = (message.data[1] << 8) | message.data[2]; // blue greenish
					// print("Data2: %i %i %i\n\r", (int)message.data[0], (int)message.data[1], (int)message.data[2]);
					(*readings)[device.number][4] = (message.data[3] << 8) | message.data[4]; // green
					(*readings)[device.number][5] = (message.data[5] << 8) | message.data[6]; // yellow
					device.lastReadingsMs = millis();
					break;
				case MRM_COL_B_SENDING_COLORS_7_TO_9:
					// print("Data3: %i %i %i\n\r", (int)message.data[0], (int)message.data[1], (int)message.data[2]);
					(*readings)[device.number][6] = (message.data[1] << 8) | message.data[2]; // orange
					(*readings)[device.number][7] = (message.data[3] << 8) | message.data[4]; // red
					(*readings)[device.number][8] = (message.data[5] << 8) | message.data[6]; // near IR
					(*_patternByHSV)[device.number] = message.data[7] & 0xF; // pattern
					(*_patternBy8Colors)[device.number] = message.data[7] >> 4;
					device.lastReadingsMs = millis();
					break;
				case MRM_COL_B_SENDING_COLORS_10_TO_11:
					(*readings)[device.number][9] = (message.data[1] << 8) | message.data[2]; // clear (white)
					// print("Data4: %i %i %i %i\n\r", (int)message.data[0], (int)message.data[1], (int)message.data[2], (int)(*readings)[deviceNumber][9]);
					device.lastReadingsMs = millis();
					break;
				case MRM_COL_B_SENDING_HSV:
					(*_hue)[device.number] = (message.data[1] << 8) | message.data[2];
					(*_saturation)[device.number] = (message.data[3] << 8) | message.data[4];
					(*_value)[device.number] = (message.data[5] << 8) | message.data[6];
					(*_patternByHSV)[device.number] = message.data[7] & 0xF;
					(*_patternBy8Colors)[device.number] = message.data[7] >> 4;
					(*_patternRecognizedAtMs)[device.number] = millis();
					device.lastReadingsMs = millis();
					//print("RCV HSV%i\n\r", (*_lastHSVMs)[deviceNumber]);
					break;
				default:
					print("Unknown command. ");
					messagePrint(message, false);
					errorAdd(message.id, ERROR_COMMAND_UNKNOWN, false);
				}
			}
			return true;
		}
	return false;
}

/** Erase all patterns
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - in all sensors.
*/
void Mrm_col_b::patternErase(Device * device) {
	if (device == nullptr)
		for (Device& dev : devices)
			patternErase(&dev);
	else {
		canData[0] = MRM_COL_B_PATTERN_ERASE;
		messageSend(canData, 1, device->number);
	}
}

/** Print HSV patterns
*/
void Mrm_col_b::patternPrint() {
	for (Device& device: devices){
		canData[0] = MRM_COL_B_PATTERN_REQUEST;
		messageSend(canData, 1, device.number);
	}
}

/** Choose a pattern closest to the current 8 colors
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@raturn - patternNumber
*/
uint8_t Mrm_col_b::patternRecognizedBy8Colors(uint8_t deviceNumber) {
	if (hsvStarted(deviceNumber))
		return (*_patternBy8Colors)[deviceNumber];
	else
		return 0;
}


/** Choose a pattern closest to the current HSV values
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param includeValue - if true, HSV compared. If not, HS.
@raturn - patternNumber
*/
uint8_t Mrm_col_b::patternRecognizedByHSV(uint8_t deviceNumber) {
	if (hsvStarted(deviceNumber))
		return (*_patternByHSV)[deviceNumber];
	else
		return 0;
}


/** Record a HSV pattern
@param patternNumber - 0 - MRM_COL_B_PATTERN_COUNT-1
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_col_b::patternRecord(uint8_t patternNumber, uint8_t deviceNumber) {
	if (!_hsv || patternNumber >= MRM_COL_B_PATTERN_COUNT || deviceNumber >= nextFree) {
		strcpy(errorMessage, "Patt. err.");
		return;
	}
	canData[0] = MRM_COL_B_PATTERN_RECORD;
	canData[1] = patternNumber;
	messageSend(canData, 2, deviceNumber);
}

/** Record patterns manually
*/
void Mrm_col_b::patternsRecord() {
	// Select device
	uint8_t sensorsAlive = count();
	print("Enter sensor id [0..%i]: ", sensorsAlive - 1);
	uint16_t deviceNumber = serialReadNumber(8000, 500, nextFree - 1 <= 9, sensorsAlive - 1, true);
	if (deviceNumber == 0xFFFF) {
		print("Exit\n\r");
		return;
	}
	print("%i\n\r", deviceNumber);
	// Select pattern
	print("Enter pattern id [0..%i]: ", MRM_COL_B_PATTERN_COUNT - 1);
	uint16_t patternNumber = serialReadNumber(8000, 500, MRM_COL_B_PATTERN_COUNT - 1 <= 9, MRM_COL_B_PATTERN_COUNT - 1, true);
	if (patternNumber == 0xFFFF) {
		print("Exit\n\r");
		return;
	}
	print("%i\n\r", patternNumber);
	patternRecord(patternNumber, deviceNumber);
}

/** Analog readings
@param colorId - one of 10 colors
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_col_b::reading(uint8_t colorId, uint8_t deviceNumber) {
	if (deviceNumber >= nextFree || colorId >= MRM_COL_B_COLORS) {
		strcpy(errorMessage, "mrm-col-b doesn't exist");
		return 0;
	}
	return (*readings)[deviceNumber][colorId];
}

/** Print all readings in a line
*/
void Mrm_col_b::readingsPrint() {
	print("Colors:");
	for (Device& dev : devices) {
		for (uint8_t colorId = 0; colorId < MRM_COL_B_COLORS; colorId++)
			print(" %3i", (*readings)[dev.number][colorId]);
	}
}


/** Saturation
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - saturation
*/
uint8_t Mrm_col_b::saturation(uint8_t deviceNumber) {
	if (hsvStarted(deviceNumber))
		return (*_saturation)[deviceNumber];
	else
		return 0;
}


/** Instruction to sensor to switch to converting R, G, and B on board and return hue, saturation and value
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0. 0xFF - all sensors.
*/
void Mrm_col_b::switchToHSV(Device* device) {
	if (device == nullptr)
		for (Device& dev : devices)
			switchToHSV(&dev);
	else {
		canData[0] = MRM_COL_B_SWITCH_TO_HSV;
		messageSend(canData, 1, device->number);
		(*_hsv)[device->number] = true;
	}
}


/** Instruction to sensor to start returning 8 raw colors
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0. 0xFF - all sensors.
*/
void Mrm_col_b::switchTo8Colors(Device* device) {
	if (device == nullptr)
		for (Device& dev : devices)
			switchTo8Colors(&dev);
	else {
		canData[0] = MRM_COL_B_SWITCH_TO_8_COLORS;
		messageSend(canData, 1, device->number);
		(*_hsv)[device->number] = false;
	}
}


/**Test
@param hsv - if not, then 10 colors
*/
void Mrm_col_b::test(bool hsvSelect)
{
	static uint64_t lastMs = 0;
	if (millis() - lastMs > 5000){
		illumination(&devices[0], 16);
	}

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for(Device& device : devices) {
			if (device.alive) {
				if (pass++)
					print(" | ");
				if (hsvSelect)
					print("HSV:%3i/%3i/%3i HSV/col:%i/%i", hue(device.number), saturation(device.number), value(device.number), patternRecognizedByHSV(device.number), patternRecognizedBy8Colors(device.number));
				else
					print("Vi:%3i B1:%3i B2:%3i B3:%3i Gr:%3i Ye:%3i Or:%3i Re:%3i IR:%3i Wh:%3i", colorViolet(device.number), colorBlueVioletish(device.number), colorBlue(device.number),
						colorBlueGreenish(device.number),	colorGreen(device.number), colorYellow(device.number), colorOrange(device.number), colorRed(device.number), colorNearIR(device.number),
						colorWhite(device.number));
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}

/** Value
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - value
*/
uint8_t Mrm_col_b::value(uint8_t deviceNumber) {
	if (hsvStarted(deviceNumber))
		return (*_value)[deviceNumber];
	else 
		return 0;
}
