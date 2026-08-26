#include "mrm-col-b.h"
#include <mrm-common.h>
#include <mrm-robot.h>

std::map<int, std::string>* Mrm_col_b::commandNamesSpecific = NULL;

/** Constructor
@param robot - robot containing this board
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_col_b::Mrm_col_b(uint8_t maxNumberOfBoards) : 
	SensorBoard(1, "Color", maxNumberOfBoards, ID_MRM_COL_B, MRM_COL_B_COLORS) {
	readings = new std::vector<uint16_t[MRM_COL_B_COLORS]>(maxNumberOfBoards);
	// _hsv = new std::vector<bool>(maxNumberOfBoards);
	// _hue = new std::vector<uint8_t>(maxNumberOfBoards);
	// _saturation = new std::vector<uint8_t>(maxNumberOfBoards);
	// _value = new std::vector<uint8_t>(maxNumberOfBoards);
	// _patternByHSV = new std::vector<uint8_t>(maxNumberOfBoards);
	// _patternBy8Colors = new std::vector<uint8_t>(maxNumberOfBoards);
	// _patternRecognizedAtMs = new std::vector<uint32_t>(maxNumberOfBoards);
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

/** Violet
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorViolet(uint8_t deviceNumber) { 
	if (started(devices[deviceNumber]))
		return(*readings)[deviceNumber][0];
	else
		return 0;
}

/** Violet / deep blue
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorVioletDeepBlue(uint8_t deviceNumber){
	if (started(devices[deviceNumber]))
		return(*readings)[deviceNumber][1];
	else
		return 0;
}

/** Broad blue
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorBroadBlue(uint8_t deviceNumber){
	if (started(devices[deviceNumber]))
		return(*readings)[deviceNumber][2];
	else
		return 0;
}

/** Blue
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorBlue(uint8_t deviceNumber) { 
	if (started(devices[deviceNumber]	))
		return(*readings)[deviceNumber][3];
	else
		return 0;
}

/** Green 1
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t  Mrm_col_b::colorGreen1(uint8_t deviceNumber){
	if (started(devices[deviceNumber]))
		return(*readings)[deviceNumber][4];
	else
		return 0;
}

/** Broad green / yellow
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorBroadGreenYellow(uint8_t deviceNumber) { 
	if (started(devices[deviceNumber]))
		return(*readings)[deviceNumber][5];
	else
		return 0;
}

/** Green 2
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorGreen2(uint8_t deviceNumber) { 
	if (started(devices[deviceNumber]))
		return(*readings)[deviceNumber][6];
	else
		return 0;
}

/** Broad yellow / orange
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorBroadYellowOrange(uint8_t deviceNumber) {
	if (started(devices[deviceNumber]))
		return(*readings)[deviceNumber][7];
	else
		return 0;
}

/** Red
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t  Mrm_col_b::colorRed(uint8_t deviceNumber){
	if (started(devices[deviceNumber]))
		return(*readings)[deviceNumber][8];
	else
		return 0;
}

/** Deep red
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorDeepRed(uint8_t deviceNumber) {
	if (started(devices[deviceNumber]	))
		return(*readings)[deviceNumber][9];
	else
		return 0;
}

/** Far red
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorFarRed(uint8_t deviceNumber) {
	if (started(devices[deviceNumber]	))
		return(*readings)[deviceNumber][10];
	else
		return 0;
}

/** Near IR
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorNearIR(uint8_t deviceNumber) {
	if (started(devices[deviceNumber]	))
		return(*readings)[deviceNumber][11];
	else
		return 0;
}

/** Flicker detection
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorFlicker(uint8_t deviceNumber) {
	if (started(devices[deviceNumber]	))
		return(*readings)[deviceNumber][12];
	else
		return 0;
}

/** Clear - non-filtered - white
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorClear(uint8_t deviceNumber) {
	if (started(devices[deviceNumber]	))
		return(*readings)[deviceNumber][13];
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
	9	256x
	10	512x
	11	1024×
	12	2048×
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

// /** Set integration time
// @param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
// @param time - sets the ATIME parameter for integration time from 0 to 255, integration time = (ATIME + 1) * (ASTEP + 1) * 2.78µS.
// @param step - sets STEP.
// */
// void Mrm_col_b::integrationTime(Device * device, uint8_t time, uint16_t step) {
// 	if (device == nullptr)
// 		for (Device& dev : devices)
// 			integrationTime(&dev, time, step);
// 	else {
// 		canData[0] = MRM_COL_B_INTEGRATION_TIME;
// 		canData[1] = time;
// 		canData[2] = step >> 8;
// 		canData[3] = step & 0xFF;
// 		messageSend(canData, 4, device->number);
// 	}
// }

/** Read CAN Bus message into local variables
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
*/
bool Mrm_col_b::messageDecode(CANMessage& message) {
	for (Device& device : devices)
		if (isForMe(message.id, device)) {
			if (!messageDecodeCommon(message, device)) {
				switch (message.data[0]) {

				case COMMAND_SENSORS_MEASURE_SENDING:
					break;
				case MRM_COL_B_SENDING_COLORS_1_TO_3:
					(*readings)[device.number][0] = (message.data[1] << 8) | message.data[2]; 
					// print("Data1: %i %i %i",(int)message.data[0], (int)message.data[1], (int)message.data[2]);
					(*readings)[device.number][1] = (message.data[3] << 8) | message.data[4]; 
					(*readings)[device.number][2] = (message.data[5] << 8) | message.data[6]; 
					device.lastReadingsMs = millis();
					break;
				case MRM_COL_B_SENDING_COLORS_4_TO_6:
					(*readings)[device.number][3] = (message.data[1] << 8) | message.data[2]; 
					// print("Data2: %i %i %i", (int)message.data[0], (int)message.data[1], (int)message.data[2]);
					(*readings)[device.number][4] = (message.data[3] << 8) | message.data[4]; 
					(*readings)[device.number][5] = (message.data[5] << 8) | message.data[6]; 
					device.lastReadingsMs = millis();
					break;
				case MRM_COL_B_SENDING_COLORS_7_TO_9:
					// print("Data3: %i %i %i", (int)message.data[0], (int)message.data[1], (int)message.data[2]);
					(*readings)[device.number][6] = (message.data[1] << 8) | message.data[2]; 
					(*readings)[device.number][7] = (message.data[3] << 8) | message.data[4]; 
					(*readings)[device.number][8] = (message.data[5] << 8) | message.data[6]; 

					device.lastReadingsMs = millis();
					break;
				case MRM_COL_B_SENDING_COLORS_10_TO_12:
					(*readings)[device.number][9] = (message.data[1] << 8) | message.data[2]; 
					(*readings)[device.number][10] = (message.data[3] << 8) | message.data[4]; 
					(*readings)[device.number][11] = (message.data[5] << 8) | message.data[6]; 
					// print("Data4: %i %i %i %i", (int)message.data[0], (int)message.data[1], (int)message.data[2], (int)(*readings)[device.number][9]);
					device.lastReadingsMs = millis();
					break;
				case MRM_COL_B_SENDING_COLORS_13_TO_14:
					(*readings)[device.number][12] = (message.data[1] << 8) | message.data[2]; 
					(*readings)[device.number][13] = (message.data[3] << 8) | message.data[4]; 
					// print("Data5: %i %i %i %i", (int)message.data[0], (int)message.data[1], (int)message.data[2], (int)(*readings)[device.number][9]);
					device.lastReadingsMs = millis();
					break;
				default:
					errorAdd(message, ERROR_COMMAND_UNKNOWN, false, true);
				}
			}
			// print(".\n\r"); 
			return true;
		}
	return false;
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


/** If sensor not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_col_b::started(Device& device) {
	if (millis() - device.lastReadingsMs > MRM_COL_B_INACTIVITY_ALLOWED_MS || device.lastReadingsMs == 0) {//Restart sensor if no readings for 10s or never started
		// print("Start mrm-col-b %i \n\r", device.number);
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(&device, 0);
			// Wait for 1. message.
			uint64_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - device.lastReadingsMs < 100) {
					// print("mrm-col-b started\n\r");
					return true;
				}
				delayMs(1);
			}
		}
		sprintf(errorMessage, "%s %i dead.", _boardsName.c_str(), device.number);
		return false;
	}
	else
		return true;
}




/**Test
*/
void Mrm_col_b::test()
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

					print("Vi:%3i ViB:%3i BB:%3i B:%3i",
						colorViolet(device.number), colorVioletDeepBlue(device.number), colorBroadBlue(device.number),
						colorBlue(device.number));
					print(" G1:%3i GY:%3i G2:%3i YO:%3i",
						colorGreen1(device.number), colorBroadGreenYellow(device.number), colorGreen2(device.number), 
						colorBroadYellowOrange(device.number));
					print(" R:%3i DR:%3i FR:%3i NIR:%3i", 
						colorRed(device.number), colorDeepRed(device.number), colorFarRed(device.number), 
						colorNearIR(device.number));
					print(" Fl:%3i Cl:%3i", 
						colorFlicker(device.number), colorClear(device.number));
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}

