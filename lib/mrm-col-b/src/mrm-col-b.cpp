#include "mrm-col-b.h"
#include <mrm-robot.h>

std::vector<uint8_t>* commandIndexes_mrm_col_b =  new std::vector<uint8_t>(); // C++ 17 enables static variables without global initialization, but no C++ 17 here
std::vector<String>* commandNames_mrm_col_b =  new std::vector<String>();

/** Constructor
@param robot - robot containing this board
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_col_b::Mrm_col_b(Robot* robot, uint8_t maxNumberOfBoards) : 
	SensorBoard(robot, 1, "Color", maxNumberOfBoards, ID_MRM_COL_B, MRM_COL_B_COLORS) {
	readings = new std::vector<uint16_t[MRM_COL_B_COLORS]>(maxNumberOfBoards);
	
	if (commandIndexes_mrm_col_b->empty()){
		commandIndexes_mrm_col_b->push_back(MRM_COL_B_SENDING_COLORS_1_TO_3);
		commandNames_mrm_col_b->push_back("Send 1-3");
		commandIndexes_mrm_col_b->push_back(MRM_COL_B_SENDING_COLORS_4_TO_6);
		commandNames_mrm_col_b->push_back("Send 4-6");
		commandIndexes_mrm_col_b->push_back(MRM_COL_B_SENDING_COLORS_7_TO_9);
		commandNames_mrm_col_b->push_back("Send 7-9");
		commandIndexes_mrm_col_b->push_back(MRM_COL_B_SENDING_COLORS_10_TO_12);
		commandNames_mrm_col_b->push_back("Send10-12");
		commandIndexes_mrm_col_b->push_back(MRM_COL_B_SENDING_COLORS_13_TO_14);
		commandNames_mrm_col_b->push_back("Send13-14");
		commandIndexes_mrm_col_b->push_back(MRM_COL_B_ILLUMINATION_CURRENT);
		commandNames_mrm_col_b->push_back("Illu curr");
		commandIndexes_mrm_col_b->push_back(MRM_COL_B_INTEGRATION_TIME);
		commandNames_mrm_col_b->push_back("Inte time");
		commandIndexes_mrm_col_b->push_back(MRM_COL_B_GAIN);
		commandNames_mrm_col_b->push_back("Gain");
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
	if (started(deviceNumber))
		return(*readings)[deviceNumber][2];
	else
		return 0;
}

/** Blue greenish
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorBlueGeenish(uint8_t deviceNumber){
	if (started(deviceNumber))
		return(*readings)[deviceNumber][3];
	else
		return 0;
}

/** Blue violetish
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorBlueVioletish(uint8_t deviceNumber){
	if (started(deviceNumber))
		return(*readings)[deviceNumber][1];
	else
		return 0;
}

/** Green
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorGreen(uint8_t deviceNumber) { 
	if (started(deviceNumber))
		return(*readings)[deviceNumber][4];
	else
		return 0;
}

/** Near IR
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t  Mrm_col_b::colorNearIR(uint8_t deviceNumber){
	if (started(deviceNumber))
		return(*readings)[deviceNumber][8];
	else
		return 0;
}

/** Orange
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorOrange(uint8_t deviceNumber) { 
	if (started(deviceNumber))
		return(*readings)[deviceNumber][6];
	else
		return 0;
}

/** Red
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorRed(uint8_t deviceNumber) { 
	if (started(deviceNumber))
		return(*readings)[deviceNumber][7];
	else
		return 0;
}

/** Violet
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorViolet(uint8_t deviceNumber) {
	if (started(deviceNumber))
		return(*readings)[deviceNumber][0];
	else
		return 0;
}

/** White
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t  Mrm_col_b::colorWhite(uint8_t deviceNumber){
	if (started(deviceNumber))
		return(*readings)[deviceNumber][9];
	else
		return 0;
}

/** Yellow
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_b::colorYellow(uint8_t deviceNumber) {
	if (started(deviceNumber))
		return(*readings)[deviceNumber][5];
	else
		return 0;
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
void Mrm_col_b::gain(uint8_t deviceNumber, uint8_t gainValue) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			gain(i, gainValue);
	else {
		canData[0] = MRM_COL_B_GAIN;
		canData[1] = gainValue;
		messageSend(canData, 2, deviceNumber);
	}
}


/** Set illumination intensity
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
@param current - 0 - 3
*/
void Mrm_col_b::illumination(uint8_t deviceNumber, uint8_t current) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			illumination(i, current);
	else {
		canData[0] = MRM_COL_B_ILLUMINATION_CURRENT;
		canData[1] = current;
		messageSend(canData, 2, deviceNumber);
	}
}

/** Set integration time
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
@param time - sets the ATIME parameter for integration time from 0 to 255, integration time = (ATIME + 1) * (ASTEP + 1) * 2.78µS.
@param step - sets STEP.
*/
void Mrm_col_b::integrationTime(uint8_t deviceNumber, uint8_t time, uint16_t step) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			integrationTime(i, time, step);
	else {
		canData[0] = MRM_COL_B_INTEGRATION_TIME;
		canData[1] = time;
		canData[2] = step >> 8;
		canData[3] = step & 0xFF;
		messageSend(canData, 4, deviceNumber);
	}
}

/** Read CAN Bus message into local variables
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
*/
bool Mrm_col_b::messageDecode(uint32_t canId, uint8_t data[8], uint8_t length) {
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) 
		if (isForMe(canId, deviceNumber)) {
			if (!messageDecodeCommon(canId, data, deviceNumber)) {
				switch (data[0]) {
				case COMMAND_SENSORS_MEASURE_SENDING:
					break;
				case MRM_COL_B_SENDING_COLORS_1_TO_3:
					(*readings)[deviceNumber][0] = (data[1] << 8) | data[2]; // violet
					// print("Data1: %i %i %i\n\r",(int)data[0], (int)data[1], (int)data[2]);
					(*readings)[deviceNumber][1] = (data[3] << 8) | data[4]; // blue violetish
					(*readings)[deviceNumber][2] = (data[5] << 8) | data[6]; // blue
					(*_lastReadingMs)[deviceNumber] = millis();
					break;
				case MRM_COL_B_SENDING_COLORS_4_TO_6:
					(*readings)[deviceNumber][3] = (data[1] << 8) | data[2]; // blue greenish
					// print("Data2: %i %i %i\n\r", (int)data[0], (int)data[1], (int)data[2]);
					(*readings)[deviceNumber][4] = (data[3] << 8) | data[4]; // green
					(*readings)[deviceNumber][5] = (data[5] << 8) | data[6]; // yellow
					(*_lastReadingMs)[deviceNumber] = millis();
					break;
				case MRM_COL_B_SENDING_COLORS_7_TO_9:
					// print("Data3: %i %i %i\n\r", (int)data[0], (int)data[1], (int)data[2]);
					(*readings)[deviceNumber][6] = (data[1] << 8) | data[2]; // orange
					(*readings)[deviceNumber][7] = (data[3] << 8) | data[4]; // red
					(*readings)[deviceNumber][8] = (data[5] << 8) | data[6]; // near IR
					(*_lastReadingMs)[deviceNumber] = millis();
					break;
				case MRM_COL_B_SENDING_COLORS_10_TO_12:
					// print("Data3: %i %i %i\n\r", (int)data[0], (int)data[1], (int)data[2]);
					(*readings)[deviceNumber][9] = (data[1] << 8) | data[2]; // orange
					(*readings)[deviceNumber][10] = (data[3] << 8) | data[4]; // red
					(*readings)[deviceNumber][11] = (data[5] << 8) | data[6]; // near IR
					(*_lastReadingMs)[deviceNumber] = millis();
					break;
				case MRM_COL_B_SENDING_COLORS_13_TO_14:
					(*readings)[deviceNumber][12] = (data[1] << 8) | data[2]; // clear (white)
					// print("Data4: %i %i %i %i\n\r", (int)data[0], (int)data[1], (int)data[2], (int)(*readings)[deviceNumber][12]);
					(*readings)[deviceNumber][13] = (data[3] << 8) | data[4]; // clear (white)
					(*_lastReadingMs)[deviceNumber] = millis();
					break;
				default:
					print("Unknown command. ");
					messagePrint(canId, length, data, false);
					errorCode = 204;
					errorInDeviceNumber = deviceNumber;
				}
			}
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
	for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
		for (uint8_t colorId = 0; colorId < MRM_COL_B_COLORS; colorId++)
			print(" %3i", (*readings)[deviceNumber][colorId]);
	}
}


/** If sensor not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_col_b::started(uint8_t deviceNumber) {
	if (millis() - (*_lastReadingMs)[deviceNumber] > MRM_COL_B_INACTIVITY_ALLOWED_MS || (*_lastReadingMs)[deviceNumber] == 0) {
		//print("Start mrm-col-b%i \n\r", deviceNumber);
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(deviceNumber, 0);
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - (*_lastReadingMs)[deviceNumber] < 100) {
					//print("Lidar confirmed\n\r"); 
					return true;
				}
				robotContainer->delayMs(1);
			}
		}
		sprintf(errorMessage, "%s %i dead.", _boardsName, deviceNumber);
		return false;
	}
	else
		return true;
}

/**Test
*/
void Mrm_col_b::test()
{
	static uint32_t lastMs = 0;
	if (millis() - lastMs > 5000){
		illumination(0, 16);
	}

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < nextFree; deviceNumber++) {
			if (alive(deviceNumber)) {
				if (pass++)
					print(" | ");
				print("Vi:%3i B1:%3i B2:%3i B3:%3i Gr:%3i Ye:%3i Or:%3i Re:%3i IR:%3i Wh:%3i", colorViolet(deviceNumber), colorBlueVioletish(deviceNumber), colorBlue(deviceNumber), 
					colorBlueGeenish(deviceNumber),	colorGreen(deviceNumber), colorYellow(deviceNumber), colorOrange(deviceNumber), colorRed(deviceNumber), colorNearIR(deviceNumber), 
					colorWhite(deviceNumber));
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}

