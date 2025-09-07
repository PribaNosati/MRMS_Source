#include "mrm-8x8a.h"
#include <mrm-robot.h>

std::map<int, std::string>* Mrm_8x8a::commandNamesSpecific = NULL;

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_8x8a::Mrm_8x8a(uint8_t maxNumberOfBoards) : 
	SensorBoard(1, "LED8x8", maxNumberOfBoards, ID_MRM_8x8A, MRM_8x8A_SWITCHES_COUNT) {
	displayedLast = new std::vector<uint8_t>(maxNumberOfBoards);
	displayedTypeLast = new std::vector<uint8_t>(maxNumberOfBoards);
	lastOn = new std::vector<bool[MRM_8x8A_SWITCHES_COUNT]>(maxNumberOfBoards);
	on = new std::vector<bool[MRM_8x8A_SWITCHES_COUNT]>(maxNumberOfBoards);
	offOnAction = new std::vector<ActionBase* [MRM_8x8A_SWITCHES_COUNT]>(maxNumberOfBoards);
	//mrm_can_bus = esp32CANBusSingleton;
	nextFree = 0;

	if (commandNamesSpecific == NULL){
		commandNamesSpecific = new std::map<int, std::string>();
		commandNamesSpecific->insert({COMMAND_8X8_DISPLAY, 						"Display"});
		commandNamesSpecific->insert({COMMAND_8X8_SWITCH_ON, 					"Switch on"});
		commandNamesSpecific->insert({COMMAND_8X8_SWITCH_ON_REQUEST_NOTIFICATION, "Sw on req n"});
		commandNamesSpecific->insert({COMMAND_8x8_TEST_CAN_BUS, 					"Test CAN b"});
		commandNamesSpecific->insert({COMMAND_8X8_BITMAP_DISPLAY_PART1, 			"Bitmap d.1"});
		commandNamesSpecific->insert({COMMAND_8X8_BITMAP_DISPLAY_PART2, 			"Bitmap d.2"});
		commandNamesSpecific->insert({COMMAND_8X8_BITMAP_DISPLAY_PART3, 			"Bitmap d.3"});
		commandNamesSpecific->insert({COMMAND_8X8_BITMAP_STORE_PART1, 			"Bitmap s.1"});
		commandNamesSpecific->insert({COMMAND_8X8_BITMAP_STORE_PART2, 			"Bitmap s.2"});
		commandNamesSpecific->insert({COMMAND_8X8_BITMAP_STORE_PART3, 			"Bitmap s.3"});
		commandNamesSpecific->insert({COMMAND_8X8_BITMAP_STORED_DISPLAY, 		"Bitm st di"});
		commandNamesSpecific->insert({COMMAND_8X8_ROTATION_SET, 					"Rotat. set"});
		commandNamesSpecific->insert({COMMAND_8X8_TEXT_1, 						"Text 1"});
		commandNamesSpecific->insert({COMMAND_8X8_TEXT_2, 						"Text 2"});
		commandNamesSpecific->insert({COMMAND_8X8_TEXT_3,  						"Text 3"});
		commandNamesSpecific->insert({COMMAND_8X8_TEXT_4, 						"Text 4"});
		commandNamesSpecific->insert({COMMAND_8X8_TEXT_5, 						"Text 5"});
		commandNamesSpecific->insert({COMMAND_8X8_TEXT_6, 						"Text 6"});
	}
}

Mrm_8x8a::~Mrm_8x8a()
{
}


ActionBase* Mrm_8x8a::actionCheck() {
	for (Device& device : devices) {
		for (uint8_t switchNumber = 0; switchNumber < MRM_8x8A_SWITCHES_COUNT; switchNumber++){
			if ((*lastOn)[device.number][switchNumber] == false && switchRead(switchNumber, device.number) && (*offOnAction)[device.number][switchNumber] != NULL)
				return (*offOnAction)[device.number][switchNumber];
			else if ((*lastOn)[device.number][switchNumber] == true && !switchRead(switchNumber, device.number)){
				((*lastOn)[device.number][switchNumber]) = false;
			}
		}
	}
	return NULL;
}

void Mrm_8x8a::actionSet(ActionBase* action, uint8_t switchNumber, uint8_t deviceNumber) {
	(*offOnAction)[deviceNumber][switchNumber] = action;
}

/** Add a mrm-8x8a board
@param deviceName - device's name
*/
void Mrm_8x8a::add(char * deviceName)
{
	uint16_t canIn = 0, canOut = 0;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_8x8A0_IN;
		canOut = CAN_ID_8x8A0_OUT;
		break;
	case 1:
		canIn = CAN_ID_8x8A1_IN;
		canOut = CAN_ID_8x8A1_OUT;
		break;
	case 2:
		canIn = CAN_ID_8x8A2_IN;
		canOut = CAN_ID_8x8A2_OUT;
		break;
	case 3:
		canIn = CAN_ID_8x8A3_IN;
		canOut = CAN_ID_8x8A3_OUT;
		break;
	case 4:
		canIn = CAN_ID_8x8A4_IN;
		canOut = CAN_ID_8x8A4_OUT;
		break;
	case 5:
		canIn = CAN_ID_8x8A5_IN;
		canOut = CAN_ID_8x8A5_OUT;
		break;
	case 6:
		canIn = CAN_ID_8x8A6_IN;
		canOut = CAN_ID_8x8A6_OUT;
		break;
	case 7:
		canIn = CAN_ID_8x8A7_IN;
		canOut = CAN_ID_8x8A7_OUT;
		break;
	default:
		sprintf(errorMessage, "Too many %s: %i.", _boardsName.c_str(), nextFree);
	}

	for (uint8_t i = 0; i < MRM_8x8A_SWITCHES_COUNT; i++) {
		(*on)[nextFree][i] = false;
		(*lastOn)[nextFree][i] = false;
		(*offOnAction)[nextFree][i] = NULL;
	}

	(*displayedLast)[nextFree] = 0xFF;
	(*displayedTypeLast)[nextFree] = LED8x8Type::LED_8X8_CUSTOM;

	SensorBoard::add(deviceName, canIn, canOut);
}

/** Display stored (in sensor, read-only) bitmap
@param bitmapId - bitmap's id
@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_8x8a::bitmapDisplay(uint8_t bitmapId, uint8_t deviceNumber){
	if (bitmapId != (*displayedLast)[deviceNumber] || (*displayedTypeLast)[deviceNumber] != LED8x8Type::LED_8X8_STORED) {
		(*displayedLast)[deviceNumber] = bitmapId;
		(*displayedTypeLast)[deviceNumber] = LED8x8Type::LED_8X8_STORED;
		aliveWithOptionalScan(&devices[deviceNumber], true);
		canData[0] = COMMAND_8X8_DISPLAY;
		canData[1] = bitmapId;
		messageSend(canData, 2, devices[deviceNumber].canIdIn);
	}
}

/** Display custom bitmap
@param red - 8-byte array for red
@param green - 8-byte array for green
@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_8x8a::bitmapCustomDisplay(uint8_t red[], uint8_t green[], uint8_t deviceNumber) {
	aliveWithOptionalScan(&devices[deviceNumber], true);
	canData[0] = COMMAND_8X8_BITMAP_DISPLAY_PART1;
	for (uint8_t i = 0; i < 7; i++) 
		canData[i + 1] = green[i];
	messageSend(canData, 8, deviceNumber);

	canData[0] = COMMAND_8X8_BITMAP_DISPLAY_PART2;
	canData[1] = green[7];
	for (uint8_t i = 0; i < 6; i++) 
		canData[i + 2] = red[i];
	messageSend(canData, 8, deviceNumber);

	canData[0] = COMMAND_8X8_BITMAP_DISPLAY_PART3;
	for (uint8_t i = 0; i < 2; i++) 
		canData[i + 1] = red[i + 6];
	messageSend(canData, 3, deviceNumber);

	(*displayedTypeLast)[deviceNumber] = LED8x8Type::LED_8X8_CUSTOM;
}

/** Store custom bitmap
@param red - 8-byte array for red
@param green - 8-byte array for green
@param addres - address in display's RAM. 0 - 99.
@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_8x8a::bitmapCustomStore(uint8_t red[], uint8_t green[], uint8_t address, uint8_t deviceNumber) {
	aliveWithOptionalScan(&devices[deviceNumber], true);

	canData[0] = COMMAND_8X8_BITMAP_STORE_PART1;
	for (uint8_t i = 0; i < 7; i++)
		canData[i + 1] = green[i];
	messageSend(canData, 8, deviceNumber);

	canData[0] = COMMAND_8X8_BITMAP_STORE_PART2;
	canData[1] = green[7];
	for (uint8_t i = 0; i < 6; i++)
		canData[i + 2] = red[i];
	messageSend(canData, 8, deviceNumber);

	canData[0] = COMMAND_8X8_BITMAP_STORE_PART3;
	for (uint8_t i = 0; i < 2; i++)
		canData[i + 1] = red[i + 6];
	canData[3] = address;
	messageSend(canData, 4, deviceNumber);
}

/** Display custom stored bitmap
@param addres - address in display's RAM. 0 - 99.
@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_8x8a::bitmapCustomStoredDisplay(uint8_t address, uint8_t deviceNumber) {
	if (address != (*displayedLast)[deviceNumber] || (*displayedTypeLast)[deviceNumber] != LED8x8Type::LED_8X8_STORED_CUSTOM) {
		(*displayedLast)[deviceNumber] = address;
		(*displayedTypeLast)[deviceNumber] = LED8x8Type::LED_8X8_STORED_CUSTOM;
		aliveWithOptionalScan(&devices[deviceNumber], true);
		canData[0] = COMMAND_8X8_BITMAP_STORED_DISPLAY;
		canData[1] = address;
		messageSend(canData, 2, deviceNumber);
	}
}

void Mrm_8x8a::bitmapsSet(const std::vector<ledSign>& selectedImages) {

	// The 2 arrays will hold red and green pixels. Both red an green pixel on - orange color.
	uint8_t red[8];
	uint8_t green[8];

	/* 1 will turn the pixel on, 0 off. 0bxxxxxxxx is a binary format of the number. Start with "0b" and list all the bits, starting from
	the most significant one (MSB). Do that for each byte of the green and red arrays.*/

	// Define Your bitmaps here.
	// Example
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_CUSTOM) != selectedImages.end()){
		green[0] = 0b00000001;
		green[1] = 0b00000011;
		green[2] = 0b00000111;
		green[3] = 0b00001111;
		green[4] = 0b00011111;
		green[5] = 0b00111111;
		green[6] = 0b01111111;
		green[7] = 0b11111111;

		red[0] = 0b11111111;
		red[1] = 0b01111111;
		red[2] = 0b00111111;
		red[3] = 0b00011111;
		red[4] = 0b00001111;
		red[5] = 0b00000111;
		red[6] = 0b00000011;
		red[7] = 0b00000001;
		bitmapCustomStore(red, green, LED_CUSTOM);
		delayMs(1);
	}


	// Curve left.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_CURVE_LEFT) != selectedImages.end()){
		green[0] = 0b00000000;
		green[1] = 0b00000000;
		green[2] = 0b11111000;
		green[3] = 0b11111000;
		green[4] = 0b00011000;
		green[5] = 0b00011000;
		green[6] = 0b00011000;
		green[7] = 0b00011000;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_CURVE_LEFT);
		delayMs(1);
	}

	// Curve right.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_CURVE_RIGHT) != selectedImages.end()){
		green[0] = 0b00000000;
		green[1] = 0b00000000;
		green[2] = 0b00011111;
		green[3] = 0b00011111;
		green[4] = 0b00011000;
		green[5] = 0b00011000;
		green[6] = 0b00011000;
		green[7] = 0b00011000;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_CURVE_RIGHT);
		delayMs(1);
	}

	// Evacuation zone
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_EVACUATION_ZONE) != selectedImages.end()){
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		green[0] = 0b11111111;
		green[1] = 0b10000001;
		green[2] = 0b10000001;
		green[3] = 0b10000001;
		green[4] = 0b10000001;
		green[5] = 0b11000001;
		green[6] = 0b11100001;
		green[7] = 0b11111111;
		bitmapCustomStore(red, green, LED_EVACUATION_ZONE);
		delayMs(1);
	}

	// Full crossing, both marks.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_FULL_CROSSING_BOTH_MARKS) != selectedImages.end()){
		green[0] = 0b00000000;
		green[1] = 0b00000000;
		green[2] = 0b11111111;
		green[3] = 0b11111111;
		green[4] = 0b00011000;
		green[5] = 0b00011000;
		green[6] = 0b00011000;
		green[7] = 0b00011000;

		red[0] = 0b00000000;
		red[1] = 0b00000000;
		red[2] = 0b00000000;
		red[3] = 0b00000000;
		red[4] = 0b01100110;
		red[5] = 0b01100110;
		red[6] = 0b00000000;
		red[7] = 0b00000000;
		bitmapCustomStore(red, green, LED_FULL_CROSSING_BOTH_MARKS);
		delayMs(1);
	}

	// Full crossing, mark left.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_FULL_CROSSING_MARK_LEFT) != selectedImages.end()){
		green[0] = 0b00000000;
		green[1] = 0b00000000;
		green[2] = 0b11111111;
		green[3] = 0b11111111;
		green[4] = 0b00011000;
		green[5] = 0b00011000;
		green[6] = 0b00011000;
		green[7] = 0b00011000;

		red[0] = 0b00000000;
		red[1] = 0b00000000;
		red[2] = 0b00000000;
		red[3] = 0b00000000;
		red[4] = 0b01100000;
		red[5] = 0b01100000;
		red[6] = 0b00000000;
		red[7] = 0b00000000;
		bitmapCustomStore(red, green, LED_FULL_CROSSING_MARK_LEFT);
		delayMs(1);
	}

	// Full crossing, mark right.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_FULL_CROSSING_MARK_RIGHT) != selectedImages.end()){
		green[0] = 0b00000000;
		green[1] = 0b00000000;
		green[2] = 0b11111111;
		green[3] = 0b11111111;
		green[4] = 0b00011000;
		green[5] = 0b00011000;
		green[6] = 0b00011000;
		green[7] = 0b00011000;

		red[0] = 0b00000000;
		red[1] = 0b00000000;
		red[2] = 0b00000000;
		red[3] = 0b00000000;
		red[4] = 0b00000110;
		red[5] = 0b00000110;
		red[6] = 0b00000000;
		red[7] = 0b00000000;
		bitmapCustomStore(red, green, LED_FULL_CROSSING_MARK_RIGHT);
		delayMs(1);
	}

	// Full crossing, no marks.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_FULL_CROSSING_NO_MARK) != selectedImages.end()){
		green[0] = 0b00011000;
		green[1] = 0b00011000;
		green[2] = 0b11111111;
		green[3] = 0b11111111;
		green[4] = 0b00011000;
		green[5] = 0b00011000;
		green[6] = 0b00011000;
		green[7] = 0b00011000;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_FULL_CROSSING_NO_MARK);
		delayMs(1);
	}

	// Half crossing, mark right.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_HALF_CROSSING_MARK_RIGHT) != selectedImages.end()){
		green[0] = 0b00011000;
		green[1] = 0b00011000;
		green[2] = 0b00011111;
		green[3] = 0b00011111;
		green[4] = 0b00011000;
		green[5] = 0b00011000;
		green[6] = 0b00011000;
		green[7] = 0b00011000;

		red[0] = 0b00000000;
		red[1] = 0b00000000;
		red[2] = 0b00000000;
		red[3] = 0b00000000;
		red[4] = 0b00000110;
		red[5] = 0b00000110;
		red[6] = 0b00000000;
		red[7] = 0b00000000;
		bitmapCustomStore(red, green, LED_HALF_CROSSING_MARK_RIGHT);
		delayMs(1);
	}

	// Half crossing, mark left.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_HALF_CROSSING_MARK_LEFT) != selectedImages.end()){
		green[0] = 0b00011000;
		green[1] = 0b00011000;
		green[2] = 0b11111000;
		green[3] = 0b11111000;
		green[4] = 0b00011000;
		green[5] = 0b00011000;
		green[6] = 0b00011000;
		green[7] = 0b00011000;

		red[0] = 0b00000000;
		red[1] = 0b00000000;
		red[2] = 0b00000000;
		red[3] = 0b00000000;
		red[4] = 0b01100000;
		red[5] = 0b01100000;
		red[6] = 0b00000000;
		red[7] = 0b00000000;
		bitmapCustomStore(red, green, LED_HALF_CROSSING_MARK_LEFT);
		delayMs(1);
	}

	// Half crossing right, no mark.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_HALF_CROSSING_RIGHT_NO_MARK) != selectedImages.end()){
		green[0] = 0b00011000;
		green[1] = 0b00011000;
		green[2] = 0b00011111;
		green[3] = 0b00011111;
		green[4] = 0b00011000;
		green[5] = 0b00011000;
		green[6] = 0b00011000;
		green[7] = 0b00011000;

		red[0] = 0b00000000;
		red[1] = 0b00000000;
		red[2] = 0b00000000;
		red[3] = 0b00000000;
		red[4] = 0b00000000;
		red[5] = 0b00000000;
		red[6] = 0b00000000;
		red[7] = 0b00000000;
		bitmapCustomStore(red, green, LED_HALF_CROSSING_RIGHT_NO_MARK);
		delayMs(1);
	}

	// Half crossing left, no mark
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_HALF_CROSSING_LEFT_NO_MARK) != selectedImages.end()){
		green[0] = 0b00011000;
		green[1] = 0b00011000;
		green[2] = 0b11111000;
		green[3] = 0b11111000;
		green[4] = 0b00011000;
		green[5] = 0b00011000;
		green[6] = 0b00011000;
		green[7] = 0b00011000;

		red[0] = 0b00000000;
		red[1] = 0b00000000;
		red[2] = 0b00000000;
		red[3] = 0b00000000;
		red[4] = 0b00000000;
		red[5] = 0b00000000;
		red[6] = 0b00000000;
		red[7] = 0b00000000;
		bitmapCustomStore(red, green, LED_HALF_CROSSING_LEFT_NO_MARK);
		delayMs(1);
	}

	// Follow IMU.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_IMU_FOLLOW) != selectedImages.end()){
		green[0] = 0b00000000;
		green[1] = 0b00010000;
		green[2] = 0b00111000;
		green[3] = 0b01111100;
		green[4] = 0b00111000;
		green[5] = 0b00111000;
		green[6] = 0b00000000;
		green[7] = 0b00000000;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_IMU_FOLLOW);
		delayMs(1);
	}

	// Full line, no marks
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_LINE_FULL) != selectedImages.end()){
		for (uint8_t i = 0; i < 8; i++)
			green[i] = 0b00011000;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_LINE_FULL);
		delayMs(1);
	}

	// Full line, both marks
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_LINE_FULL_BOTH_MARKS) != selectedImages.end()){
		for (uint8_t i = 0; i < 8; i++)
			green[i] = 0b00011000;
		red[0] = 0b00000000;
		red[1] = 0b00000000;
		red[2] = 0b00000000;
		red[3] = 0b00000000;
		red[4] = 0b01100110;
		red[5] = 0b01100110;
		red[6] = 0b00000000;
		red[7] = 0b00000000;
		/* Store this bitmap in mrm-8x8a. The 3rd parameter is bitmap's address. If You want to define new bitmaps, expand LedSign enum with
		Your names, and use the new values for Your bitmaps. This parameter can be a plain number, but enum keeps thing tidy.*/
		bitmapCustomStore(red, green, LED_LINE_FULL_BOTH_MARKS);
		delayMs(1);
	}

	// Full line, left mark.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_LINE_FULL_MARK_LEFT) != selectedImages.end()){
		for (uint8_t i = 0; i < 8; i++)
			green[i] = 0b00011000;
		red[0] = 0b00000000;
		red[1] = 0b00000000;
		red[2] = 0b00000000;
		red[3] = 0b00000000;
		red[4] = 0b01100000;
		red[5] = 0b01100000;
		red[6] = 0b00000000;
		red[7] = 0b00000000;
		bitmapCustomStore(red, green, LED_LINE_FULL_MARK_LEFT);
		delayMs(1);
	}

	// Full line, right mark.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_LINE_FULL_MARK_RIGHT) != selectedImages.end()){
		for (uint8_t i = 0; i < 8; i++)
			green[i] = 0b00011000;
		red[0] = 0b00000000;
		red[1] = 0b00000000;
		red[2] = 0b00000000;
		red[3] = 0b00000000;
		red[4] = 0b00000110;
		red[5] = 0b00000110;
		red[6] = 0b00000000;
		red[7] = 0b00000000;
		bitmapCustomStore(red, green, LED_LINE_FULL_MARK_RIGHT);
		delayMs(1);
	}

	// Interrupted line.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_LINE_INTERRUPTED) != selectedImages.end()){
		green[0] = 0b00011000;
		green[1] = 0b00011000;
		green[2] = 0b00000000;
		green[3] = 0b00000000;
		green[4] = 0b00000000;
		green[5] = 0b00000000;
		green[6] = 0b00011000;
		green[7] = 0b00011000;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_LINE_INTERRUPTED);
		delayMs(1);
	}

	// Obstacle.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_OBSTACLE) != selectedImages.end()){
		green[0] = 0b00011000;
		green[1] = 0b00011000;
		green[2] = 0b00000000;
		green[3] = 0b00011000;
		green[4] = 0b00111100;
		green[5] = 0b01111110;
		green[6] = 0b00011000;
		green[7] = 0b00011000;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_OBSTACLE);
		delayMs(1);
	}

	// Around obstacle left.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_OBSTACLE_AROUND_LEFT) != selectedImages.end()){
		green[0] = 0b00000000;
		green[1] = 0b00000000;
		green[2] = 0b00000011;
		green[3] = 0b00100011;
		green[4] = 0b01110000;
		green[5] = 0b11111000;
		green[6] = 0b01110000;
		green[7] = 0b01110000;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_OBSTACLE_AROUND_LEFT);
		delayMs(1);
	}

	// Around obstacle right.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_OBSTACLE_AROUND_RIGHT) != selectedImages.end()){
		green[0] = 0b00000000;
		green[1] = 0b00000000;
		green[2] = 0b11000000;
		green[3] = 0b11000100;
		green[4] = 0b00001110;
		green[5] = 0b00011111;
		green[6] = 0b00001110;
		green[7] = 0b00001110;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_OBSTACLE_AROUND_RIGHT);
		delayMs(1);
	}

	// Pause.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_PAUSE) != selectedImages.end()){
		green[0] = 0b11100111;
		green[1] = 0b11100111;
		green[2] = 0b11100111;
		green[3] = 0b11100111;
		green[4] = 0b11100111;
		green[5] = 0b11100111;
		green[6] = 0b11100111;
		green[7] = 0b11100111;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_PAUSE);
		delayMs(1);
	}

	// Play.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_PLAY) != selectedImages.end()){
		green[0] = 0b0110000;
		green[1] = 0b0111000;
		green[2] = 0b0111100;
		green[3] = 0b0111110;
		green[4] = 0b0111110;
		green[5] = 0b0111100;
		green[6] = 0b0111000;
		green[7] = 0b0110000;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_PLAY);
		delayMs(1);
	}

	// T-crossing approached by left side.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_T_CROSSING_BY_L) != selectedImages.end()){
		green[0] = 0b0100000;
		green[1] = 0b0100000;
		green[2] = 0b0100000;
		green[3] = 0b0111111;
		green[4] = 0b0100000;
		green[5] = 0b0100000;
		green[6] = 0b0100000;
		green[7] = 0b0100000;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_T_CROSSING_BY_L);
		delayMs(1);
	}

	// T-crossing approached by right side.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_T_CROSSING_BY_R) != selectedImages.end()){
		green[0] = 0b0000100;
		green[1] = 0b0000100;
		green[2] = 0b0000100;
		green[3] = 0b1111100;
		green[4] = 0b0000100;
		green[5] = 0b0000100;
		green[6] = 0b0000100;
		green[7] = 0b0000100;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_T_CROSSING_BY_R);
		delayMs(1);
	}

	// Wall ahead
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_WALL_AHEAD) != selectedImages.end()){
		green[0] = 0b11111111;
		green[1] = 0b00010000;
		green[2] = 0b00111000;
		green[3] = 0b01010100;
		green[4] = 0b00000000;
		green[5] = 0b00111000;
		green[6] = 0b00101000;
		green[7] = 0b00111000;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_WALL_AHEAD);
		delayMs(1);
	}

	// Follow wall down, green taken from Follow IMU bitmap.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_WALL_DOWN_FOLLOW) != selectedImages.end()){
		for (uint8_t i = 0; i < 7; i++)
			red[i] = 0;
		red[7] = 0b11111111;
		bitmapCustomStore(red, green, LED_WALL_DOWN_FOLLOW);
		delayMs(1);
	}

	// Wall on the left side
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_WALL_L) != selectedImages.end()){
		green[0] = 0b10001000;
		green[1] = 0b10011100;
		green[2] = 0b10101010;
		green[3] = 0b10001000;
		green[4] = 0b10000000;
		green[5] = 0b10011100;
		green[6] = 0b10010100;
		green[7] = 0b10011100;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_WALL_L);
		delayMs(1);
	}

	// Follow wall left, green taken from Follow IMU bitmap.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_WALL_LEFT_FOLLOW) != selectedImages.end()){
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0b10000000;
		bitmapCustomStore(red, green, LED_WALL_LEFT_FOLLOW);
		delayMs(1);
	}

	// Wall on the right side
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_WALL_R) != selectedImages.end()){
		green[0] = 0b00010001;
		green[1] = 0b00111001;
		green[2] = 0b01010101;
		green[3] = 0b00010001;
		green[4] = 0b00000001;
		green[5] = 0b00111001;
		green[6] = 0b00101001;
		green[7] = 0b00111001;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_WALL_R);
		delayMs(1);
	}

	// Follow wall right, green taken from Follow IMU bitmap.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_WALL_RIGHT_FOLLOW) != selectedImages.end()){
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0b00000001;
		bitmapCustomStore(red, green, LED_WALL_RIGHT_FOLLOW);
		delayMs(1);
	}

	// Follow wall up, green taken from Follow IMU bitmap.
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_WALL_UP_FOLLOW) != selectedImages.end()){
		red[0] = 0b11111111;
		for (uint8_t i = 1; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_WALL_UP_FOLLOW);
		delayMs(1);
	}

	// LED approach opponent's goal
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_GOAL_APPROACH) != selectedImages.end()){
		green[0] = 0b01111110;
		green[1] = 0b01000010;
		green[2] = 0b00000000;
		green[3] = 0b00000000;
		green[4] = 0b00000000;
		green[5] = 0b00000000;
		green[6] = 0b00000000;
		green[7] = 0b00000000;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_GOAL_APPROACH);
		delayMs(1);
	}

	// LED approach opponent's goal
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_IDLE) != selectedImages.end()){
		green[0] = 0b00000000;
		green[1] = 0b00000000;
		green[2] = 0b00000000;
		green[3] = 0b00000000;
		green[4] = 0b00000000;
		green[5] = 0b00000000;
		green[6] = 0b01000010;
		green[7] = 0b01111110;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_IDLE);
		delayMs(1);
	}
	// LED approach opponent's goal
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_LINE_AVOID) != selectedImages.end()){
		green[0] = 0b11111111;
		green[1] = 0b10000001;
		green[2] = 0b10000001;
		green[3] = 0b10000001;
		green[4] = 0b10000001;
		green[5] = 0b10000001;
		green[6] = 0b10000001;
		green[7] = 0b11111111;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_LINE_AVOID);
		delayMs(1);
	}
	// LED approach opponent's goal
	if (std::find(selectedImages.begin(), selectedImages.end(), LED_CATCH) != selectedImages.end()){
		green[0] = 0b00011000;
		green[1] = 0b00011000;
		green[2] = 0b00000000;
		green[3] = 0b00100100;
		green[4] = 0b01011010;
		green[5] = 0b01000010;
		green[6] = 0b00100100;
		green[7] = 0b00011000;
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0;
		bitmapCustomStore(red, green, LED_CATCH);
		delayMs(1);
	}
}


std::string Mrm_8x8a::commandName(uint8_t byte){
	auto it = commandNamesSpecific->find(byte);
	if (it == commandNamesSpecific->end())
		return "Warning: no command found for key " + (int)byte;
	else
		return it->second;//commandNamesSpecific->at(byte);
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
@return - true if canId for this class
*/
bool Mrm_8x8a::messageDecode(CANMessage message) {
	for (Device& device : devices)
		if (isForMe(message.id, device)) {
			if (!messageDecodeCommon(message, device)) {
				switch (message.data[0]) {
				case COMMAND_8X8_SWITCH_ON:
				case COMMAND_8X8_SWITCH_ON_REQUEST_NOTIFICATION: {
					uint8_t switchNumber = message.data[1] >> 1;
						if (switchNumber > 4) {
							sprintf(errorMessage, "No %s: %i.", _boardsName.c_str(), switchNumber);
							return false;
						}
					(*on)[device.number][switchNumber] = message.data[1] & 1;
						if (message.data[0] == COMMAND_8X8_SWITCH_ON_REQUEST_NOTIFICATION) {
							canData[0] = COMMAND_NOTIFICATION;
							canData[1] = switchNumber; //todo - deviceNumber not taken into account
							messageSend(canData, 2, device.number);
						}
						device.lastReadingsMs = millis();
				}
					break;
				case COMMAND_8x8_TEST_CAN_BUS:
					print("Test: %i\n\r", message.data[1]);
					break;
				default:
					print("Unknown command. ");
					messagePrint(message, false);
					print("\n\r");
					errorAdd(message.id, ERROR_COMMAND_UNKNOWN, false);
				} 
			}
			return true;
		}
	return false;
}

/** Displays 8-row progress bar. Useful for visual feedback of a long process.
@param period - total count (100%)
@param current - current count (current percentage)
@param reset - reset to start (no bar)
@return - display changed
*/
bool Mrm_8x8a::progressBar(uint32_t period, uint32_t current, bool reset) {
	// static uint32_t _period = 0;
	static uint32_t lastDisplayMs = 0;
	static uint8_t lastDot = 0xFF;
	static bool lastGreen = true;

	if (current > period) {
		strcpy(errorMessage, "Overflow in progress bar");
		return false;
	}

	if (reset) {
		// _period = period;
		lastDisplayMs = 0;
	}

	uint8_t currentDot = ceil(current * 64 / (float)period); // 0 - 64
	if (millis() - lastDisplayMs > 500 || currentDot != lastDot) { // Blink each 0.5 sec
		uint8_t green[8], red[8];
		for (uint8_t i = 0; i < 8; i++)
			red[i] = 0b00000000;

		// Display full rows.
		uint8_t partialRow = currentDot / 8;
		for (uint8_t i = 0; i < partialRow; i++) // Display full rows, 0 - 8
			green[i] = 0b11111111;

		// Display partially full row.
		for (uint8_t i = 0; i < 8; i++)
			green[partialRow] = (green[partialRow] << 1) | (i < currentDot - partialRow * 8 ? 1 : 0);

		// Display empty rows.
		for (uint8_t i = partialRow + 1; i < 8; i++)
			green[i] = 0b00000000;

		bitmapCustomDisplay(lastGreen ? red : green, lastGreen ? green : red);
		lastDot = currentDot;
		lastDisplayMs = millis();
		lastGreen = !lastGreen; // todo
		return true;
	}
	else
		return false;
}

/** Set rotation from now on
@param rotation - 0, 90, or 270 degrees counterclockwise
@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_8x8a::rotationSet(enum LED8x8Rotation rotation, uint8_t deviceNumber) {
	aliveWithOptionalScan(&devices[deviceNumber], true);
	canData[0] = COMMAND_8X8_ROTATION_SET;
	canData[1] = rotation;
	messageSend(canData, 2, deviceNumber);
}

/** If sensor not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_8x8a::started(uint8_t deviceNumber) {
	static uint64_t lastTriedMs = 0;
	if (_activeCheckIfStarted && (millis() - devices[deviceNumber].lastReadingsMs > MRM_8X8A_INACTIVITY_ALLOWED_MS || devices[deviceNumber].lastReadingsMs == 0)) {
		if (millis() - lastTriedMs > 5000){
			for (uint8_t i = 0; i < 2; i++) { // 2 tries
				start(&devices[deviceNumber], 0);
				// Wait for 1. message.
				uint64_t startMs = millis();
				while (millis() - startMs < 50) {
					if (millis() - devices[deviceNumber].lastReadingsMs < 100) {
						//print("8x8 confirmed\n\r");
						return true;
					}
					delayMs(1);
				}
			}
			print("%s %i not responding.", _boardsName.c_str(), deviceNumber);
			lastTriedMs = millis();
		}
		return false;
	}
	else
		return true;
}

/** Read switch
@param switchNumber
@deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - true if pressed, false otherwise
*/
bool Mrm_8x8a::switchRead(uint8_t switchNumber, uint8_t deviceNumber) {
	aliveWithOptionalScan(&devices[deviceNumber], true);
	if (deviceNumber >= nextFree || switchNumber >= MRM_8x8A_SWITCHES_COUNT) {
		strcpy(errorMessage, "Switch doesn't exist");
		return false;
	}
	if (started(deviceNumber)){
		return (*on)[deviceNumber][switchNumber];
	}
	else
		return false;
}


/**Test
*/
void Mrm_8x8a::test()
{
	#define MRM_8x8A_START_BITMAP_1 0x01
	#define MRM_8x8A_END_BITMAP_1 0x04
	#define MRM_8x8A_START_BITMAP_2 0x30
	#define MRM_8x8A_END_BITMAP_2 0x5A
	static uint32_t lastMs = 0;
	static uint8_t bitmapId = MRM_8x8A_START_BITMAP_1;

	if (setup()) {
		uint8_t red[8] = { 0b00000000, 0b01100110, 0b11111111, 0b11111111, 0b11111111, 0b01111110, 0b00111100, 0b00011000 };
		uint8_t green[8] = { 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000 };
		bitmapCustomStore(red, green, 7);
	}

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;

		// Built-in bitmap
		for (Device& device: devices){
			if (device.alive) {
				bitmapDisplay(bitmapId, device.number);
				if (pass++)
					print("| ");
				print("Map 0x%02x, sw:", bitmapId);
				for (uint8_t i = 0; i < MRM_8x8A_SWITCHES_COUNT; i++)
					print("%i ", switchRead(i, device.number));
			}
		}
		bitmapId++;
		if (bitmapId > MRM_8x8A_END_BITMAP_1 && bitmapId < MRM_8x8A_START_BITMAP_2)
			bitmapId = MRM_8x8A_START_BITMAP_2;
		else if (bitmapId > MRM_8x8A_END_BITMAP_2) {
			bitmapId = MRM_8x8A_START_BITMAP_1;

			// Custom bitmap.
			delayMs(300);
			uint8_t red[8] = { 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000100, 0b00111000, 0b00000000, 0b00111100 };
			uint8_t green[8] = { 0b00111100, 0b01000010, 0b10101001, 0b10101001, 0b10000001, 0b10000001, 0b01000010, 0b00111100 };
			bitmapCustomDisplay(red, green);

			// Custom stored bitmap
			delayMs(300);
			bitmapCustomStoredDisplay(7);
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}

/** Display text
@param content - text
@param deviceNumber - Displays's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_8x8a::text(const char content[], uint8_t deviceNumber) {
	uint8_t message = 0;
	bool unsent = false;
	for (uint8_t i = 0; i < MRM_8X8A_TEXT_LENGTH; i++) {
		if (i % 7 == 0 && i != 0) {
			canData[0] = COMMAND_8X8_TEXT_1 + message;
			messageSend(canData, 8, deviceNumber);
			message++;
			unsent = false;
		}
		canData[i % 7 + 1] = content[i];
		unsent = true;
		if (content[i] == '\0')
			break;
	}
	if (unsent) {
		canData[0] = COMMAND_8X8_TEXT_1 + message;
		messageSend(canData, 8, deviceNumber);
	}
}

