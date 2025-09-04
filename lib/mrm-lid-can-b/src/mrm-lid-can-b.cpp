#include "mrm-lid-can-b.h"
#include <mrm-robot.h>

std::map<int, std::string>* Mrm_lid_can_b::commandNamesSpecific = NULL;

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_lid_can_b::Mrm_lid_can_b(Robot* robot, uint8_t maxNumberOfBoards) : 
	SensorBoard(robot, 1, "Lid2m", maxNumberOfBoards, ID_MRM_LID_CAN_B, 1) {
	readings = new std::vector<uint16_t>(maxNumberOfBoards);

	if (commandNamesSpecific == NULL){
		commandNamesSpecific = new std::map<int, std::string>();
		commandNamesSpecific->insert({COMMAND_LID_CAN_B_CALIBRATE, 	"Calibrate"});
		commandNamesSpecific->insert({COMMAND_LID_CAN_B_RANGING_TYPE, "Rang type"});
	}
}

Mrm_lid_can_b::~Mrm_lid_can_b()
{
}

/** Add a mrm-lid-can-b device
@param deviceName - device's name
*/
void Mrm_lid_can_b::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_LID_CAN_B0_IN;
		canOut = CAN_ID_LID_CAN_B0_OUT;
		break;
	case 1:
		canIn = CAN_ID_LID_CAN_B1_IN;
		canOut = CAN_ID_LID_CAN_B1_OUT;
		break;
	case 2:
		canIn = CAN_ID_LID_CAN_B2_IN;
		canOut = CAN_ID_LID_CAN_B2_OUT;
		break;
	case 3:
		canIn = CAN_ID_LID_CAN_B3_IN;
		canOut = CAN_ID_LID_CAN_B3_OUT;
		break;
	case 4:
		canIn = CAN_ID_LID_CAN_B4_IN;
		canOut = CAN_ID_LID_CAN_B4_OUT;
		break;
	case 5:
		canIn = CAN_ID_LID_CAN_B5_IN;
		canOut = CAN_ID_LID_CAN_B5_OUT;
		break;
	case 6:
		canIn = CAN_ID_LID_CAN_B6_IN;
		canOut = CAN_ID_LID_CAN_B6_OUT;
		break;
	case 7:
		canIn = CAN_ID_LID_CAN_B7_IN;
		canOut = CAN_ID_LID_CAN_B7_OUT;
		break;
	case 8:
		canIn = CAN_ID_LID_CAN_B8_IN;
		canOut = CAN_ID_LID_CAN_B8_OUT;
		break;
	case 9:
		canIn = CAN_ID_LID_CAN_B9_IN;
		canOut = CAN_ID_LID_CAN_B9_OUT;
		break;
	case 10:
		canIn = CAN_ID_LID_CAN_B10_IN;
		canOut = CAN_ID_LID_CAN_B10_OUT;
		break;
	case 11:
		canIn = CAN_ID_LID_CAN_B11_IN;
		canOut = CAN_ID_LID_CAN_B11_OUT;
		break;
	case 12:
		canIn = CAN_ID_LID_CAN_B12_IN;
		canOut = CAN_ID_LID_CAN_B12_OUT;
		break;
	case 13:
		canIn = CAN_ID_LID_CAN_B13_IN;
		canOut = CAN_ID_LID_CAN_B13_OUT;
		break;
	case 14:
		canIn = CAN_ID_LID_CAN_B14_IN;
		canOut = CAN_ID_LID_CAN_B14_OUT;
		break;
	case 15:
		canIn = CAN_ID_LID_CAN_B15_IN;
		canOut = CAN_ID_LID_CAN_B15_OUT;
		break;
	default:
		sprintf(errorMessage, "Too many %s: %i.", _boardsName.c_str(), nextFree);
		return;
	}
	SensorBoard::add(deviceName, canIn, canOut);
}

/** Calibration, only once after production
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_lid_can_b::calibration(Device * device){
	if (device == nullptr)
		for (Device& dev : devices)
			calibration(&dev);
	else if (device->alive){
		canData[0] = COMMAND_LID_CAN_B_CALIBRATE;
		messageSend(canData, 1, device->number);
	}
}

std::string Mrm_lid_can_b::commandName(uint8_t byte){
	auto it = commandNamesSpecific->find(byte);
	if (it == commandNamesSpecific->end())
		return "Warning: no command found for key " + (int)byte;
	else
		return it->second;//commandNamesSpecific->at(byte);
}

/** Distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - distance in mm
*/
uint16_t Mrm_lid_can_b::distance(uint8_t deviceNumber, uint8_t sampleCount, uint8_t sigmaCount){
	const uint16_t TIMEOUT = 3000;
	if (deviceNumber > nextFree) {
		sprintf(errorMessage, "%s %i doesn't exist.", _boardsName.c_str(), deviceNumber);
		return 0;
	}
	aliveWithOptionalScan(&devices[deviceNumber], true); // This command doesn't make sense
	if (started(devices[deviceNumber]))
		if (sampleCount == 0)
			return (*readings)[deviceNumber];
		else{
			uint16_t rds[sampleCount];
			for (uint8_t i = 0; i < sampleCount; i++){
				if (i != 0) // For 2. reading, etc. - force new readout
					(*readings)[deviceNumber] = 0;
				uint32_t ms = millis();
				while ((*readings)[deviceNumber] == 0){
					robotContainer->noLoopWithoutThis();
					if (millis() - ms > TIMEOUT){
						robotContainer->errors->add(devices[deviceNumber].canIdIn, ERROR_TIMEOUT, false);
						break;
					}
				}
				rds[i] = (*readings)[deviceNumber];
				//print("Reading %i\n\r", (*readings)[deviceNumber]);
			}

			// Average and standard deviation
			float sum = 0.0;
			for(uint8_t i = 0; i < sampleCount; i++)
				sum += rds[i];
			//print("Sum %i\n\r", (int)sum);
			float mean = sum / sampleCount;
			//print("Mean %i\n\r", (int)mean);
			float standardDeviation = 0.0;
			for(int i = 0; i < sampleCount; i++) 
				standardDeviation += pow(rds[i] - mean, 2);
			standardDeviation = sqrt(standardDeviation / sampleCount);
			//print("SD %i\n\r", (int)standardDeviation);

			// Filter out all the values outside n-sigma boundaries and return average value of the rest
			sum = 0;
			uint8_t cnt = 0;
			//print("Limits: %i %i (%i)\n\r", (int)(mean - sigmaCount * standardDeviation), (int)(mean + sigmaCount * standardDeviation), sigmaCount);
			for (uint8_t i = 0; i < sampleCount; i++)
				if (mean - sigmaCount * standardDeviation < rds[i] && rds[i] < mean + sigmaCount * standardDeviation){
					sum += rds[i];
					cnt++;
				}

			//print("Cnt %i\n\r", cnt);
			return (uint16_t)(sum / cnt);
		}
	else
		return 0;
}

/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
@return - true if canId for this class
*/
bool Mrm_lid_can_b::messageDecode(CANMessage message) {
	for (Device& device : devices)
		if (isForMe(message.id, device)) {
			if (!messageDecodeCommon(message, device)) {
				switch (message.data[0]) {
				case COMMAND_SENSORS_MEASURE_SENDING: {
					uint16_t mm = (message.data[2] << 8) | message.data[1];
					(*readings)[device.number] = mm;
					device.lastReadingsMs = millis();
				}
				break;
				default:
					print("Unknown command. ");
					messagePrint(message, false);
					robotContainer->errors->add(message.id, ERROR_COMMAND_UNKNOWN, false);
				}
			}
			return true;
		}
	return false;
}

/** Enable plug and play
@param enable - enable or disable
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_lid_can_b::pnpSet(bool enable, Device * device){
	if (device == nullptr)
		for (Device& dev : devices)
			pnpSet(enable, &dev);
	else if (device->alive) {
		delay(1);
		canData[0] = enable ? COMMAND_PNP_ENABLE : COMMAND_PNP_DISABLE;
		canData[1] = enable;
		messageSend(canData, 2, device->number);
	}
}

/** Ranging type
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param value - long range 0, high speed 1, high accuracy 2
*/
void Mrm_lid_can_b::rangingType(Device * device, uint8_t value) {
	if (device == nullptr)
		for (Device& dev : devices)
			calibration(&dev);
	else {
		canData[0] = COMMAND_LID_CAN_B_RANGING_TYPE;
		canData[1] = value;
		messageSend(canData, 2, device->number);
	}
}

/** Analog readings
@param receiverNumberInSensor - always 0
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_lid_can_b::reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber){
	return distance(deviceNumber);
}

/** Print all readings in a line
*/
void Mrm_lid_can_b::readingsPrint() {
	print("Lid2m:");
	for (Device& dev : devices)
		if (dev.alive)
			print(" %4i", (*readings)[dev.number]);
}

/** If sensor not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_lid_can_b::started(Device& device) {
	if (millis() - device.lastReadingsMs > MRM_LID_CAN_INACTIVITY_ALLOWED_MS || device.lastReadingsMs == 0) {
		//print("Start mrm-lid-can-b%i \n\r", deviceNumber);
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(&device, 0);
			// Wait for 1. message.
			uint64_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - device.lastReadingsMs < 100) {
					//print("Lidar confirmed\n\r");
					return true;
				}
				delay(1); // Messages are exchanged here
			}
		}
		printf(errorMessage, "%s %i dead.", _boardsName.c_str(), device.number);
		return false;
	}
	else
		return true;
}



/**Test
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
@param betweenTestsMs - time in ms between 2 tests. 0 - default.
*/
void Mrm_lid_can_b::test(uint16_t betweenTestsMs)
{
	static uint32_t lastMs = 0;
	static uint16_t deviceNumber = 0xFF;
	if (millis() - lastMs > (betweenTestsMs == 0 ? 300 : betweenTestsMs)) {
		uint8_t pass = 0;
		for (uint8_t i = 0; i < nextFree; i++) {
			bool isAlive = aliveWithOptionalScan(&devices[i]);
			// print("L%i:%s", i, isAlive ? "Y" : "N"); 
			if (isAlive && (deviceNumber == 0xFF || i == deviceNumber)) {
				if (pass++)
					print(" ");
				print("%4i ", distance(i));
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}