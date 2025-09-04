#pragma once
#include <vector>
#include <map>
#include <string>
#include <numeric>

class CANMessage;
struct Device;

class DistanceInterface
{
private:

    /** If sensor not started, start it and wait for 1. message
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - started or not
	*/
	virtual bool started(Device& device) = 0;

public:

	/** Add a mrm-ref-can sensor
	@param deviceName - device's name
	*/
	virtual void add(char * deviceName = (char*)"") = 0;

	/** Calibration, only once after production
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	virtual void calibration(Device * device = nullptr) = 0;

	virtual std::string commandName(uint8_t byte) = 0;

    /** Distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param sampleCount - Number or readings. 40% of the c, with extreme values, will be discarded and the
					rest will be averaged. Keeps returning 0 till all the sample is read.
					If sampleCount is 0, it will not wait but will just return the last value.
	@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
					Therefore, lower sigma number will remove more errornous readings.
	@return - distance in mm
	*/
	virtual uint16_t distance(uint8_t deviceNumber = 0, uint8_t sampleCount = 0, uint8_t sigmaCount = 1) = 0;

	/** Read CAN Bus message into local variables
	@param canId - CAN Bus id
	@param data - 8 bytes from CAN Bus message.
	*/
	virtual bool messageDecode(CANMessage message) = 0;

    /** Enable plug and play
	@param enable - enable or disable
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	*/
	virtual void pnpSet(bool enable = true, Device * device = nullptr) = 0;

	/** Ranging type
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@param value - long range 0, high speed 1, high accuracy 2
	*/
	virtual void rangingType(Device * device = nullptr, uint8_t value = 0) = 0;

	/** Analog readings
	@param receiverNumberInSensor - always 0
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
	@return - analog value
	*/
	virtual uint16_t reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber = 0) = 0;

	/** Print all readings in a line
	*/
	virtual void readingsPrint() = 0;

	/**Test
	@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all devices.
	@param betweenTestsMs - time in ms between 2 tests. 0 - default.
	*/
	virtual void test(uint16_t betweenTestsMs = 0) = 0;
};
