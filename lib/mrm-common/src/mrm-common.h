#pragma once
#define RADIO 1 // 0 - no radio, 1 Bluetooth, 2 WiFi
#include <Arduino.h>
#include <map>
#include <string>
#include <vector>

extern char errorMessage[60];


/** Angle between -180 and 180 degrees
@return - angle
*/
float angleNormalized(float angle);

/** Filter out data outliers and return average of the rest
@param sampleCount - count.
@param sample - values.
@param averageValue - average value.
@param sigmaCount - number of sigmas to keep.
@param standardDeviation - standard deviation.
@return average value of the filtered set*/
float outlierlessAverage(uint8_t sampleCount, uint16_t sample[], float averageValue, uint8_t sigmaCount,
    float standardDeviation);

/** Print to all serial ports
@param fmt - C format string
@param ... - variable arguments
*/
void print(const char* fmt, ...);

/** Standard deviation
@param sampleCount - count.
@param sample - values.
@param averageValue - output parameter.
@return - standard deviation.*/
float stardardDeviation(uint8_t sampleCount, uint16_t sample[], float * averageValue);

void startBT(const char* name);

/** Print to all serial ports, pointer to list
*/
void vprint(const char* fmt, va_list argp);

// Errors
#define ERROR_COMMAND_UNKNOWN 0x00
#define ERROR_DLC_TOO_BIG 0x01
#define ERROR_TIMEOUT 0x02

struct Error {
    uint32_t time;
    uint16_t canId;
    uint8_t errorCode;
    bool peripheral;
    static std::map<uint8_t, std::string> errorNames;
    Error(uint16_t canId, uint8_t errorCode, bool peripheral) {
        this->time = millis();
        this->canId = canId;
        this->errorCode = errorCode;
        this->peripheral = peripheral;
    }
};

class Errors{
	std::vector<::Error> errorList;
public:
	void add(uint16_t canId, uint8_t errorCode, bool peripheral);
	void deleteAll();
	void display();
    static std::string find(uint8_t code){
		auto it = Error::errorNames.find(code);
		if (it == Error::errorNames.end())
			return "Unknown error code " + std::to_string(code);
		else
			return it->second;
	}
};