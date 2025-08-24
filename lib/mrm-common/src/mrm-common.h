#pragma once
#define RADIO 1 // 0 - no radio, 1 Bluetooth, 2 WiFi
#include <Arduino.h>
#include <vector>

extern char errorMessage[60];

void startBT(const char* name);

/** Print to all serial ports
@param fmt - C format string
@param ... - variable arguments
*/
void print(const char* fmt, ...);

/** Print to all serial ports, pointer to list
*/
void vprint(const char* fmt, va_list argp);

struct Error {
    uint32_t time;
    uint16_t canId;
    uint8_t errorCode;
    bool peripheral;
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
};