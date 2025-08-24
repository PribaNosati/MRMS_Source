#pragma once
#define RADIO 1 // 0 - no radio, 1 Bluetooth, 2 WiFi
#include <Arduino.h>

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