#include <mrm-common.h>
#if RADIO == 1
#include <BluetoothSerial.h>
#endif
char errorMessage[60] = ""; // Global variable enables functions to set it although not passed as parameter
#if RADIO == 1
BluetoothSerial *serialBT = NULL;
#endif

void startBT(const char* name){
	#if RADIO == 1
	if (serialBT == NULL) {
		serialBT = new BluetoothSerial(); // Additional serial port
		serialBT->begin(name); //Start Bluetooth. ESP32 - Bluetooth device name, choose one.
	}
#endif
}

/** Print to all serial ports
@param fmt - C format string
@param ... - variable arguments
*/
void print(const char* fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}

/** Print to all serial ports, pointer to list
*/
void vprint(const char* fmt, va_list argp) {
	if (strlen(fmt) >= 100)
		return;
	static char buffer[100];
	vsprintf(buffer, fmt, argp);

    printf(buffer);
	if (serialBT != NULL)
		serialBT->print(buffer);
}