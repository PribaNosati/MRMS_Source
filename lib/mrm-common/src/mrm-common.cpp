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

void Errors::add(uint16_t canId, uint8_t errorCode, bool peripheral) {
    try{
	    errorList.push_back(::Error(canId, errorCode, peripheral));
    } catch (const std::exception& e) {
        snprintf(errorMessage, sizeof(errorMessage), "Error adding to error list: %s", e.what());
        exit(77);
    }
}

void Errors::deleteAll() {
	errorList.clear();
}

/** Displays errors and stops motors, if any.
*/
void Errors::display() {
	for (const ::Error& error: errorList)
		print("% ms, id: 0x%02X, %s. err. %i\n\r", error.time, error.canId,  (error.peripheral ? ", periph." : ", local"), error.errorCode);
}