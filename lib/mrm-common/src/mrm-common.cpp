#include <mrm-common.h>
#include <math.h>
#if RADIO == 1
#include <BluetoothSerial.h>
#endif
char errorMessage[60] = ""; // Global variable enables functions to set it although not passed as parameter
#if RADIO == 1
BluetoothSerial *serialBT = NULL;
#endif

/** Angle between -180 and 180 degrees
@return - angle
*/
float angleNormalized(float angle) {
	if (angle < -180)
		angle += 360;
	else if (angle > 180)
		angle -= 360;
	return angle;
}

/** Filter out data outliers and return average of the rest
@param sampleCount - count.
@param sample - values.
@param averageValue - average value.
@param sigmaCount - number of sigmas to keep.
@param standardDeviation - standard deviation.
@return average value of the filtered set*/
float outlierlessAverage(uint8_t sampleCount, uint16_t sample[], float averageValue, uint8_t sigmaCount,
	float standardDeviation){
	// Filter out all the values outside n-sigma boundaries and return average value of the rest
	float sum = 0;
	uint8_t cnt = 0;
	for (uint8_t i = 0; i < sampleCount; i++)
		if (averageValue - sigmaCount * standardDeviation < sample[i] && 
				sample[i] < averageValue + sigmaCount * standardDeviation){
			sum += sample[i];
			cnt++;
		}
	return (uint16_t)(sum / cnt);
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

/** Standard deviation
@param sampleCount - count.
@param sample - values.
@param averageValue - output parameter.
@return - standard deviation.*/
float stardardDeviation(uint8_t sampleCount, uint16_t sample[], float * averageValue){
				// Average and standard deviation
			float sum = 0.0;
			for(uint8_t i = 0; i < sampleCount; i++)
				sum += sample[i];
			//print("Sum %i\n\r", (int)sum);
			*averageValue = sum / sampleCount;
			//print("Mean %i\n\r", (int)mean);
			float sd = 0.0;
			for(int i = 0; i < sampleCount; i++) 
				sd += pow(sample[i] - *averageValue, 2);
			sd = sqrt(sd / sampleCount);
			//print("SD %i\n\r", (int)standardDeviation);
			return sd;
}

void startBT(const char* name){
	#if RADIO == 1
	if (serialBT == NULL) {
		serialBT = new BluetoothSerial(); // Additional serial port
		serialBT->begin(name); //Start Bluetooth. ESP32 - Bluetooth device name, choose one.
	}
#endif
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