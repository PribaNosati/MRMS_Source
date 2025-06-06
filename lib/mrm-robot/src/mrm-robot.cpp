#include <mrm-board.h>
#include <mrm-8x8a.h>
#include <mrm-bldc2x50.h>
#include <mrm-bldc4x2.5.h>
#include <mrm-can-bus.h>
#include <mrm-col-b.h>
#include <mrm-col-can.h>
#include <mrm-fet-can.h>
#include <mrm-imu.h>
#include <mrm-pid.h>
//#include <mrm-ir-finder2.h>
//#include <mrm-ir-finder-can.h>
#include <mrm-ir-finder3.h>
#include <mrm-lid-can-b.h>
#include <mrm-lid-can-b2.h>
#include <mrm-lid-d.h>
#include <mrm-mot2x50.h>
#include <mrm-mot4x10.h>
#include <mrm-mot4x3.6can.h>
#include <mrm-node.h>
#include <mrm-ref-can.h>
#include <mrm-robot.h>
#include <mrm-servo.h>
#include <mrm-switch.h>
#include <mrm-therm-b-can.h>
// #include <mrm-us.h>
#include <mrm-us-b.h>
#include <mrm-us1.h>
#include <BluetoothSerial.h>

#if RADIO == 1
extern BluetoothSerial* serialBT;
#endif

/**
*/
Robot::Robot(char name[15], char ssid[15], char wiFiPassword[15]) {
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
	Serial.begin(115200);

	if (strlen(name) > 15)
		strcpy(errorMessage, "Name overflow");
	strcpy(_name, name);

	if (strlen(ssid) > 15)
		strcpy(errorMessage, "SSID overflow");
	strcpy(_ssid, ssid);

	if (strlen(wiFiPassword) > 15)
		strcpy(errorMessage, "WiFi pwd. overflow");
	strcpy(_wiFiPassword, wiFiPassword);
	boardInfo = new BoardInfo();

	// EEPROM, data retained after system powered down
	preferences = new Preferences();
	preferences->begin("data", false);

	startBT(_name);

	delay(50);
	print("%s started.\r\n", _name);

	Wire.begin(); // Start I2C

#if RADIO == 2
	delay(100);
	webServer = new WiFiServer(80);

	  // Connect to Wi-Fi network with SSID and password
	print("Connecting to %s", ssid);
	WiFi.begin(ssid, wiFiPassword);
	uint32_t startMs = millis();
	bool ok = true;
	while (WiFi.status() != WL_CONNECTED) {
		delay(200);
		print(".");
		if (millis() - startMs > 2000){
			ok = false;
			break;
		}
	}
	if (ok){
		// Print local IP address and start web server
		print("\n\r");
		print("WiFi connected.\n\r");
		print("IP address: %i.%i.%i.%i\n\r", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
		webServer->begin();
	}
	else
		print("\n\rWeb server not started.\n\r");
#endif

	mrm_can_bus = new Mrm_can_bus();

	// LED Test
	Mrm_8x8a::LEDSignText* signTest = new Mrm_8x8a::LEDSignText();
	strcpy(signTest->text, "Test");

	_actionCurrent = NULL;
	_actionPrevious = _actionCurrent;

	// Actions that will be called from code, so it is necessary to assign them to objects. 
	// Also, if an action is to be assigned to a button, it will have to be defined here.
	_actionDoNothing = new ActionRobot(this, "", "No action", 0, Board::BoardId::ID_ANY, NULL, NULL);
	_actionLoop = new ActionRobot(this, "loo", "Loop", 8, Board::BoardId::ID_ANY, signTest, &Robot::loop);
	_actionLoop0 = new ActionRobot(this, "lo0", "Loop 0", 8, Board::BoardId::ID_ANY, signTest, &Robot::loop0);
	_actionLoop1 = new ActionRobot(this, "lo1", "Loop 1", 8, Board::BoardId::ID_ANY, signTest, &Robot::loop1);
	_actionLoop2 = new ActionRobot(this, "lo2", "Loop 2", 8, Board::BoardId::ID_ANY, signTest, &Robot::loop2);
	_actionLoop3 = new ActionRobot(this, "lo3", "Loop 3", 8, Board::BoardId::ID_ANY, signTest, &Robot::loop3);
	_actionLoop4 = new ActionRobot(this, "lo4", "Loop 4", 8, Board::BoardId::ID_ANY, signTest, &Robot::loop4);
	_actionStop = new ActionRobot(this, "sto", "Stop", 1, Board::BoardId::ID_ANY, signTest, &Robot::stopAll);

	// Each object must be added to the collection of actions if we want to display them in menu
	actionAdd(_actionLoop);
	actionAdd(_actionLoop0);
	actionAdd(_actionLoop1);
	actionAdd(_actionLoop2);
	actionAdd(_actionLoop3);
	actionAdd(_actionLoop4);
	actionAdd(_actionStop);

	// Actions for menu only, therefore no (non-anonymus) objects needed.
	actionAdd(new ActionRobot(this, "all", "CAN Bus stress", 16, Board::BoardId::ID_ANY, signTest, &Robot::stressTest));///1 | 2 | 4 | 8 | 16 | 32 | 64 | 128-> in all menus. 0 - in no menu.
	actionAdd(new ActionRobot(this, "led", "Test 8x8", 1, Board::BoardId::ID_MRM_8x8A, signTest, &Robot::led8x8Test));
	actionAdd(new ActionRobot(this, "blt", "Test Bluetooth", 16, Board::BoardId::ID_ANY, signTest, &Robot::bluetoothTest));
	actionAdd(new ActionRobot(this, "can", "Report devices", 16, Board::BoardId::ID_ANY, signTest, &Robot::devicesScan));
	actionAdd(new ActionRobot(this, "ca1", "Report device", 16, Board::BoardId::ID_ANY, signTest, &Robot::deviceScan));
	actionAdd(new ActionRobot(this, "sni", "Sniff bus toggle", 16, Board::BoardId::ID_ANY, signTest, &Robot::canBusSniffToggle));
	actionAdd(new ActionRobot(this, "10c", "Test 10 colors", 4, Board::BoardId::ID_MRM_COL_B, signTest, &Robot::colorTest10));
	actionAdd(new ActionRobot(this, "hsv", "Test HSV", 4, Board::BoardId::ID_MRM_COL_B, signTest, &Robot::colorTestHSV));
	actionAdd(new ActionRobot(this, "lof", "Light off", 4, Board::BoardId::ID_MRM_COL_CAN, signTest, &Robot::colorIlluminationOff));
	actionAdd(new ActionRobot(this, "lon", "Light on", 4, Board::BoardId::ID_MRM_COL_CAN, signTest, &Robot::colorIlluminationOn));
	actionAdd(new ActionRobot(this, "per", "Erase patterns", 4, Board::BoardId::ID_MRM_COL_CAN, signTest, &Robot::colorPatternErase));
	actionAdd(new ActionRobot(this, "ppr", "Print patterns", 4, Board::BoardId::ID_MRM_COL_CAN, signTest, &Robot::colorPatternPrint));
	actionAdd(new ActionRobot(this, "pre", "Recognize patern", 4, Board::BoardId::ID_MRM_COL_CAN, signTest, &Robot::colorPatternRecognize));
	actionAdd(new ActionRobot(this, "par", "Record patterns", 4, Board::BoardId::ID_MRM_COL_CAN, signTest, &Robot::colorPatternRecord));
	actionAdd(new ActionRobot(this, "6co", "Test 6 colors", 4, Board::BoardId::ID_MRM_COL_CAN, signTest, &Robot::colorTest6));
	actionAdd(new ActionRobot(this, "hsv", "Teset HSV", 4, Board::BoardId::ID_MRM_COL_CAN, signTest, &Robot::colorTest6HSV));
	actionAdd(new ActionRobot(this, "idc", "Dev. id change", 16, Board::BoardId::ID_ANY, signTest, &Robot::canIdChange));
	actionAdd(new ActionRobot(this, "sca", "CAN scan toggl", 16, Board::BoardId::ID_ANY, signTest, &Robot::canScanToggle));
	actionAdd(new ActionRobot(this, "fir", "Firmware", 16, Board::BoardId::ID_ANY, signTest, &Robot::firmwarePrint));
	actionAdd(new ActionRobot(this, "fps", "FPS", 16, Board::BoardId::ID_ANY, signTest, &Robot::fpsPrint));
	actionAdd(new ActionRobot(this, "ahe", "Go ahead", 1, Board::BoardId::ID_ANY, signTest, &Robot::goAhead));
	actionAdd(new ActionRobot(this, "i2c", "Test I2C", 8, Board::BoardId::ID_ANY, signTest, &Robot::i2cTest));
	actionAdd(new ActionRobot(this, "imu", "Test IMU", 1, Board::BoardId::ID_ANY, signTest, &Robot::imuTest));
	actionAdd(new ActionRobot(this, "inf", "Info", 8, Board::BoardId::ID_ANY, signTest, &Robot::info));
	actionAdd(new ActionRobot(this, "irf", "Test ball analog", 1, Board::BoardId::ID_MRM_IR_FINDER3, signTest, &Robot::irFinderTest));
	actionAdd(new ActionRobot(this, "irc", "Test bal calcul.", 1, Board::BoardId::ID_MRM_IR_FINDER3, signTest, &Robot::irFinderTestCalculated));
	actionAdd(new ActionRobot(this, "li2", "Test li. 2m", 1, Board::BoardId::ID_MRM_LID_CAN_B, signTest, &Robot::lidar2mTest));
	actionAdd(new ActionRobot(this, "li4", "Test li 4m", 1, Board::BoardId::ID_MRM_LID_CAN_B2, signTest, &Robot::lidar4mTest));
	actionAdd(new ActionRobot(this, "lim", "Test li. mul.", 1, Board::BoardId::ID_MRM_LID_D, signTest, &Robot::lidar4mMultiTest));
	actionAdd(new ActionRobot(this, "lic", "Calibrate lidar", 1, Board::BoardId::ID_ANY, signTest, &Robot::lidarCalibrate));
	actionAdd(new ActionRobot(this, "col", "Color (menu)", 1, Board::BoardId::ID_MRM_COL_CAN, signTest, &Robot::menuColor));
	actionAdd(new ActionRobot(this, "col", "Color (menu)", 1, Board::BoardId::ID_MRM_COL_B, signTest, &Robot::menuColor));
	actionAdd(new ActionRobot(this, "x", "Escape", 1 | 2 | 4 | 8 | 16, Board::BoardId::ID_ANY, signTest, &Robot::menuMainAndIdle)); //2 | 4 | 8 | 16 -> in all menus except 1. 0 - in no menu.
	actionAdd(new ActionRobot(this, "ref", "Reflectance (menu)", 1, Board::BoardId::ID_MRM_REF_CAN, signTest, &Robot::menuReflectance));
	actionAdd(new ActionRobot(this, "sys", "System (menu)", 1, Board::BoardId::ID_ANY, signTest, &Robot::menuSystem));
	actionAdd(new ActionRobot(this, "mot", "Test motors", 1, Board::BoardId::ID_ANY, signTest, &Robot::motorTest));
	actionAdd(new ActionRobot(this, "nod", "Test node", 1, Board::BoardId::ID_ANY, signTest, &Robot::nodeTest));
	actionAdd(new ActionRobot(this, "nos", "Test node servo", 1, Board::BoardId::ID_MRM_NODE, signTest, &Robot::nodeServoTest));
	actionAdd(new ActionRobot(this, "pof", "PnP off", 16, Board::BoardId::ID_ANY, signTest, &Robot::pnpOff));
	actionAdd(new ActionRobot(this, "pon", "PnP on", 16, Board::BoardId::ID_ANY, signTest, &Robot::pnpOn));
	actionAdd(new ActionRobot(this, "cal", "Calibrate ref.", 2, Board::BoardId::ID_MRM_REF_CAN, signTest, &Robot::reflectanceArrayCalibrate));
	actionAdd(new ActionRobot(this, "pri", "Calibration printrint", 2, Board::BoardId::ID_MRM_REF_CAN, signTest, &Robot::reflectanceArrayCalibrationPrint));
	actionAdd(new ActionRobot(this, "anr", "Test refl. anal.", 2, Board::BoardId::ID_MRM_REF_CAN, signTest, &Robot::reflectanceArrayTestAnalog));
	actionAdd(new ActionRobot(this, "dgr", "Test refl. dig.", 2, Board::BoardId::ID_MRM_REF_CAN, signTest, &Robot::reflectanceArrayTestDigital));
	actionAdd(new ActionRobot(this, "ses", "Set servo", 1, Board::BoardId::ID_ANY, signTest, &Robot::servoInteractive));
	actionAdd(new ActionRobot(this, "ser", "Test servo", 1, Board::BoardId::ID_ANY, signTest, &Robot::servoTest));
	actionAdd(new ActionRobot(this, "the", "Test thermo", 1, Board::BoardId::ID_MRM_THERM_B_CAN, signTest, &Robot::thermoTest));
	actionAdd(new ActionRobot(this, "uls", "Test ultras.", 1, Board::BoardId::ID_MRM_US_B, signTest, &Robot::usBTest));
	actionAdd(new ActionRobot(this, "ult", "Test ultras.", 1, Board::BoardId::ID_MRM_US1, signTest, &Robot::us1Test));
	actionAdd(new ActionRobot(this, "lme", "Loop (menu)", 1, Board::BoardId::ID_ANY, signTest, &Robot::menuLoop));

	mrm_8x8a = new Mrm_8x8a(this);
	mrm_bldc2x50 = new Mrm_bldc2x50(this);
	mrm_bldc4x2_5 = new Mrm_bldc4x2_5(this);
	mrm_col_b = new Mrm_col_b(this);
	mrm_col_can = new Mrm_col_can(this);
	mrm_fet_can = new Mrm_fet_can(this);
	mrm_imu = new Mrm_imu(this);
	//mrm_ir_finder2 = new Mrm_ir_finder2(this);
	// mrm_ir_finder_can = new Mrm_ir_finder_can(this);
	mrm_ir_finder3 = new Mrm_ir_finder3(this);
	mrm_lid_can_b = new Mrm_lid_can_b(this);
	mrm_lid_can_b2 = new Mrm_lid_can_b2(this);
	mrm_lid_d = new Mrm_lid_d(this);
	mrm_mot2x50 = new Mrm_mot2x50(this);
	mrm_mot4x3_6can = new Mrm_mot4x3_6can(this);
	mrm_mot4x10 = new Mrm_mot4x10(this);
	mrm_node = new Mrm_node(this);
	mrm_ref_can = new Mrm_ref_can(this);
	mrm_servo = new Mrm_servo(this);
	mrm_switch = new Mrm_switch(this);
	mrm_therm_b_can = new Mrm_therm_b_can(this);
	// mrm_us = new Mrm_us(this);
	mrm_us_b = new Mrm_us_b(this);
	mrm_us1 = new Mrm_us1(this);

	// 8x8 LED
	mrm_8x8a->add((char*)"LED8x8-0");
	// Motors mrm-bldc2x50
	mrm_bldc2x50->add(false, (char*)"BL2x50-0");
	mrm_bldc2x50->add(false, (char*)"BL2x50-1");
	mrm_bldc2x50->add(false, (char*)"BL2x50-2");
	mrm_bldc2x50->add(false, (char*)"BL2x50-3");

	// LEDs
	pinMode(2, OUTPUT);
	digitalWrite(2, false);
	pinMode(LED_ERROR, OUTPUT); // Error LED
	digitalWrite(15, false);

	// Motors mrm-bldc4x2.5
	mrm_bldc4x2_5->add(false, (char*)"BL4x2.5-0");
	mrm_bldc4x2_5->add(false, (char*)"BL4x2.5-1");
	mrm_bldc4x2_5->add(false, (char*)"BL4x2.5-2");
	mrm_bldc4x2_5->add(false, (char*)"BL4x2.5-3");

	// Colors sensors mrm-col-b
	mrm_col_b->add((char*)"Clr-0");
	mrm_col_b->add((char*)"Clr-1");

	// Colors sensors mrm-col-can
	mrm_col_can->add((char*)"Col-0");
	mrm_col_can->add((char*)"Col-1");
	mrm_col_can->add((char*)"Col-2");
	mrm_col_can->add((char*)"Col-3");

	// FET outputs
	mrm_fet_can->add((char*)"FET-0");

	// IMU
	mrm_imu->add();

	// // mrm-ir-finder2
	// mrm_ir_finder3->add(34, 33);

	// // mrm-ir-finder-can
	// mrm_ir_finder_can->add((char*)"IRFind-0");

	// mrm-ir-finder3
	mrm_ir_finder3->add((char*)"IR3Fin-0");

	// Motors mrm-mot2x50
	mrm_mot2x50->add(false, (char*)"Mot2x50-0");
	mrm_mot2x50->add(false, (char*)"Mot2x50-1");
	mrm_mot2x50->add(false, (char*)"Mot2x50-2");
	mrm_mot2x50->add(false, (char*)"Mot2x50-3");
	mrm_mot2x50->add(false, (char*)"Mot2x50-4");
	mrm_mot2x50->add(false, (char*)"Mot2x50-5");

	// Motors mrm-mot4x10
	mrm_mot4x10->add(false, (char*)"Mot4x10-0");
	mrm_mot4x10->add(false, (char*)"Mot4x10-1");
	mrm_mot4x10->add(false, (char*)"Mot4x10-2");
	mrm_mot4x10->add(false, (char*)"Mot4x10-3");

	// Motors mrm-mot4x3.6can
	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-0");
	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-1");
	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-2");
	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-3");

	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-4");
	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-5");
	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-6");
	mrm_mot4x3_6can->add(false, (char*)"Mot3.6-7");

	// Lidars mrm-lid-can-b, VL53L0X, 2 m
	mrm_lid_can_b->add((char*)"Lidar2m-0");
	mrm_lid_can_b->add((char*)"Lidar2m-1");
	mrm_lid_can_b->add((char*)"Lidar2m-2");
	mrm_lid_can_b->add((char*)"Lidar2m-3");
	mrm_lid_can_b->add((char*)"Lidar2m-4");
	mrm_lid_can_b->add((char*)"Lidar2m-5");
	mrm_lid_can_b->add((char*)"Lidar2m-6");
	mrm_lid_can_b->add((char*)"Lidar2m-7");
	mrm_lid_can_b->add((char*)"Lidar2m-8");
	mrm_lid_can_b->add((char*)"Lidar2m-9");
	mrm_lid_can_b->add((char*)"Lidar2m10");
	mrm_lid_can_b->add((char*)"Lidar2m11");
	mrm_lid_can_b->add((char*)"Lidar2m12");
	mrm_lid_can_b->add((char*)"Lidar2m13");

	// Lidars mrm-lid-can-b2, VL53L1X, 4 m
	mrm_lid_can_b2->add((char*)"Lidar4m-0");
	mrm_lid_can_b2->add((char*)"Lidar4m-1");
	mrm_lid_can_b2->add((char*)"Lidar4m-2");
	mrm_lid_can_b2->add((char*)"Lidar4m-3");
	mrm_lid_can_b2->add((char*)"Lidar4m-4");
	mrm_lid_can_b2->add((char*)"Lidar4m-5");
	mrm_lid_can_b2->add((char*)"Lidar4m-6");
	mrm_lid_can_b2->add((char*)"Lidar4m-7");

	// Lidars mrm-lid-can-b2, VL53L5X, 4 m multi
	mrm_lid_d->add((char*)"LidMul-0");
	// mrm_lid_d->add((char*)"LidMul-1");

	// CAN Bus node
	mrm_node->add((char*)"Node-0");
	mrm_node->add((char*)"Node-1");

	// Reflective array
	mrm_ref_can->add((char*)"RefArr-0");
	mrm_ref_can->add((char*)"RefArr-1");
	mrm_ref_can->add((char*)"RefArr-2");
	mrm_ref_can->add((char*)"RefArr-3");
	mrm_ref_can->add((char*)"RefArr-4");

	// Switch
	mrm_switch->add(18, 19, (char*)"Switch");

	// Thermal array
	mrm_therm_b_can->add((char*)"Thermo-0");
	mrm_therm_b_can->add((char*)"Thermo-1");
	mrm_therm_b_can->add((char*)"Thermo-2");
	mrm_therm_b_can->add((char*)"Thermo-3");

	// Ultrasonic
	mrm_us_b->add((char*)"US-B-0");
	mrm_us_b->add((char*)"US-B-1");
	mrm_us_b->add((char*)"US-B-2");
	mrm_us_b->add((char*)"US-B-3");
	mrm_us_b->add((char*)"US-B-4");

	mrm_us1->add((char*)"US1-0");
	// mrm_us->add((char*)"US-0");
	// mrm_us->add((char*)"US-1");
	// mrm_us->add((char*)"US-2");
	// mrm_us->add((char*)"US-3");

	// Add boards
	add(mrm_8x8a);
	add(mrm_bldc2x50);
	add(mrm_bldc4x2_5);
	add(mrm_col_b);
	add(mrm_col_can);
	add(mrm_fet_can);
	add(mrm_ir_finder3);
	add(mrm_lid_can_b);
	add(mrm_lid_can_b2);
	add(mrm_lid_d);
	add(mrm_mot2x50);
	add(mrm_mot4x10);
	add(mrm_mot4x3_6can);
	add(mrm_node);
	add(mrm_ref_can);
	add(mrm_therm_b_can);
	// add(mrm_us);
	add(mrm_us_b);
	add(mrm_us1);

	_devicesAtStartup =  _devicesScanOnStartup ? devicesScan(true) : 0;
	devicesLEDCount();
}

/** Add a new action to the collection of robot's possible actions.
@param action - the new action.
*/
void Robot::actionAdd(ActionBase* action) {
	if (_actionNextFree >= ACTIONS_LIMIT) {
		strcpy(errorMessage, "ACTIONS_LIMIT exceeded.");
		return;
	}
	_action[_actionNextFree++] = action;
}

/** Is this current action's initialization
@param andFinish - finish initialization
@return - it is.
*/
bool Robot::actionPreprocessing(bool andFinish) {
	bool itIs = _actionCurrent->preprocessing();
	if (andFinish)
		_actionCurrent->preprocessingEnd();
	return itIs;
}

/** Finish action's intialization phase
*/
void Robot::actionPreprocessingEnd() { 
	_actionCurrent->preprocessingEnd(); 
}

/** Actually perform the action
*/
void Robot::actionProcess() {
	if (_actionCurrent != NULL) {
		if (_actionCurrent->preprocessing()) {
			_actionCurrent->performBefore();
		}
		_actionCurrent->perform();
		//if (_actionCurrent != NULL)
		//	_actionCurrent->_preprocessing = false;
	}
}

/** User sets a new action, using keyboard or Bluetooth
*/
void Robot::actionSet() {
	static uint32_t lastUserActionMs = 0;
	const uint16_t TIMEOUT_MS = 2000;

	// If a button pressed, first execute its action
	ActionBase* action8x8 = NULL;
	if (mrm_8x8a->alive() && _devicesScanBeforeMenuAndSwitches) // Disable message if CAN silence needed
		action8x8 = mrm_8x8a->actionCheck(); 
	ActionBase* actionSw = mrm_switch->actionCheck(); 
	if (action8x8 != NULL)
		actionSet(action8x8);
	else if (actionSw != NULL)
		actionSet(actionSw);
	else { // Check keyboard
		bool btAvailable = false;
#if RADIO == 1
		btAvailable = serialBT != NULL && serialBT->available();
#endif
		if (Serial.available() || btAvailable) {
			lastUserActionMs = millis();
			uint8_t ch = ' ';
			if (Serial.available())
				ch = Serial.read();
#if RADIO == 1
			else
				if (serialBT != NULL)
					ch = serialBT->read();
#endif

			if (ch != 13 && uartRxCommandIndex < 23) //if received data different from ascii 13 (enter)
				uartRxCommandCumulative[uartRxCommandIndex++] = ch;	//add data to Rx_Buffer

			if (ch == 13 || (uartRxCommandIndex >= 3 && !(uartRxCommandCumulative[0] == 'e' && uartRxCommandCumulative[1] == 's' && uartRxCommandCumulative[2] == 'c')) || ch == 'x' && uartRxCommandIndex == 1) //if received data = 13
			{

				print("Command: %s", uartRxCommandCumulative);

				uint8_t found = 0;
				uartRxCommandCumulative[uartRxCommandIndex] = '\0';
				if (uartRxCommandCumulative[0] == 'e' && uartRxCommandCumulative[1] == 's' && uartRxCommandCumulative[2] == 'c'){
					found = 1;
				}
				else{
					uartRxCommandCumulative[uartRxCommandIndex] = 0;
					uartRxCommandIndex = 0;
				}

				for (uint8_t i = 0; i < _actionNextFree; i++) {
					if (strcmp(_action[i]->_shortcut, uartRxCommandCumulative) == 0) {
						print(" ok.\r\n");
						actionSet(_action[i]);
						//commandPrevious = commandCurrent;
						//commandCurrent = commands[i];
						//commandCurrent->firstProcess = true;
						found = 1;
						break;
					}
				}
				if (!found) {
					print(" not found.\r\n");
					uartRxCommandIndex = 0;
				}
			}
		}

		if (uartRxCommandIndex != 0 && millis() - lastUserActionMs > TIMEOUT_MS) {
			print(" Timeout.\r\n");
			uartRxCommandIndex = 0;
		}
	}
}

/** New action is set in the program
@param newAction - the new action.
*/
void Robot::actionSet(ActionBase* newAction) {
	_actionPrevious = _actionCurrent;
	_actionCurrent = newAction;
	_actionCurrent->preprocessingStart();
	// Display action on 8x8 LED
	if (mrm_8x8a->alive() && _actionTextDisplay){
		if (_actionCurrent->ledSign == NULL)
			devicesLEDCount();
		else if (_actionCurrent->ledSign->type == 1 && strcmp(((Mrm_8x8a::LEDSignText*)(_actionCurrent->ledSign))->text, "") != 0)
			mrm_8x8a->text(((Mrm_8x8a::LEDSignText*)(_actionCurrent->ledSign))->text);
		else if (_actionCurrent->ledSign->type == 0){
			mrm_8x8a->bitmapCustomDisplay(
				((Mrm_8x8a::LEDSignBitmap*)_actionCurrent->ledSign)->red, 
				((Mrm_8x8a::LEDSignBitmap*)_actionCurrent->ledSign)->green);
		}
	}
}

/** Add a new board to the collection of possible boards for the robot
@param aBoard - the board.
*/
void Robot::add(Board* aBoard) {
	if (_boardNextFree > BOARDS_LIMIT - 1) {
		strcpy(errorMessage, "Too many boards");
		return;
	}
	board[_boardNextFree++] = aBoard;
}

/** Blink LED
*/
void Robot::blink() {
	const uint16_t onMs = 100;
	const uint16_t offMs = 1000;
	uint8_t repeatOnTimes;
	static uint32_t lastBlinkMs = 0;
	static uint8_t isOn = 0;
	static uint8_t pass = 0;

	if (strcmp(errorMessage, "") == 0)
		repeatOnTimes = 1;
	else
		repeatOnTimes = 2;

	if (pass < repeatOnTimes) {
		if (millis() - lastBlinkMs > onMs) {
			isOn = !isOn;
			if (!isOn)
				pass++;
			digitalWrite(LED_OK, isOn);
			lastBlinkMs = millis();
		}
	}
	else if (millis() - lastBlinkMs > offMs) {
		pass = 0;
		lastBlinkMs = 0;
	}
}

/** Displays all boards
@return - last board and device's index, 0 if none
*/
uint8_t Robot::boardsDisplayAll() {
	// Print all devices alive
	uint8_t last = 0;
	for (uint8_t boardNumber = 0; boardNumber < _boardNextFree; boardNumber++) {
		uint8_t currentCount = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < board[boardNumber]->deadOrAliveCount(); deviceNumber++)
			if (board[boardNumber]->alive(deviceNumber)) {
				if (currentCount == 0)
					print("%i.", ++last);
				else
					print(",");
				print(" %s", board[boardNumber]->name(deviceNumber));
				if (++currentCount == board[boardNumber]->devicesOnASingleBoard()) {
					currentCount = 0;
					print("\n\r");
				}
			}
	}
	if (last == 0)
		print("No boards\n\r");
	return last;
}

/** Finds board and device's index. Similar to next function, but display choices, too.
@param selectedBoardIndex - output
@param selectedDeviceIndex - otuput
@param maxInput - output
@param lastBoardAndIndex - output
@return - true if found
*/
bool Robot::boardDisplayAndSelect(uint8_t *selectedBoardIndex, uint8_t* selectedDeviceIndex, uint8_t* maxInput, uint8_t* lastBoardAndIndex) {
	*lastBoardAndIndex = boardsDisplayAll();
	bool found = false;
	if (*lastBoardAndIndex > 0) {

		// Choose device
		print("Enter board [1 - %i]: ", *lastBoardAndIndex);
		uint16_t selectedNumber = serialReadNumber(15000, 500, *lastBoardAndIndex <= 9, *lastBoardAndIndex);
		print("%i", selectedNumber);

		if (selectedNumber != 0xFFFF)
			found = boardSelect(selectedNumber, selectedBoardIndex, selectedDeviceIndex, maxInput);

	}
	return found;
}

bool Robot::boardIdentify(uint32_t canId, bool out, Board** boardFound, int& index){
	for (uint8_t i = 0; i < _boardNextFree; i++) {
		for (uint8_t j = 0; j < board[i]->deadOrAliveCount(); j++) 
			if (out ? board[i]->isFromMe(canId, j) : board[i]->isForMe(canId, j)){
				*boardFound = board[i];
				index = j;
				return true;
			}
	}
	return false;
}

/** Finds board and device's index for a number received from boardsDisplayAll()
@param selectedNumber - input
@param selectedBoardIndex - output
@param selectedDeviceIndex - otuput
@param maxInput - output
@return - true if found
*/
bool Robot::boardSelect(uint8_t selectedNumber, uint8_t *selectedBoardIndex, uint8_t * selectedDeviceIndex, uint8_t *maxInput) {
	// Find selected board
	uint8_t lastBoardAndIndex = 0;
	*selectedBoardIndex = 0;
	*selectedDeviceIndex = 0xFF;
	*maxInput = 0;
	for (uint8_t boardNumber = 0; boardNumber < _boardNextFree && *selectedDeviceIndex == 0xFF; boardNumber++) {
		uint8_t currentCount = 0;
		for (uint8_t deviceNumber = 0; deviceNumber < board[boardNumber]->deadOrAliveCount() && *selectedDeviceIndex == 0xFF; deviceNumber++)
			if (board[boardNumber]->alive(deviceNumber)) {
				if (currentCount == 0) {
					if (++lastBoardAndIndex == selectedNumber) {
						// Board *selectedBoard = board[boardNumber];
						*selectedDeviceIndex = deviceNumber;
						*selectedBoardIndex = boardNumber;
						*maxInput = board[boardNumber]->deadOrAliveCount() / board[boardNumber]->devicesOnASingleBoard() - 1;
						break;
					}
				}

				if (++currentCount == board[boardNumber]->devicesOnASingleBoard())
					currentCount = 0;
			}
	}
	return *selectedDeviceIndex != 0xFF;
}

/** Test Bluetooth
*/
void Robot::bluetoothTest() {
	static uint32_t startMs = millis();
	if (millis() - startMs > 100) {
		print("Time: %i ms.\r\n", millis() - startMs);
		startMs = millis();
	}
}

/** Display all the incomming and outcomming CAN Bus messages
*/
void Robot::canBusSniffToggle() {
	_sniff = !_sniff;
	print(_sniff ? "Sniff on\n\r" : "Sniff off\n\r");
	end();
}

/** Detects if there is a gap in CAN Bus addresses' sequence of any device, like 0, 2, 3 (missing 1).
@return - is there a gap.
*/
bool Robot::canGap() {
	for (uint8_t i = 0; i < _boardNextFree; i++)
		if (board[i]->canGap())
			return true;
	return false;
}

/** Change device's id
*/
void Robot::canIdChange() {
	uint8_t selectedBoardIndex;
	uint8_t selectedDeviceIndex;
	uint8_t maxInput;
	uint8_t lastBoardAndDeviceIndex;
	if (boardDisplayAndSelect(&selectedBoardIndex, &selectedDeviceIndex, &maxInput, &lastBoardAndDeviceIndex)) {
		// Enter new id
		print(". %s\n\rEnter new board id [0..%i]: ", board[selectedBoardIndex]->name(), maxInput);
		uint8_t newDeviceNumber = serialReadNumber(15000, 500, maxInput <= 9, maxInput);

		if (newDeviceNumber != 0xFF) {
			// Change
			print("%i\n\rChange requested.\n\r", newDeviceNumber);
				board[selectedBoardIndex]->idChange(newDeviceNumber, selectedDeviceIndex);
			delayMs(500); // Delay for firmware handling of devices with the same ids.
		}
	}

	end();
}

void Robot::canScanToggle(){
	_devicesScanBeforeMenuAndSwitches = !_devicesScanBeforeMenuAndSwitches;
	print(_devicesScanBeforeMenuAndSwitches ? "CAN scan on\n\r" : "CAN scan off\n\r");
	end();
}

/** mrm-color-can illumination off
*/
void Robot::colorIlluminationOff() {
	if (mrm_col_can->alive())
		mrm_col_can->illumination(0xFF, 0);
	else if (mrm_col_b->alive())
		mrm_col_b->illumination(0xFF, 0);
	end();
}

/** mrm-color-can illumination on
*/
void Robot::colorIlluminationOn() {
	if (mrm_col_can->alive())
		mrm_col_can->illumination(0xFF, 5);
	else if (mrm_col_b->alive())
		mrm_col_b->illumination(0xFF, 5);
	end();
}

/** Erase HSV patterns
*/
void Robot::colorPatternErase() {
	if (mrm_col_can->alive())
		mrm_col_can->patternErase();
	else if (mrm_col_b->alive())
		mrm_col_b->patternErase();
	end();
}

/** Print HSV patterns
*/
void Robot::colorPatternPrint() {
	if (mrm_col_can->alive())
		mrm_col_can->patternPrint();
	else if (mrm_col_b->alive())
		mrm_col_b->patternPrint();
	end();
}

/** Record HSV color patterns
*/
void Robot::colorPatternRecord() {
	if (mrm_col_can->alive())
		mrm_col_can->patternsRecord();
	else if (mrm_col_b->alive())
		mrm_col_b->patternsRecord();
	end();
}

/** Recognize HSV color pattern
*/
void Robot::colorPatternRecognize() {
	end();
}

void Robot::colorTest10(){
		// mrm_col_b->test(false);
	mrm_col_b->test(false);
}

void Robot::colorTest6(){
	mrm_col_can->test(false);
}

void Robot::colorTest6HSV(){
	mrm_col_can->test(true);
}

void Robot::colorTestHSV(){
	// mrm_col_b->test(true);
	mrm_col_b->test(true);
}

/** The right way to use Arduino function delay
@param pauseMs - pause in ms. One run even if pauseMs == 0, so that delayMs(0) receives all messages.
*/
void Robot::delayMs(uint16_t pauseMs) {
	uint32_t startMs = millis();
	do  {
		noLoopWithoutThis();
	} while (millis() - startMs < pauseMs);
}

/** The right way to use Arduino function delayMicros
@param pauseMicros - pause in micros. One run even if pauseMicros == 0, so that delayMicross(0) receives all messages.
*/
void Robot::delayMicros(uint16_t pauseMicros) {
	uint32_t startMicros = micros();
	do {
		noLoopWithoutThis();
		if (startMicros > micros()) // micros() will overflow after 72 min!
			startMicros = 0; // In that case, reset the start time
	} while (micros() < startMicros + pauseMicros);
}

/** Lists all the alive (responded to last ping) CAN Bus devices.
@boardType - sensor, motor, or all boards
@return count
*/
void Robot::deviceInfo(uint8_t deviceGlobalOrdinalNumber, BoardInfo * deviceInfo, Board::BoardType boardType){
	uint8_t count = 0;
	for (uint8_t boardKind = 0; boardKind < _boardNextFree; boardKind++){
		if (boardType == Board::ANY_BOARD || board[boardKind]->boardType() == boardType){ // Board types
			for (uint8_t deviceNumber = 0; deviceNumber < board[boardKind]->count(); deviceNumber++){// Devices for the current board type
				if (board[boardKind]->alive(deviceNumber)){
					if (count == deviceGlobalOrdinalNumber)
					{
						strcpy(deviceInfo->name, board[boardKind]->name(deviceNumber));
						deviceInfo->board = board[boardKind];
						deviceInfo->deviceNumber = deviceNumber;
						//print("In func: %s %i", deviceInfo->name, deviceNumber);
						if (boardType == Board::SENSOR_BOARD)
							deviceInfo->readingsCount = ((SensorBoard*)(board[boardKind]))->readingsCount();
						return;
					}
					else
						count++;
				}
			}
		}
	}
	strcpy(deviceInfo->name, "");
	deviceInfo->readingsCount = 0;
}

/** Display number of CAN Bus devices using 8x8 display
*/
void Robot::devicesLEDCount(){
	if (mrm_8x8a->alive()){
		char buffer[7];
		sprintf(buffer, "N:%i.", _devicesAtStartup);
		mrm_8x8a->text(buffer);
	}
}

/** Scan single device
*/
void Robot::deviceScan() {
	//Board type
	for (uint8_t i = 0; i < _boardNextFree; i++)
		print("%i. %s\n\r", i, board[i]->name());

	print("Enter board id.\n\r");
	uint8_t selectedBoardIndex = serialReadNumber(60000, 500, _boardNextFree <= 10, _boardNextFree - 1);

	print("%i.\n\rEnter device id.\n\r", selectedBoardIndex);
	uint8_t selectedDeviceIndex = serialReadNumber(60000, 500, false, 100);
	print("%i\n\r", selectedDeviceIndex);

	print("Scan %s\n\r", board[selectedBoardIndex]->name(selectedDeviceIndex));
	board[selectedBoardIndex]->_aliveReport = true; // So that Board::messageDecodeCommon() prints board's name
	uint8_t canData[8];
	canData[0] = COMMAND_REPORT_ALIVE;
	board[selectedBoardIndex]->messageSend(canData, 1, selectedDeviceIndex);

	end();
}

/** Contacts all the CAN Bus devices and checks which one is alive.
@verbose - if true, print.
@boardType - sensor, motor, or all boards
@return count
*/
uint8_t Robot::devicesScan(bool verbose, Board::BoardType boardType) {
	while (millis() < 3000) //Wait for all the devices to complete start-up
		delayMs(50);

	devicesStop();

	delayMs(5); // Read all the messages sent after stop.

	// Set not alive
	for (uint8_t i = 0; i < _boardNextFree; i++)
		if (boardType == Board::ANY_BOARD || board[i]->boardType() == boardType)
			board[i]->aliveSet(false); // Mark as not alive. It will be marked as alive when returned message arrives.

	// Send alive ping
	for (uint8_t k = 0; k < 2; k++)
		for (uint8_t i = 0; i < _boardNextFree; i++){
			if (boardType == Board::ANY_BOARD || board[i]->boardType() == boardType)
				board[i]->devicesScan(verbose);
				// print("SC1 %s ", board[i]->name()),count += board[i]->devicesScan(verbose), print("SC2");
		}

	// In the meantime, Board::messageDecodeCommon and derived count the alives

	// Count alive
	uint8_t count = 0;
	for (uint8_t i = 0; i < _boardNextFree; i++)
		if (boardType == Board::ANY_BOARD || board[i]->boardType() == boardType)
			count += board[i]->aliveCount(); 

	if (verbose)
		print("%i devices.\n\r", count);
	if (canGap())
		strcpy(errorMessage, "CAN gap");
	end();
	return count;
}

void Robot::devicesScan(){
	devicesScan(true);
}

/** Starts devices' CAN Bus messages broadcasting.
*/
void Robot::devicesStart(uint8_t measuringMode) {
	for (uint8_t deviceNumber = 0; deviceNumber < _boardNextFree; deviceNumber++)
		board[deviceNumber]->start(0xFF, measuringMode);
}

/** Stops broadcasting of CAN Bus messages
*/
void Robot::devicesStop() {
	for (uint8_t deviceNumber = 0; deviceNumber < _boardNextFree; deviceNumber++) {
		board[deviceNumber]->stop();
		delayMs(1); // TODO
	}
}

/** Displays errors and stops motors, if any.
*/
void Robot::errors() {
	if (strcmp(errorMessage, "") != 0) {
		print("Error! %s\n\r", errorMessage);
		strcpy(errorMessage, "");
		stopAll(); // Stop all motors
		end();
	}
}

/** Displays each CAN Bus device's firmware
*/
void Robot::firmwarePrint() {
	for (uint8_t i = 0; i < _boardNextFree; i++) {
		board[i]->firmwareRequest();
		delayMs(1);
	}
	end();
}

/** Returns FPS (frames per second).
@return - FPS
*/
float Robot::fpsGet() {
	float fpsNow;
	//print("Next: %i %i %i\n\r", fpsNextIndex, fpsMs[0], fpsMs[1]);
	if (fpsMs[1] == 0 || fpsMs[0] == 0)
		fpsNow = 0;
	else if (fpsMs[0] == fpsMs[1])
		fpsNow = 1000000;
	else
		fpsNow = 1000000.0 / (float)(fpsMs[1] > fpsMs[0] ? fpsMs[1] - fpsMs[0] : fpsMs[0] - fpsMs[1]);
	return fpsNow;
}

/** Avoids FPS measuring in the next 2 cycles.
*/
void Robot::fpsPause() {
	fpsMs[0] = 0;
	fpsMs[1] = 0;
}

/** Prints FPS all CAN Bus devices and mrm-eps32 boards. Also prints CAN Bus frequency.
*/
void Robot::fpsPrint() {
	print("CAN peaks: %i received/s, %i sent/s\n\r", mrm_can_bus->messagesPeakReceived(), mrm_can_bus->messagesPeakSent());
	print("Arduino: %i FPS, low peak: %i FPS\n\r", (int)fpsGet(), fpsTopGap == 1000 ? 0 : (int)(1000 / (float)fpsTopGap));
	for (uint8_t i = 0; i < _boardNextFree; i++) {
		board[i]->fpsRequest();
		uint32_t startMs = millis();
		while (millis() - startMs < 30)
			noLoopWithoutThis();
		board[i]->fpsDisplay();
	}
	fpsReset();
	end();
}

/** Resets FPS data
*/
void Robot::fpsReset() {
	fpsTopGap = 0;
	fpsMs[0] = 0;
	fpsMs[1] = 0;
	mrm_can_bus->messagesReset();
}

/** Updates data for FPS calculation
*/
void Robot::fpsUpdate() {
	fpsMs[fpsNextIndex] = millis(); // millis() will overflow after some time! Not taken into account here.
	fpsNextIndex = (fpsNextIndex == 0 ? 1 : 0);
	if (fpsMs[0] != 0 && fpsMs[1] != 0) {
		uint32_t gap = (fpsNextIndex == 0 ? fpsMs[1] - fpsMs[0] : fpsMs[0] - fpsMs[1]);
		if (gap > fpsTopGap)
			fpsTopGap = gap;
	}
}


/**Compass
@return - North is 0 degrees, clockwise are positive angles, values 0 - 360.
*/
float Robot::heading() {
	return mrm_imu->heading();
}


/** Lists I2C devices
*/
void Robot::i2cTest() {
	print("Scanning.\n\r");

	bool any = false;
	for (byte address = 1; address < 127; address++)
	{
		Wire.beginTransmission(address); // Transmission tried
		byte status = Wire.endTransmission(); // Was it successful?
		if (status == 0)
		{
			print("Found at address 0x%02x\n\r", address);
			any = true;
		}
		else if (status == 4)
			print("Found at address 0x%02x\n\r", address);
	}
	if (!any)
		print("Nothing found.\n\n\r");

	end();
}

void Robot::imuTest(){
	mrm_imu->test();
}

/** Request information
*/
void Robot::info() {
	for (uint8_t i = 0; i < _boardNextFree; i++) {
		board[i]->info();
		delay(1);
	}
	end();
}

void Robot::irFinderTest(){
	mrm_ir_finder3->test();
}

void Robot::irFinderTestCalculated(){
	mrm_ir_finder3->testCalculated();
}

void Robot::led8x8Test(){
	mrm_8x8a->test(); 
}

/** Tests mrm-lid-can-b
*/
void Robot::lidar2mTest() {
	static uint16_t selected;
	if (setup()) {
		//devicesScan(false, SENSOR_BOARD);
		// Select lidar
		uint8_t count = mrm_lid_can_b->deadOrAliveCount();
		print("%s - enter lidar number [0-%i] or wait for all\n\r", mrm_lid_can_b->name(), count - 1);
		selected = serialReadNumber(2000, 1000, count - 1 < 9, count - 1, false);
		if (selected == 0xFFFF) { // Test all
			print("Test all\n\r");
			selected = 0xFF;
		}
		else { // Test only selected
			if (mrm_lid_can_b->alive(selected))
				print("\n\rTest lidar %s\n\r", mrm_lid_can_b->name(selected));
			else {
				print("\n\rLidar %s dead, test all\n\r", mrm_lid_can_b->name(selected));
				selected = 0xFF;
			}
		}
		//mrm_lid_can_b->start(selected);
	}
	mrm_lid_can_b->test(selected);
}

/** Tests mrm-lid-can-d
*/
void Robot::lidar4mMultiTest() {
	mrm_lid_d->test();
}

/** Tests mrm-lid-can-b2
*/
void Robot::lidar4mTest() {
	if (setup())
		mrm_lid_can_b2->start();
	mrm_lid_can_b2->test();
}

/** Calibrates lidars
*/
void Robot::lidarCalibrate() {
	print("Lidar calibration\n\r");

	// Select lidar 2 or 4 m
	int8_t selected2Or4 = -1;
	uint32_t lastMs;
	while (selected2Or4 != 2 && selected2Or4 != 4) {
		print("Enter max distance [2 or 4]m or wait to abort ");
		lastMs = millis();
		selected2Or4 = -1;
		while (millis() - lastMs < 10000 && selected2Or4 == -1)
			if (Serial.available()) {
				uint8_t ch = Serial.read() - 48;
				print("%i\n\r", ch);
				selected2Or4 = ch;
			}
		if (selected2Or4 == -1) {
			print("- abort\n\r");
			break;
		}
	}

	// Select lidar number
	if (selected2Or4 != -1) {
		// Select lidar
		uint8_t count = selected2Or4 == 2 ? mrm_lid_can_b->deadOrAliveCount() : mrm_lid_can_b2->deadOrAliveCount();
		print("Enter lidar number [0-%i] or wait to abort", count - 1);
		uint16_t selected = serialReadNumber(3000, 1000, count - 1 < 9, count - 1, false);
		if (selected == 0xFFFF)
			print("\n\rAbort\n\r");
		else {
			if (selected2Or4 == 2 ? mrm_lid_can_b->alive(selected) : mrm_lid_can_b2->alive(selected)) {
				print("\n\rCalibrate lidar %s\n\r", mrm_lid_can_b->name(selected));
				selected2Or4 == 2 ? mrm_lid_can_b->calibration(selected) : mrm_lid_can_b2->calibration(selected);
			}
			else
				print("\n\rLidar %s dead\n\r", selected2Or4 == 2 ? mrm_lid_can_b->name(selected) : mrm_lid_can_b2->name(selected));
		}
	}

	end();
}

/** Displays menu
*/
void Robot::menu() {
	// Print menu
	if (_devicesScanBeforeMenuAndSwitches){
		uint8_t cnt = devicesScan(false);
		if (cnt > _devicesAtStartup)  // Late-booters
			_devicesAtStartup = cnt;
		else if (cnt < _devicesAtStartup){
			print("%i devices instead of %i!\n\r", cnt, _devicesAtStartup);
			if (mrm_8x8a->alive(0, false))
				mrm_8x8a->text((char*)"Error. Cnt.");
		}
	}
	print("\r\n");

	bool any = false;
	uint8_t column = 1;
	uint8_t maxColumns = 2;
	for (uint8_t i = 0; i < _actionNextFree; i++) {
		if ((_action[i]->_menuLevel | menuLevel) == _action[i]->_menuLevel) {
			bool anyAlive = false;
			if (_action[i]->boardsId() == Board::ID_ANY)
				anyAlive = true;
			else
				for (uint8_t j = 0; j < _boardNextFree && !anyAlive; j++)
					if (board[j]->alive(0xFF) && board[j]->id() == _action[i]->boardsId())
						anyAlive = true;
			if (anyAlive) {
				print("%-3s - %-15s%s", _action[i]->_shortcut, _action[i]->_text, column == maxColumns ? "\n\r" : ""); 
				delayMs(2);
				any = true;
				if (column++ == maxColumns)
					column = 1;
			}
		}
	}

	if (!any)
		print("Menu level %i empty.\r\n", menuLevel);
	else
		if (column != 1)
			print("\r\n");

	// Display errors
	for (uint8_t deviceNumber = 0; deviceNumber < _boardNextFree; deviceNumber++)
		if (board[deviceNumber]->errorCodeLast() != 0)
			print("Error %i in %s\n\r", board[deviceNumber]->errorCodeLast(), board[deviceNumber]->name(board[deviceNumber]->errorWasInDeviceNumber()));

	fpsPause(); // this function took too much time

	_actionCurrent = _actionDoNothing;
}

/** Color menu
*/
void Robot::menuColor() {
	menuLevel = 4;
	end();
}

/** Generic menu
*/
void Robot::menuLoop() {
	menuLevel = 8;
	end();
}

/** Displays menu and stops motors
*/
void Robot::menuMainAndIdle() {
	stopAll();
	menuLevel = 1;
}

/** Reflectance menu
*/
void Robot::menuReflectance() {
	menuLevel = 2;
	end();
}

/** System menu
*/
void Robot::menuSystem() {
	menuLevel = 16;
	end();
}

/** Print CAN Bus message
@param msg - message
@param oubound - if not, inbound
*/
void Robot::messagePrint(CANBusMessage *msg, Board* board, uint8_t boardIndex, bool outbound) {
	if (msg->dlc > 8){
		print("dlc too big: %i\n\r", (int)msg->dlc);
		exit(12);
	}
	print("%.3lfs %s id:%s (0x%02X)", millis() / 1000.0, outbound ? "Out" : "In", 
		board == NULL ? "Unknown" : board->name(boardIndex), msg->messageId);

	for (uint8_t i = 0; i < msg->dlc; i++) {
		if (i == 0){
			print(" command: ");
			if ((msg->data[0] > 0x0F && msg->data[0] < 0x50) || msg->data[0] == 0xFF)
				board->commandNamePrint(msg->data[0]);
			else{	
				if (board != NULL && board->commandName(msg->data[0]) != "")
					print((board->commandName(msg->data[0])).c_str());
				else
					print("Unknown");
			}
			print(" (%02X)", _msg->data[0]);
		}
		else
			print(" %02X", _msg->data[i]);
	}
	print("\n\r");
}


/** Receives CAN Bus messages. 
*/
void Robot::messagesReceive() {
	#define REPORT_DEVICE_TO_DEVICE_MESSAGES_AS_UNKNOWN false
	while (true) {
		_msg = mrm_can_bus->messageReceive();
		if (_msg == NULL) // No more messages
			break;
		uint32_t id = _msg->messageId;
		if (_sniff){
			Board* boardFound;
			int index;
			if (boardIdentify(_msg->messageId, true , &boardFound, index))
				messagePrint(_msg, boardFound, index, true);
			else
				messagePrint(_msg, NULL, 0, true);
		}
		#if REPORT_DEVICE_TO_DEVICE_MESSAGES_AS_UNKNOWN
		bool any = false;
		#endif
		for (uint8_t boardId = 0; boardId < _boardNextFree; boardId++) {
			// if (!strcmp(board[boardId]->name(), "US-B")){ 
			// 	print ("BRD: %s\n\r", board[boardId]->name());
			// 	messagePrint(_msg, false);
			// }
 			if (board[boardId]->messageDecode(id, _msg->data, _msg->dlc)) {
				#if REPORT_DEVICE_TO_DEVICE_MESSAGES_AS_UNKNOWN
				any = true;
				break;
				#endif
			}
		}

// #if REPORT_DEVICE_TO_DEVICE_MESSAGES_AS_UNKNOWN
// 		if (!any)
// 			print("Address device unknown: 0x%X\n\r", id);
// #endif
	}
}

/** Tests motors
*/
void Robot::motorTest() {
	print("Test motors\n\r");
	for (uint8_t i = 0; i < _boardNextFree; i++) 
		if (board[i]->boardType() == Board::MOTOR_BOARD && board[i]->count() > 0)
			board[i]->test();
	end();
}

void Robot::nodeServoTest() {
	mrm_node->servoTest();
}

/** Tests mrm-node
*/
void Robot::nodeTest() {
	if (setup())
		mrm_node->start();
	mrm_node->test();
}

/** Any for or while loop must include call to this function.
*/
void Robot::noLoopWithoutThis() {
	blink(); // Keep-alive LED. Solder jumper must be shorted in order to work in mrm-esp32.
	messagesReceive();
	fpsUpdate(); // Measure FPS. Less than 30 - a bad thing.
	verbosePrint(); // Print FPS and maybe some additional data
	errors();
#if RADIO == 2
	web();
#endif
}

/** Production test
*/
void Robot::oscillatorTest() {
	if (setup()) {
		uint8_t selectedBoardIndex;
		uint8_t selectedDeviceIndex;
		uint8_t maxInput;
		uint8_t lastBoardAndDeviceIndex;
		if (boardDisplayAndSelect(&selectedBoardIndex, &selectedDeviceIndex, &maxInput, &lastBoardAndDeviceIndex)) {
			print("\n\r");
			board[selectedBoardIndex]->oscillatorTest(selectedDeviceIndex);
		}
	}
}


/**Pitch
@return - Pitch in degrees. Inclination forwards or backwards. Leveled robot shows 0 degrees.
*/
float Robot::pitch() {
	return mrm_imu->pitch();
}


/** Enable plug and play for all the connected boards.
 */
void Robot::pnpOn(){
	pnpSet(true);
}

/** Disable plug and play for all the connected boards.
 */
void Robot::pnpOff(){
	pnpSet(false);
}

/** Enable or disable plug and play for all the connected boards.
 @param enable - enable or disable
 */
void Robot::pnpSet(bool enable){
	uint8_t count = mrm_lid_can_b2->deadOrAliveCount();
	for (uint8_t i = 0; i < count; i++)
		if (mrm_lid_can_b2->alive(i)){
			mrm_lid_can_b2->pnpSet(enable, i);
			print("%s PnP %s\n\r", mrm_lid_can_b2->name(i), enable ? "on" : "off");
		}
	count = mrm_lid_can_b->deadOrAliveCount();
	for (uint8_t i = 0; i < count; i++)
		if (mrm_lid_can_b->alive(i)){
			mrm_lid_can_b->pnpSet(enable, i);
			print("%s PnP %s\n\r", mrm_lid_can_b->name(i), enable ? "on" : "off");
		}
	count = mrm_ref_can->deadOrAliveCount();
	for (uint8_t i = 0; i < count; i++)
		if (mrm_ref_can->alive(i)){
			mrm_ref_can->pnpSet(enable, i);
			print("%s PnP %s\n\r", mrm_ref_can->name(i), enable ? "on" : "off");
		}
	count = mrm_col_can->deadOrAliveCount();
	for (uint8_t i = 0; i < count; i++)
		if (mrm_col_can->alive(i)){
			mrm_col_can->pnpSet(enable, i);
			print("%s PnP %s\n\r", mrm_col_can->name(i), enable ? "on" : "off");
		}
	end();
}

void Robot::reflectanceArrayCalibrate(){
	mrm_ref_can->calibrate();
}

/** Prints mrm-ref-can* calibration data
*/
void Robot::reflectanceArrayCalibrationPrint() {
	mrm_ref_can->calibrationDataRequest(0xFF, true);
	mrm_ref_can->calibrationPrint();
	end();
}

void Robot::reflectanceArrayTestAnalog(){
	mrm_ref_can->test(true);
}

void Robot::reflectanceArrayTestDigital(){
	mrm_ref_can->test(false);
}

/** One pass of robot's program
*/
void Robot::refresh(){
		actionSet(); // Check if a key pressed and update current command buffer.
		if (_actionCurrent == NULL) // If last command finished, display menu.
			menu();
		else 
			actionProcess(); // Process current command. The command will be executed while currentCommand is not NULL. Here state maching processing occurs, too.
		noLoopWithoutThis(); // Receive all CAN Bus messages. This call should be included in any loop, like here.
}


/** Roll
@return - Roll in degrees. Inclination to the left or right. Values -90 - 90. Leveled robot shows 0�.
*/
float Robot::roll() {
	return mrm_imu->roll();
}


/** Starts robot's program
*/
void Robot::run() {
	while (true) 
		refresh();
}


/** Reads serial ASCII input and converts it into an integer
@param timeoutFirst - timeout for first input
@param timeoutBetween - timeout between inputs
@param onlySingleDigitInput - completes input after first digit
@param limit - returns 0xFFFF if overstepped
@param printWarnings - prints out of range or timeout warnings
@return - converted number or 0xFFFF when timeout
*/
uint16_t Robot::serialReadNumber(uint16_t timeoutFirst, uint16_t timeoutBetween, bool onlySingleDigitInput, uint16_t limit, bool printWarnings) {

	// Read input
	uint32_t lastMs = millis();
	uint32_t convertedNumber = 0;
	bool any = false;
	while ((millis() - lastMs < timeoutFirst && !any) || (!onlySingleDigitInput && millis() - lastMs < timeoutBetween && any)) {
		bool btAvailable = false;
#if RADIO == 1
		btAvailable = serialBT != NULL && serialBT->available();
#endif
		if (Serial.available() || btAvailable) {
			uint8_t character = 0;
			if (Serial.available()){
				character = Serial.read();
				if (character == 13) // Enter
					break;
			}
#if RADIO == 1
			else if (serialBT != NULL && serialBT->available())
				character = serialBT->read();
#endif
			if (48 <= character && character <= 57) {
				convertedNumber = convertedNumber * 10 + (character - 48);
				any = true;
				lastMs = millis();
			}
		}
		noLoopWithoutThis();
	}

	// Eat tail
	while (Serial.available())
		Serial.read();
#if RADIO == 1
	while (serialBT != NULL && serialBT->available())
		serialBT->read();
#endif

	// Return result
	if (any) {
		if (convertedNumber > limit) {
			if (printWarnings)
				print("Out of range\n\r");
			return 0xFFFF;
		}
		else
			return convertedNumber;
	}
	else {
		if (printWarnings)
			print("Timeout.\n\r");
		return 0xFFFF;
	}
}

/** Moves servo motor manually
*/
void Robot::servoInteractive() {
	mrm_servo->writeInteractive();
	end();
}

void Robot::servoTest(){
	mrm_servo->test();
}

/** Stops all motors
*/
void Robot::stopAll() {
	devicesStop();
	for (uint8_t i = 0; i < _boardNextFree; i++)
		if (board[i]->boardType() == Board::MOTOR_BOARD && board[i]->count() > 0)
			((MotorBoard*)board[i])->stop();
	end();
}

/** CAN Bus stress test
*/
void Robot::stressTest() {
	const bool STOP_ON_ERROR = true;
	const uint32_t LOOP_COUNT = 1000000;
	const bool TRY_ONLY_ALIVE = true;

	// Setup
	static uint32_t pass;
	static uint8_t lastPercent = 101;
	static uint8_t count[BOARDS_LIMIT];
	static uint32_t errors[BOARDS_LIMIT];
	static uint16_t mask[BOARDS_LIMIT]; // 16 bits - no more than 16 devices per board!

	if (setup()) {
		print("Before test.\n\r");
		pass = 0;
		devicesStop();
		for (uint8_t i = 0; i < _boardNextFree; i++)
			errors[i] = 0;
		uint8_t totalCnt = 0;
		for (uint8_t i = 0; i < _boardNextFree; i++) {
			board[i]->devicesScan(true);
			count[i] = board[i]->aliveCount();
			totalCnt += count[i];
			mask[i] = TRY_ONLY_ALIVE ? 0 : 0xFFFF;
			for (uint8_t j = 0; j < board[i]->deadOrAliveCount(); j++)
				if (board[i]->alive(j))
					mask[i] |= 1 << j;
		}
		print("Start.\n\r");
		if (mrm_8x8a->alive()) {
			char buffer[50];
			sprintf(buffer, "%i devices.\n\r", totalCnt);
			mrm_8x8a->text(buffer);
			mrm_8x8a->activeCheckIfStartedSet(false); // This is not a normal state, but just not to disrupt the test.
		}
	}

	// Display progress numerically
	uint8_t percent = 100 * pass / LOOP_COUNT;
	if (percent != lastPercent && percent > 0) {
		lastPercent = percent;
		print("%i %%\n\r", percent);
	}

	// Display progress using mrm-8x8a
	if (mrm_8x8a->alive())
		if (mrm_8x8a->progressBar(LOOP_COUNT, pass, pass == 0))
			delayMs(5); // To avoid disturbing stress test

	// Stress test
	for (uint8_t i = 0; i < _boardNextFree; i++) {
		if (count[i] > 0 || !TRY_ONLY_ALIVE) {
			delayMicros(40);
			digitalWrite(15, HIGH);
			board[i]->devicesScan(false, mask[i]);
			uint8_t cnt = board[i]->aliveCount();
			digitalWrite(15, LOW);
			if (cnt != count[i]) {
				errors[i]++;
				print("***** %s: found %i, not %i.\n\r", board[i]->name(), cnt, count[i]);
				if (STOP_ON_ERROR) {
					if (mrm_8x8a->alive()) {
						char buffer[50];
						sprintf(buffer, "%s: error.\n\r", board[i]->name());
						mrm_8x8a->text(buffer);
					}
					pass = LOOP_COUNT - 1;
					break;
				}
			}
		}
	}

	// Results
	if (++pass >= LOOP_COUNT || userBreak()) {
		bool allOK = true;
		for (uint8_t i = 0; i < _boardNextFree; i++)
			if (count[i] > 0 && errors[i] > 0) {
				print("%s: %i errors.\n\r", board[i]->name(), errors[i]);
				allOK = false;
				delay(5000); // To freeze oscilloscope
			}
		if (allOK) {
			if (mrm_8x8a->alive())
				mrm_8x8a->bitmapDisplay(0);
			print("No errors.");
		}
		print("\n\r");
		end();
		// return true;
	}
	// else
	// 	return false;
}

/** Tests mrm-therm-b-can
*/
void Robot::thermoTest() {
	if (setup())
		mrm_therm_b_can->start();
	mrm_therm_b_can->test();
}

void Robot::us1Test(){
	mrm_us1->test();
}

void Robot::usBTest(){
	mrm_us_b->test();;
}

/** Checks if user tries to break the program
@return - true if break requested.
*/
bool Robot::userBreak() {
	bool btAvailable = false;
#if RADIO == 1
	btAvailable = serialBT != NULL && serialBT->available();
#endif
	if (/*switchOn() ||*/ Serial.available() || btAvailable) {
		return true;
	}
	else
		return false;
}

/** Prints additional data in every loop pass
*/
void Robot::verbosePrint() {
	if (verbose) {
		static uint32_t lastMs = 0;
		if (lastMs == 0 || millis() - lastMs > 5000) {
			print("%i fps\r\n", (uint16_t)fpsGet());
			lastMs = millis();
		}
	}
}

/** Verbose output toggle
*/
void Robot::verboseToggle() {
	verbose = !verbose;
};

#if RADIO == 2
/** Web server
*/
void Robot::web(){
	static uint32_t previousTime = 0;
	static uint32_t currentTime = 0;
	const uint16_t timeoutTime = 2000;
	// Variable to store the HTTP request
	String header;

	WiFiClient client = webServer->available();   // Listen for incoming clients
   


  	if (client) {                             // If a new client connects,
		currentTime = millis();
		previousTime = currentTime;
		print("New Client.\n\r");          // print a message out in the serial port
		String currentLine = "";                // make a String to hold incoming data from the client
		while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
			currentTime = millis();
			if (client.available()) {             // if there's bytes to read from the client,
				char c = client.read();             // read a byte, then
				print("%c", c);                    // print it out the serial monitor
				header += c;
				if (c == '\n') {                    // if the byte is a newline character
					// if the current line is blank, you got two newline characters in a row.
					// that's the end of the client HTTP request, so send a response:
					if (currentLine.length() == 0) {
						// HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
						// and a content-type so the client knows what's coming, then a blank line:
						client.println("HTTP/1.1 200 OK");
						client.println("Content-type:text/html");
						client.println("Connection: close");
						client.println();
						
						// Display the HTML web page
						client.println("<!DOCTYPE html><html>");
						client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
						client.println("<link rel=\"icon\" href=\"data:,\">");
						// CSS to style the on/off buttons 
						// Feel free to change the background-color and font-size attributes to fit your preferences
						client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
						client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
						client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
						client.println(".button2 {background-color: #555555;}</style></head>");
						
						// Web Page Heading
						client.println("<body><h1>ESP32 Web Server</h1>");
						
						// Display current state, and ON/OFF buttons for GPIO 26  
						client.println("<p>State </p>");

						client.println("</body></html>");
						
						// The HTTP response ends with another blank line
						client.println();
						// Break out of the while loop
						break;
					} else { // if you got a newline, then clear currentLine
						currentLine = "";
					}
				} else if (c != '\r') {  // if you got anything else but a carriage return character,
					currentLine += c;      // add it to the end of the currentLine
				}
			}
		}
		// Clear the header variable
		header = "";
		// Close the connection
		client.stop();
		Serial.println("Client disconnected.");
		Serial.println("");
	}
}
#endif