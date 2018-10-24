// FeatherFly Transmitter - eSk8 Remote
//
// basic code by SolidGeek | https://github.com/SolidGeek/nRF24-Esk8-Remote
// modified to run on an Feather M0 with RFM69 | https://github.com/StefanMeGit/FeatherM0-Esk8-Remote
//
// ATTENTION!!! This is not for daily use!! Still under hard development!

#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <FlashStorage.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

//#define DEBUG

#define VERSION 1.1

// Defining the type of display used (128x32)
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const unsigned char logo[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x80, 0x3c, 0x01,
  0xe0, 0x00, 0x07, 0x70, 0x18, 0x0e, 0x30, 0x18, 0x0c, 0x98, 0x99, 0x19,
  0x80, 0xff, 0x01, 0x04, 0xc3, 0x20, 0x0c, 0x99, 0x30, 0xec, 0xa5, 0x37,
  0xec, 0xa5, 0x37, 0x0c, 0x99, 0x30, 0x04, 0xc3, 0x20, 0x80, 0xff, 0x01,
  0x98, 0x99, 0x19, 0x30, 0x18, 0x0c, 0x70, 0x18, 0x0e, 0xe0, 0x00, 0x07,
  0x80, 0x3c, 0x01, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Transmit and receive package
struct debug { 
  unsigned long cycleTime;
  unsigned long transmissionTime;
  uint8_t rssi;
  unsigned long counterJoined;
  unsigned long counterSend;
  unsigned long counterReceived;
  unsigned long differenceJoinedSend;
  unsigned long differenceJoinedReceived;
  unsigned long longestCycleTime;
} debugData;


// Defining struct to hold setting values while remote is turned on.
typedef struct {
  uint8_t triggerMode;              // 0
  uint8_t batteryType;              // 1
  uint8_t batteryCells;             // 2
  uint8_t motorPoles;               // 3
  uint8_t motorPulley;              // 4
  uint8_t wheelPulley;              // 5
  uint8_t wheelDiameter;            // 6
  uint8_t controlMode;              // 7
  short minHallValue;               // 8
  short centerHallValue;            // 9
  short maxHallValue;               // 10
  uint8_t boardID;                  // 11
  uint8_t pairNewBoard;             // 12
  uint8_t transmissionPower;        // 13
  uint8_t customEncryptionKey[16];  // 14
  float firmVersion;                // 15
} TxSettings;

TxSettings txSettings;

// Defining flash storage
FlashStorage(flash_TxSettings, TxSettings);

uint8_t currentSetting = 0;
const uint8_t numOfSettings = 19;

// Setting rules format: default, min, max.
const short rules[numOfSettings][3] {
  {0, 0, 1},          //0 0: Killswitch  | 1: Cruise control
  {1, 0, 1},          //1 0: Li-ion      | 1: LiPo
  {10, 6, 12},        //2 Cell count
  {14, 0, 250},       //3 Motor poles
  {14, 0, 250},       //4 Motor pully
  {38, 0, 250},       //5 Wheel pulley
  {80, 0, 250},       //6 Wheel diameter
  {1, 0, 2},          //7 0: PPM only   | 1: PPM and UART | 2: UART only
  {200, 0, 300},      //8 Min hall value
  {500, 300, 700},    //9 Center hall value
  {800, 700, 1023},   //10 Max hall value
  {1, 0, 9},          //11 boardID
  { -1, 0, 0},        //12 pair new board
  {18, 14, 20},       //13 transmission power
  { -1, 0, 0},        //14 show Key
  { -1, 0 , 0},       //15 Firmware
  { -1, 0, 0},        //16 Set default key
  { -1, 0, 0},        //17 Settings
  { -1, 0, 0}         //18 Exit

};

const char titles[numOfSettings][19] = {
  "Trigger use", "Battery type", "Battery cells", "Motor poles", "Motor pulley",
  "Wheel pulley", "Wheel diameter", "Control mode", "Throttle min", "Throttle center",
  "Throttle max", "Board ID", "Pair new Board", "Transmission Power", "Encyption key",
  "Firmware Version", "Set default key", "Settings", "Exit"
};

const uint8_t unitIdentifier[numOfSettings]  = {0, 0, 1, 0, 2, 2, 3, 0, 0, 0, 0, 4, 0, 5, 0, 0, 0, 0};
const uint8_t valueIdentifier[numOfSettings] = {1, 2, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const char stringValues[3][3][13] = {
  {"Killswitch", "Cruise", ""},
  {"Li-ion", "LiPo", ""},
  {"PPM", "PPM and UART", "UART only"},
};
const char settingUnits[5][4] = {"S", "T", "mm", "#", "dBm"};

const char dataSuffix[7][4] = {"V", "KMH", "KM", "A","ms","dBm", ""};
const char dataPrefix[3][9] = {"SPEED", "POWER", "DEBUG"};

// Defining constants to hold the special settings, so it's easy changed though the code
#define TRIGGER     0
#define MODE        7
#define BOARDID     11
#define PAIR        12
#define TxPower     13
#define KEY         14
#define FIRMWARE    15
#define DEFAULTKEY  16
#define SETTINGS    17
#define EXIT        18

// Defining struct to handle callback data (auto ack)
struct callback {
  float ampHours;
  float inpVoltage;
  long rpm;
  long tachometerAbs;
  uint8_t headlightActive;
  float avgInputCurrent;
  float avgMotorCurrent;
  float dutyCycleNow;
} returnData;

// defining button data
unsigned long buttonPrevMillis = 0;
const unsigned long buttonSampleIntervalsMs = 200;
byte longbuttonPressCountMax = 8;    // 80 * 25 = 2000 ms
byte mediumbuttonPressCountMin = 1;    // 20 * 25 = 500 ms
byte buttonPressCount = 0;
byte prevButtonState = HIGH;         // button is active low

// Transmit and receive package
struct package {    // | Normal   | Setting   | Dummy
  uint8_t type;   // | 0      | 1     | 2
  uint16_t throttle;  // | Throttle   |       |
  uint8_t trigger;  // | Trigger  |       |
  uint8_t headlight; //       | ON       | OFF         |
} remPackage;

// Define package to transmit settings
struct settingPackage {
  uint8_t setting;
  uint64_t value;
} setPackage;

// Defining struct to hold stats
struct stats {
  float minVoltage;
  float maxVoltage;
};

// Defining variables to hold values for speed and distance calculation
float gearRatio;
float ratioRpmSpeed;
float ratioPulseDistance;

// Pin defination
const uint8_t triggerPin = 5;
const uint8_t extraButtonPin = 6;
const uint8_t batteryMeasurePin = 9;
const uint8_t hallSensorPin = A3;
const uint8_t vibrationActuatorPin = 6;

// Definition for RFM69HW radio on Feather m0
#define RFM69_CS     8
#define RFM69_INT   3
#define RFM69_RST   4
#define DiagLED     13
#define RF69_FREQ   433.0
#define DEST_ADDRESS   1 // where the packages goes to
#define MY_ADDRESS     2 // own address

RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

uint8_t encryptionKey[16] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                              0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                            };

unsigned long  counterRecived = 0;
unsigned long  counterSent = 0;
unsigned long  counterJoined = 0;


// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";
uint8_t transmissionFailCounter = 0;

// define variable for battery alert
bool alarmTriggered = false;

// Battery monitering
const float minVoltage = 3.1;
const float maxVoltage = 4.2;
const float refVoltage = 3.3;

// Defining variables for Hall Effect throttle.
uint16_t hallValue, throttle;
const uint16_t centerThrottle = 512;
const uint8_t hallNoiseMargin = 10;
const uint8_t hallMenuMargin = 100;
uint8_t throttlePosition;

#define TOP 0
#define MIDDLE 1
#define BOTTOM 2

// Defining variables for OLED display
String tString;
uint8_t displayView = 3;
uint8_t x, y;

// Defining variables for alarm
unsigned long lastAlarmBlink;
bool alarmBlink = false;
bool batteryAlarmBlocked = false;
unsigned long lastBlockBlink;
bool blockBlink = false;

// Defining variables for vibration actuator
unsigned long lastVibrationBlink;
bool vibrationBlink = false;

// Defining variables for Settings menu
bool changeSettings     = false; // Global flag for whether or not one is editing the settings
bool changeThisSetting  = false;
bool settingsLoopFlag   = false;
bool triggerFlag = false;
bool settingScrollFlag  = false;
bool settingsChangeValueFlag = false;
unsigned short settingWaitDelay = 500;
unsigned short settingScrollWait = 800;
unsigned long settingChangeMillis = 0;

// TEST
unsigned short transmissionTimeStart = 0;
unsigned short transmissionTimeFinish = 0;
unsigned short transmissionTimeDuration = 0;

unsigned long cycleTimeStart = 0;
unsigned long cycleTimeFinish = 0;
unsigned long cycleTimeDuration = 0;

uint8_t useDefaultKeyForTransmission = 0;


// SETUP
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setup() {

#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial) { delay(1);};
#endif

  pinMode(triggerPin, INPUT_PULLUP);
  pinMode(extraButtonPin, INPUT_PULLUP);
  pinMode(hallSensorPin, INPUT);
  pinMode(batteryMeasurePin, INPUT);
  pinMode(vibrationActuatorPin, OUTPUT);

  pinMode(DiagLED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);

  digitalWrite(RFM69_RST, LOW);

  u8g2.begin();
  Serial.println("Draw start screen");
  drawStartScreen();

  loadFlashSettings();

  // Start Radio
  initiateTransmitter();

  // check if default encryptionKey is still in use and create custom one if needed
  // checkEncryptionKey();

  // Enter settings on startup if trigger is hold down
  if (triggerActive()) {
    changeSettings = true;
    drawTitle("Settings", 1000);
  }
}

// loop
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void loop() {
	
	cycleTimeStart = millis();
  detectButtonPress();
  calculateThrottlePosition();

  if (changeSettings == true) {
    controlSettingsMenu();
    updateMainDisplay();
  } else {
    remPackage.type = 0;
    remPackage.trigger = triggerActive();
    remPackage.throttle = throttle;

    if (transmitToReceiver()) {
      updateMainDisplay();
      } else if (changeSettings == true) {
        updateMainDisplay();
        }
    debugData.differenceJoinedSend = debugData.counterJoined - debugData.counterSend;
    debugData.differenceJoinedReceived = debugData.counterJoined - debugData.counterReceived;
  }

  cycleTimeFinish = millis();
  debugData.cycleTime = cycleTimeFinish - cycleTimeStart;
  if (debugData.cycleTime > debugData.longestCycleTime) {
      debugData.longestCycleTime = debugData.cycleTime;
    }
	#ifdef DEBUG
    Serial.print("cycleTimeStart: "); Serial.print(cycleTimeStart); Serial.println("ms");
    Serial.print("cycleTimeFinish: "); Serial.print(cycleTimeFinish); Serial.println("ms");
    Serial.print("cycleTime: "); Serial.print(debugData.cycleTime); Serial.println("ms");
    Serial.print("longestCycleTime: "); Serial.print(debugData.longestCycleTime); Serial.println("ms");
	#endif

}


// initiate radio
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void initiateTransmitter() {
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  if (!rf69_manager.init()) {
    while (1);
  }

  if (!rf69.setFrequency(RF69_FREQ)) {
  }
  if (useDefaultKeyForTransmission == 1) {
    rf69.setEncryptionKey(encryptionKey);
  } else {
    rf69.setEncryptionKey(txSettings.customEncryptionKey);
  }
  Serial.print("Set transmitter custom encryptionKey with boardID: ");
  for (uint8_t i = 0; i < 16; i++) {
    Serial.print(txSettings.customEncryptionKey[i]);
  }
  Serial.println("");
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
  Serial.print("Set frequecy to: "); Serial.println(RF69_FREQ);
  delay(250);
}

// check encryptionKey
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void checkEncryptionKey() {

  Serial.println("Check for default encription key");
  for (uint8_t i = 0; i < 16; i++) {
    Serial.print("Stored encryptionKey: "); Serial.println(txSettings.customEncryptionKey[i]);
    Serial.print("Default encryptionKey: "); Serial.println(encryptionKey[i]);

    if (txSettings.customEncryptionKey[i] == encryptionKey[i]) {

      if (i == 15 ) {
        Serial.println("Default key detected => createCustomKey()");
        createCustomKey();
      }

    } else {

      Serial.println("Custom key detected => setup/loop");
      break;

    }

  }

}

// create a new custom encryptionKey and send it to receiver
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void createCustomKey() {
  Serial.print("Create custom encryptionKey");
  uint8_t generatedCustomEncryptionKey[16];

  Serial.print("Custom encryptionKey: ");
  for (uint8_t i = 0; i < 16; i++) {
    generatedCustomEncryptionKey[i] = random(9);
    Serial.print(generatedCustomEncryptionKey[i]);
  }
  Serial.println("");

  generatedCustomEncryptionKey[15] = 1;
  for (uint8_t i = 0; i < 16; i++) {
    txSettings.customEncryptionKey[i] = generatedCustomEncryptionKey[i];
  }

  Serial.print("Custom encryptionKey with boardID: ");
  for (uint8_t i = 0; i < 16; i++) {
    Serial.print(txSettings.customEncryptionKey[i]);
  }
  Serial.println("");

  transmitSettingsToReceiver(); //TODO make shure receiver got message

  initiateTransmitter(); // restart receiver with new key
}

//Function used to transmit the remPackage and receive auto acknowledgment
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
bool transmitToReceiver() {

  transmissionTimeStart = millis();

  rf69_manager.setRetries(1);
  rf69_manager.setTimeout(20);

  debugData.counterJoined++;

  if (rf69_manager.sendtoWait((byte*)&remPackage, sizeof(remPackage), DEST_ADDRESS)) {
    uint8_t len = sizeof(returnData);
    uint8_t from;
	#ifdef DEBUG
    Serial.print("Needed retries OK: "); Serial.println(rf69_manager.retries());
    Serial.print("Normal transmission remPackage.type: "); Serial.println(remPackage.type);
	#endif
    debugData.counterSend++;

    if (rf69_manager.recvfromAckTimeout((uint8_t*)&returnData, &len, 20, &from)) { // TEST!
//    if (rf69_manager.recvfromAck((uint8_t*)&returnData, &len, &from)) {

#ifdef DEBUG
	    Serial.print("Amp hours: "); Serial.println(returnData.ampHours);
      Serial.print("Battery voltage: "); Serial.println(returnData.inpVoltage);
      Serial.print("Tachometer: "); Serial.println(returnData.tachometerAbs);
      Serial.print("Headlight active: "); Serial.println(returnData.headlightActive);
      Serial.print("Battery current: "); Serial.println(returnData.avgInputCurrent);
      Serial.print("Motor current: "); Serial.println(returnData.avgMotorCurrent);
      Serial.print("Duty cycle: "); Serial.println(returnData.dutyCycleNow);
#endif

      debugData.counterReceived++;
      transmissionTimeFinish = millis();
      debugData.transmissionTime = transmissionTimeFinish - transmissionTimeStart;
      debugData.rssi = rf69.lastRssi();

	  #ifdef DEBUG
      Serial.print("Got ack and reply from board #"); Serial.print(from);
      Serial.print(" transmission # "); Serial.print(counterRecived);
      Serial.print(" period: "); Serial.print(debugData.transmissionTime); Serial.print("ms");
      Serial.print(" [RSSI :"); Serial.print(rf69.lastRssi());
      Serial.println("]");
		#endif
      return true;

    } else {
		#ifdef DEBUG
		Serial.println("No reply, is anyone listening?");
    #endif
      return false;
    }
  } else {
	  
    return false;
  }
}

//Function used to transmit settings to receiver
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
bool transmitSettingsToReceiver() {

  remPackage.type = 1;

  Serial.println("Try to send receiver next package are settings");
  if ( transmitToReceiver()) {
    Serial.println("Send settings... ");
    if (rf69_manager.sendtoWait((byte*)&txSettings, sizeof(txSettings), DEST_ADDRESS)) {
      uint8_t len = sizeof(remPackage);
      uint8_t from;
      if (rf69_manager.recvfromAckTimeout((uint8_t*)&remPackage, &len, 200, &from)) {
        Serial.print("Received ack for settings from receiver #"); Serial.println(from);
        Serial.print("Received remPackage.type: "); Serial.println(remPackage.type);
      } else {
        Serial.println("Sending settings failed, no ack");
        remPackage.type = 0;
        return false;
      }
    } else {
      Serial.println("Sending settings failed, no one listening");
      remPackage.type = 0;
      return false;
    }
    Serial.println("Receiver have new Settings, restart transmitter");
    return true;
  } else {
    Serial.println("Could not tell the receiver next package are settings");
    remPackage.type = 0;
    return false;
  }
}

//Function used to transmit settings to receiver
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
bool transmitKeyToReceiver() {

  useDefaultKeyForTransmission = 1;

  initiateTransmitter();

  remPackage.type = 1;

  Serial.println("Try to send receiver next package are settings");
  if ( transmitToReceiver()) {
    Serial.println("Send settings... ");
    if (rf69_manager.sendtoWait((byte*)&txSettings, sizeof(txSettings), DEST_ADDRESS)) {
      uint8_t len = sizeof(remPackage);
      uint8_t from;
      if (rf69_manager.recvfromAckTimeout((uint8_t*)&remPackage, &len, 200, &from)) {
        Serial.print("Received ack for settings from receiver #"); Serial.println(from);
        Serial.print("Received remPackage.type: "); Serial.println(remPackage.type);
      } else {
        Serial.println("Sending settings failed, no ack");
        remPackage.type = 0;
        useDefaultKeyForTransmission = 0;
        initiateTransmitter();
        return false;
      }
    } else {
      Serial.println("Sending settings failed, no one listening");
      remPackage.type = 0;
      useDefaultKeyForTransmission = 0;
      initiateTransmitter();
      return false;
    }
    Serial.println("Receiver have new Settings, restart transmitter");
    useDefaultKeyForTransmission = 0;
    initiateTransmitter();
    return true;
  } else {
    Serial.println("Could not tell the receiver next package are settings");
    remPackage.type = 0;
    useDefaultKeyForTransmission = 0;
    initiateTransmitter();
    return false;
  }
}

// write boardID to the encryptionKey
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void selectBoard(uint8_t receiverID) {

  Serial.print("Join pairNewBoard(receiverID) receiverID: "); Serial.println(receiverID);

  txSettings.customEncryptionKey[15] = receiverID;

  Serial.print("Custom encryptionKey with boardID: ");
  for (uint8_t i = 0; i < 16; i++) {
    Serial.print(txSettings.customEncryptionKey[i]);
  }
  Serial.println("");

  initiateTransmitter(); // restart receiver with new key

  Serial.print("Exit pairNewBoard(receiverID)");

}

// pair with new board
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
bool pairNewBoard() {

  Serial.print("Join pairNewBoard()");
  useDefaultKeyForTransmission = 1;

  txSettings.customEncryptionKey[15] = txSettings.boardID;

  Serial.print("Custom encryptionKey with boardID: ");
  for (uint8_t i = 0; i < 16; i++) {
    Serial.print(txSettings.customEncryptionKey[i]);
  }
  Serial.println("");

  Serial.println("Send custom encryptionKey to receiver by default key");

  initiateTransmitter();

  transmitSettingsToReceiver();

  // set variable to use custom key for the next transmissions
  useDefaultKeyForTransmission = 0;
  // reset transmitter to activate custom key
  initiateTransmitter();
  delay(500);

  if (transmitToReceiver()) {
    drawMessage("Complete", "New board paired!", 2000);
  } else {
    drawMessage("Fail", "Board not paired", 2000);
  }

  Serial.print("Exit pairNewBoard()");
}

// Uses the throttle and trigger to navigate and change settings
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void controlSettingsMenu() {

  if (changeThisSetting == true) {

    if (currentSetting == EXIT) {
      changeSettings = false;
    }

    if (settingsLoopFlag == false && rules[currentSetting][0] != -1) {
      short value = getSettingValue(currentSetting);
      if ( throttlePosition == TOP ) {
        value++;
      } else if ( throttlePosition == BOTTOM ) {
        value--;
      }

      if (inRange(value, rules[currentSetting][1], rules[currentSetting][2])) {
        setSettingValue(currentSetting, value);
        if (settingChangeMillis == 0) {
          settingChangeMillis = millis();
        }
      }

      if (settingScrollFlag == true) {
        settingsLoopFlag = false;
        delay(20);
      } else {
        settingsLoopFlag = true;
      }
    }
    // If the throttle is at top or bottom for more than "settingScrollWait" allow the setting to change fast
    if ( millis() - settingChangeMillis >= settingScrollWait && settingChangeMillis != 0 ) {
      settingScrollFlag = true;
      settingsLoopFlag = false;
    } else {
      settingScrollFlag = false;
    }

  } else {

    if (settingsLoopFlag == false) {
      if (throttlePosition == TOP && currentSetting != 0) {
        Serial.print("Current Setting before currentSetting++ : "); Serial.println(currentSetting);
        currentSetting--;
        Serial.print("Current Setting after currentSetting-- : "); Serial.println(currentSetting);
        settingsLoopFlag = true;
      } else if (throttlePosition == BOTTOM && currentSetting < (numOfSettings - 1)) {
        Serial.print("Current Setting before currentSetting++ : "); Serial.println(currentSetting);
        currentSetting++;
        Serial.print("Current Setting after currentSetting++ : "); Serial.println(currentSetting);
        settingsLoopFlag = true;
      }
    }
  }

  // If thumbwheel is in middle position
  if ( throttlePosition == MIDDLE ) {
    settingsLoopFlag = false;
    settingChangeMillis = 0;
  }

  if ( triggerActive() ) {
    if (changeThisSetting == true && triggerFlag == false) {
      // Settings that needs to be transmitted to the recevier
      if (currentSetting == TRIGGER) {
        txSettings.triggerMode = getSettingValue(currentSetting);
        Serial.println("Settings menu - TRIGGER");
        if ( ! transmitSettingsToReceiver()) {
          loadFlashSettings();
          drawMessage("Failed", "No communication", 2000);
        } else {
          drawMessage("Complete", "Trigger mode changed", 2000);
        }
      } else if (currentSetting == MODE) {
        txSettings.controlMode = getSettingValue(currentSetting);
        Serial.println("Settings menu - MODE");
        // needed?? TODO
      } else if (currentSetting == TxPower) {
        txSettings.transmissionPower = getSettingValue(currentSetting);
        Serial.println("Settings menu - TxPower");
        if ( ! transmitSettingsToReceiver()) {
          loadFlashSettings();
          drawMessage("Failed", "No communication", 2000);
        } else {
          updateFlashSettings();
          initiateTransmitter();
          drawMessage("Complete", "Power changed", 2000);
        }
      } else if (currentSetting == BOARDID) {
        Serial.println("Settings menu - BOARDID");
        selectBoard(getSettingValue(currentSetting));
        drawMessage("Complete", "New board selected!", 2000);
      } else if (currentSetting == PAIR) {
        Serial.println("Settings menu - PAIR");
        pairNewBoard();
      } else if (currentSetting == KEY) {
        for (uint8_t i = 0; i < 16; i++) {
          txSettings.customEncryptionKey[i] = encryptionKey[1];
        }
        updateFlashSettings();
        initiateTransmitter();
        drawMessage("Complete", "Default encryption Key!", 2000);
      } else if (currentSetting == SETTINGS) {
        setDefaultFlashSettings();
        drawMessage("Complete", "Default settings loaded!", 2000);
      }
      updateFlashSettings();
    }

    if (triggerFlag == false) {
      changeThisSetting = !changeThisSetting;
      triggerFlag = true;
    }
  }
  else
  {
    triggerFlag = false;
  }
}

// Save the default settings in the FLASH
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setDefaultFlashSettings() {

  Serial.println("Load default flash settings");
  for ( uint8_t i = 0; i < numOfSettings; i++ )
  {
    setSettingValue( i, rules[i][0] );
    Serial.println(rules[i][0]);
  }
  Serial.print("Default encryptionKey: ");
  for ( uint8_t i = 0; i < 16 ; i++) {
    txSettings.customEncryptionKey[i] = encryptionKey[i];
    Serial.print(encryptionKey[i]);
  }
  txSettings.firmVersion = VERSION;
  Serial.println("");

  Serial.print("Killswitch: "); Serial.println(txSettings.triggerMode);
  Serial.print("Battery: "); Serial.println(txSettings.batteryType);
  Serial.print("Cell count: "); Serial.println(txSettings.batteryCells);
  Serial.print("Motor poles: "); Serial.println(txSettings.motorPoles);
  Serial.print("Motor pulley: "); Serial.println(txSettings.motorPulley);
  Serial.print("Wheel Pulley"); Serial.println(txSettings.wheelPulley);
  Serial.print("Wheel diameter: "); Serial.println(txSettings.wheelDiameter);
  Serial.print("Control mode: "); Serial.println(txSettings.controlMode);
  Serial.print("Min hall: "); Serial.println(txSettings.minHallValue);
  Serial.print("Center hall: "); Serial.println(txSettings.centerHallValue);
  Serial.print("Max hall: "); Serial.println(txSettings.maxHallValue);
  Serial.print("BoardID: "); Serial.println(txSettings.boardID);
  Serial.print("Transmission power: "); Serial.println(txSettings.transmissionPower);

  updateFlashSettings();
}

// load settings from flash
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void loadFlashSettings() {

  Serial.println("Load flash settings");

  txSettings = flash_TxSettings.read();

  if (txSettings.firmVersion != VERSION) {
    setDefaultFlashSettings();
  }
  else {
    updateFlashSettings();
  }

  calculateRatios();

}

// write/update settings to flash
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void updateFlashSettings() {

  Serial.println("Update flash settings");

  flash_TxSettings.write(txSettings);

  calculateRatios();
}

// Detect button press
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void detectButtonPress() {
  if (millis() - buttonPrevMillis >= buttonSampleIntervalsMs) {
    buttonPrevMillis = millis();
    byte currButtonState = digitalRead(extraButtonPin);
    if ((prevButtonState == HIGH) && (currButtonState == LOW)) {
      buttonPress();
    }
    else if ((prevButtonState == LOW) && (currButtonState == HIGH)) {
      buttonRelease();
    }
    else if (currButtonState == LOW) {
      buttonPressCount++;
	  #ifdef DEBUG
      Serial.print("buttonPressCount: "); Serial.println(buttonPressCount);
	  #endif
      if (buttonPressCount >= longbuttonPressCountMax) {
        longbuttonPress();
      }
    }
    prevButtonState = currButtonState;
  }
}

// Update values used to calculate speed and distance travelled.
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void calculateRatios() {
  // Gearing ratio
  gearRatio = (float)txSettings.motorPulley / (float)txSettings.wheelPulley;
  // ERPM to Km/h
  ratioRpmSpeed = (gearRatio * 60 * (float)txSettings.wheelDiameter * 3.14156) / (((float)txSettings.motorPoles / 2) * 1000000);
  // Pulses to km travelled
  ratioPulseDistance = (gearRatio * (float)txSettings.wheelDiameter * 3.14156) / (((float)txSettings.motorPoles * 3) * 1000000);
}

// Get settings value by index (usefull when iterating through settings)
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
short getSettingValue(uint8_t index) {
  short value;
  switch (index) {
    case TRIGGER:   value = txSettings.triggerMode; break;
    case 1:     value = txSettings.batteryType;     break;
    case 2:     value = txSettings.batteryCells;    break;
    case 3:     value = txSettings.motorPoles;      break;
    case 4:     value = txSettings.motorPulley;     break;
    case 5:     value = txSettings.wheelPulley;     break;
    case 6:     value = txSettings.wheelDiameter;   break;
    case MODE:  value = txSettings.controlMode;     break;
    case 8:     value = txSettings.minHallValue;    break;
    case 9:     value = txSettings.centerHallValue; break;
    case 10:    value = txSettings.maxHallValue;    break;
    case 11:    value = txSettings.boardID;         break;
    case 12:    value = 0;                          break;
    case 13:    value = txSettings.transmissionPower; break;
    case 14:    value = 0;                          break;

    default: /* Do nothing */ break;
  }
  return value;
}

// Set a value of a specific setting by index.
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setSettingValue(uint8_t index, uint64_t value) {
  switch (index) {
    case TRIGGER:   txSettings.triggerMode = value;     break;
    case 1:         txSettings.batteryType = value;     break;
    case 2:         txSettings.batteryCells = value;    break;
    case 3:         txSettings.motorPoles = value;      break;
    case 4:         txSettings.motorPulley = value;     break;
    case 5:         txSettings.wheelPulley = value;     break;
    case 6:         txSettings.wheelDiameter = value;   break;
    case MODE:      txSettings.controlMode = value;     break;
    case 8:         txSettings.minHallValue = value;    break;
    case 9:         txSettings.centerHallValue = value; break;
    case 10:        txSettings.maxHallValue = value;    break;
    case 11:        txSettings.boardID = value;         break;
    case 13:        txSettings.transmissionPower = value; break;

    default: /* Do nothing */ break;
  }
}

// Check if an integer is within a min and max value
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
bool inRange(short val, short minimum, short maximum) {
  return ((minimum <= val) && (val <= maximum));
}

// Return true if trigger is activated, false otherwise
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
bool triggerActive() {
  if (digitalRead(triggerPin) == LOW)
    return true;
  else
    return false;
}


// Update the OLED for each loop
// To-Do: Only update display when needed
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void updateMainDisplay()
{
  u8g2.firstPage();

  do {
    if ( changeSettings == true )
    {
      drawSettingsMenu();
    }
    else
    {
      drawThrottle();
      drawBatteryLevel();
      drawSignal();
      drawHeadlightStatus();
      drawPage();
    }
  } while ( u8g2.nextPage() );
}

// Measure the hall sensor output and calculate throttle posistion
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void calculateThrottlePosition()
{
  // Hall sensor reading can be noisy, lets make an average reading.
  uint16_t total = 0;
  uint8_t samples = 10;

  for ( uint8_t i = 0; i < samples; i++ )
  {
    total += analogRead(hallSensorPin);
  }

  hallValue = total / samples;

  if ( hallValue >= txSettings.centerHallValue )
  {
    throttle = constrain( map(hallValue, txSettings.centerHallValue, txSettings.maxHallValue, centerThrottle, 1023), centerThrottle, 1023 );
  } else {
    throttle = constrain( map(hallValue, txSettings.minHallValue, txSettings.centerHallValue, 0, centerThrottle), 0, centerThrottle );
  }

  // Remove hall center noise
  if ( abs(throttle - centerThrottle) < hallNoiseMargin )
  {
    throttle = centerThrottle;
  }
#ifdef DEBUG
  Serial.print("Throttle: "); Serial.println(throttle);
  #endif

  // Find the throttle positions
  if (throttle >= (centerThrottle + hallMenuMargin)) {
    throttlePosition = TOP;
  }
  else if (throttle <= (centerThrottle - hallMenuMargin)) {
    throttlePosition = BOTTOM;
  }
  else if ( inRange( throttle, (centerThrottle - hallMenuMargin), (centerThrottle + hallMenuMargin) ) ) {
    throttlePosition = MIDDLE;
  }
}

// Calculate the remotes battery level
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
uint8_t batteryLevel() {

  uint16_t total = 0;
  uint8_t samples = 10;

  for (uint8_t i = 0; i < samples; i++) {
    total += analogRead(batteryMeasurePin);
  }

  float voltage = (refVoltage * 2 / 1024.0) * ( (float)total / (float)samples );

  if (voltage <= minVoltage) {
    return 0;
  } else if (voltage >= maxVoltage) {
    return 100;
  }

  return (voltage - minVoltage) * 100 / (maxVoltage - minVoltage);
}

// Calculate the battery level of the board based on the telemetry voltage
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
float batteryPackPercentage( float voltage ) {

  float maxCellVoltage = 4.2;
  float minCellVoltage;

  if (txSettings.batteryType == 0) {
    // Li-ion
    minCellVoltage = 3.1;
  }
  else
  {
    // Li-po
    minCellVoltage = 3.4;
  }

  float percentage = (100 - ( (maxCellVoltage - voltage / txSettings.batteryCells) / ((maxCellVoltage - minCellVoltage)) ) * 100);

  if (percentage > 100.0) {
    return 100.0;
  } else if (percentage < 0.0) {
    return 0.0;
  }

  return percentage;

}

// check battery level and set alarm - 120 seconds blocked(not implemented yet)
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void checkBatteryLevel() {
  float boardBattery;
  uint16_t remoteBattery;
  uint16_t boardBatteryAbs;

  boardBattery = batteryPackPercentage( returnData.inpVoltage );
  boardBatteryAbs = abs( floor(boardBattery) );
  remoteBattery = batteryLevel();
  if (((boardBattery <= 15) || (remoteBattery <= 15) || alarmTriggered) && !batteryAlarmBlocked) {
    if (millis() - lastAlarmBlink > 3000) {
      alarmBlink = !alarmBlink;
      lastAlarmBlink = millis();
    }

    if (alarmBlink == true) {
      alarmTriggered = true;
      alarmActivated();
    } else {
      alarmTriggered = false;
      batteryAlarmBlocked = true;
    }
  }
  if (batteryAlarmBlocked = true) {
    if (millis() - lastBlockBlink > 120000) {
      blockBlink = !blockBlink;
      lastBlockBlink = millis();
    }
    if (blockBlink == true) {
      batteryAlarmBlocked = true;
    } else {
      batteryAlarmBlocked = false;
    }
  }
}

// Vibrate every 500ms for 500ms as long as function is called
void alarmActivated() {
  if (millis() - lastVibrationBlink > 500) {
    vibrationBlink = !vibrationBlink;
    lastVibrationBlink = millis();
  }

  if (vibrationBlink == true) {
    digitalWrite(vibrationActuatorPin, HIGH);
  } else {
    digitalWrite(vibrationActuatorPin, LOW);
  }
}

// Prints the settings menu on the OLED display
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void drawSettingsMenu() {
  // Local variables to store the setting value and unit
  uint64_t value;

  x = 0;
  y = 10;

  // Print setting title
  u8g2.setFont(u8g2_font_profont12_tr);
  u8g2.drawStr(x, y, titles[ currentSetting ] );

  // Get current setting value
  switch (currentSetting) {
    default:
      value = getSettingValue(currentSetting);
      break;

  }

  // Check if there is a text string for the setting value
  if ( valueIdentifier[currentSetting] != 0 )
  {
    uint8_t index = valueIdentifier[ currentSetting ] - 1;
    tString = stringValues[ index ][ value ];
  } else { // else normal value from txSettings
    tString = uint64ToString(value);
  }
  

  if ( unitIdentifier[ currentSetting ] != 0 ) {
    tString += settingUnits[ unitIdentifier[ currentSetting ] - 1 ];
  }

  if ( currentSetting == EXIT ) {
    tString = F("Exit");
  }

  if ( currentSetting == PAIR ) {
    tString = F("Pair now");
  }

  if ( currentSetting == DEFAULTKEY ) {
    tString = F("Restore");
  }

  if ( currentSetting == KEY ) {
    for (uint8_t i = 7; i < 16; i++) {
      tString += String(txSettings.customEncryptionKey[i]);
    }
  }

  if ( currentSetting == SETTINGS ) {
    tString = F("Reset");
  }

  if ( currentSetting == FIRMWARE ) {
    tString = String(txSettings.firmVersion);
  }

  if ( changeThisSetting == true ) {
    drawString(tString, tString.length(), x + 10, y + 20, u8g2_font_10x20_tr );

    // If setting has something to do with the hallValue
    if ( inRange(currentSetting, 8, 10) ) {
      tString = "(" + String(hallValue) + ")";
      drawString(tString, tString.length(), x + 92, y + 20, u8g2_font_profont12_tr );
    }
  } else {
    drawString(tString, tString.length(), x, y + 20, u8g2_font_10x20_tr );
  }
}


// Print the startup screen
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void drawStartScreen() {
  u8g2.firstPage();

  do {

    u8g2.drawXBMP( 4, 4, 24, 24, logo);

    u8g2.setFont(u8g2_font_10x20_tr);
    u8g2.drawStr(20, 26, "Firefly");
    u8g2.setFont(u8g2_font_t0_12_tr);
    u8g2.drawStr(30, 41, "mod by StefanMe");

  } while ( u8g2.nextPage() );

  delay(200);
}

// Print a title on the OLED
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void drawTitle(String title, uint16_t duration) {
  u8g2.firstPage();

  do {

    drawString(title, 20, 5, 30, u8g2_font_10x20_tr );

  } while ( u8g2.nextPage() );

  delay(duration);
}

// Print a message on OLED
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void drawMessage(String title, String details, uint16_t duration) {
  u8g2.firstPage();

  do {

    drawString(title, 20, 1, 20, u8g2_font_10x20_tr);
    drawString(details, 30, 5, 35, u8g2_font_t0_12_tr);

  } while ( u8g2.nextPage() );

  delay(duration);
}

// Extra button handling
// called when key goes from pressed to not pressed
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void buttonRelease() {
  if (buttonPressCount < longbuttonPressCountMax && buttonPressCount >= mediumbuttonPressCountMin) {
    mediumbuttonPress();
  }
  else {
    if (buttonPressCount < mediumbuttonPressCountMin) {
      shortbuttonPress();
    }
  }
}

// called when key goes from not pressed to pressed
void buttonPress() {

  buttonPressCount = 0;
}

// called when button is kept pressed for less than .5 seconds
void shortbuttonPress() {

  displayView++;
}

// called when button is kept pressed for more than 2 seconds
void mediumbuttonPress() {
  if ( returnData.headlightActive == 1 ) {

    remPackage.headlight = 0;
    Serial.println("Lights OFF");
  } else {

    remPackage.headlight = 1;
    Serial.println("Lights ON");
  }

}

// called when button is kept pressed for 2 seconds or more
void longbuttonPress() {
  changeSettings = true;
}

// draw main page
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void drawPage() {

  uint8_t decimalsMain, decimalsSecond, decimalsThird;
  float valueMain, valueSecond, valueThird;
  int unitMain, unitSecond, unitThird;
  uint16_t first, last, firstSecond, lastSecond, firstThird, lastThird;

  x = 0;
  y = 15;

  // handle rotation of different views
  // - first view: Speed, voltage, distance
  // - second view; voltage, battery amps, motor amps
  if (displayView > 3) {
    displayView = 0;
  }

  switch (displayView) {
    case 0:
      valueMain = ratioRpmSpeed * returnData.rpm;
      decimalsMain = 1;
      unitMain = 1;
      valueSecond = returnData.inpVoltage;
      decimalsSecond = 2;
      unitSecond = 0;
      valueThird = ratioPulseDistance * returnData.tachometerAbs;
      decimalsThird = 2;
      unitThird = 2;
      break;
    case 1:
      valueMain = returnData.inpVoltage;
      decimalsMain = 1;
      unitMain = 0;
      valueSecond = returnData.avgInputCurrent;
      decimalsSecond = 1;
      unitSecond = 3;
      valueThird = returnData.avgMotorCurrent;
      decimalsThird = 1;
      unitThird = 3;
      break;
    case 2:
      valueMain = debugData.rssi;
      decimalsMain = 1;
      unitMain = 5;
      valueSecond = debugData.transmissionTime;
      decimalsSecond = 1;
      unitSecond = 4;
      valueThird = debugData.cycleTime;
      decimalsThird = 1;
      unitThird = 4;
      break;
    case 3:
      valueMain = debugData.longestCycleTime;
      decimalsMain = 1;
      unitMain = 4;
      valueSecond = debugData.differenceJoinedSend;
      decimalsSecond = 1;
      unitSecond = 6;
      valueThird = debugData.differenceJoinedReceived;
      decimalsThird = 1;
      unitThird = 6;
      break;
  }

  // Display prefix (title)
  u8g2.setFont(u8g2_font_profont12_tr);
  u8g2.drawStr(x, y - 1, dataPrefix[ displayView ] );

  // Split up the float value: a number, b decimals.
  first = abs( floor(valueMain) );
  last = (valueMain - first) * pow(10, decimalsMain);

  // Draw main decimals
  tString = last;
  drawString(tString, decimalsMain, x + 35, y + 12, u8g2_font_profont12_tr);

  // Add leading zero main numbers
  if ( first <= 9 ) {
    tString = "0" + String(first);
  } else {
    tString = first;
  }
  // Display main numbers
  drawString(tString, 10, x, y + 29, u8g2_font_logisoso26_tn );

  // Display decimals
  tString = ".";
  if ( last <= 9 && decimalsMain > 1) {
    tString += "0";
  }

  // Display main units
  u8g2.setFont(u8g2_font_profont12_tr);
  u8g2.drawStr( x + 35, y + 29, dataSuffix[unitMain]);


  // Convert valueSecond to string
  firstSecond = abs( floor(valueSecond) );
  // Display second numbers
  if ( firstSecond <= 9 ) {
    tString = "0" + String(firstSecond);
  } else {
    tString = firstSecond;
  }
  drawString(tString, 10, x + 60, y + 29, u8g2_font_logisoso18_tn );
  // Display second units
  u8g2.setFont(u8g2_font_profont12_tr);
  u8g2.drawStr( x + 60 + 25, y + 29, dataSuffix[unitSecond]);


  // Convert valueThird to string
  firstThird = abs( floor(valueThird) );
  if ( firstThird <= 9 ) {
    tString = "0" + String(firstThird);
  } else {
    tString = firstThird;
  }
  // Display third numbers
  drawString(tString, 10, x + 60, y + 29 - 20, u8g2_font_logisoso18_tn );
  // Display main units
  u8g2.setFont(u8g2_font_profont12_tr);
  u8g2.drawStr( x + 60 + 25, y + 29 - 20, dataSuffix[unitThird]);
}

/*
   Prepare a string to be displayed on the OLED
*/
void drawString(String string, uint8_t lenght, uint8_t x, uint8_t y, const uint8_t *font) {

  static char cache[40];

  string.toCharArray(cache, lenght + 1);

  u8g2.setFont(font);
  u8g2.drawStr(x, y, cache);

}

/*
   Print the throttle value as a bar on the OLED
*/
void drawThrottle() {

  x = 0;
  y = 55;

  uint8_t width;

  // Draw throttle
  u8g2.drawHLine(x, y, 126);
  u8g2.drawVLine(x, y, 10);
  u8g2.drawVLine(x + 15, y, 3);
  u8g2.drawVLine(x + 31, y, 5);
  u8g2.drawVLine(x + 46, y, 3);
  u8g2.drawVLine(x + 63, y, 5);
  u8g2.drawVLine(x + 78, y, 3);
  u8g2.drawVLine(x + 94, y, 5);
  u8g2.drawVLine(x + 110, y, 3);
  u8g2.drawVLine(x + 126, y, 20);

  if (throttle >= 512) {
    width = map(throttle, 512, 1023, 0, 62);

    for (uint8_t i = 0; i < width; i++)
    {
      u8g2.drawVLine(x + 63 - i, y + 4, 4);
    }
  } else {
    width = map(throttle, 0, 511, 62, 0);

    for (uint8_t i = 0; i < width; i++)
    {
      u8g2.drawVLine(x + 63 + i, y + 4, 4);
    }
  }
}

void drawSignal() {

  x = 125;
  y = 23;

  u8g2.drawCircle(x, y, 2);

  if (debugData.cycleTime > 130) {
    } else {
      u8g2.drawCircle(x, y, 3);
	  u8g2.drawCircle(x, y, 4);
      }
  
}

/*
   Print the remotes battery level as a battery on the OLED
*/
void drawBatteryLevel() {

  x = 108;
  y = 4;

  uint8_t level = batteryLevel();

  u8g2.drawFrame(x + 2, y, 18, 9);
  u8g2.drawBox(x, y + 2, 2, 5);

  for (uint8_t i = 0; i < 5; i++) {
    uint8_t p = round((100 / 5) * i);
    if (p <= level)
    {
      u8g2.drawBox(x + 4 + (3 * i), y + 2, 2, 5);
    }
  }
}

/*
   Print status of the extra output from board on the OLED
*/
void drawHeadlightStatus() {

  x = 122;
  y = 39;

  u8g2.drawDisc(x , y , 5, U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawDisc(x , y , 5, U8G2_DRAW_LOWER_RIGHT);
  u8g2.drawLine(x - 1 , y - 3, x - 1, y + 3);

  if (remPackage.headlight == 1) {
    u8g2.drawLine(x - 3 , y, x - 5, y);
    u8g2.drawLine(x - 3 , y + 3, x - 5, y + 4);
    u8g2.drawLine(x - 3 , y - 3, x - 5, y - 4);
  }
}

String uint64ToString(uint64_t number)
{
  unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
  unsigned long part2 = (unsigned long)((number));

  if (part1 == 0) {
    return String(part2, DEC);
  }
  return String(part1, DEC) + String(part2, DEC);
}

/*
   Convert hex String to uint64_t: http://forum.arduino.cc/index.php?topic=233813.0
*/
uint64_t StringToUint64( char * string ) {
  uint64_t x = 0;
  char c;

  do {
    c = hexCharToBin( *string++ );
    if (c < 0)
      break;
    x = (x << 4) | c;
  } while (1);

  return x;
}

String uint64ToAddress(uint64_t number)
{
  unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
  unsigned long part2 = (unsigned long)((number));

  return String(part1, HEX) + String(part2, HEX);
}

char hexCharToBin(char c) {
  if (isdigit(c)) {  // 0 - 9
    return c - '0';
  } else if (isxdigit(c)) { // A-F, a-f
    return (c & 0xF) + 9;
  }
  return -1;
}
