// FeatherFly Transmitter - eSk8 Remote

#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <FlashStorage.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

//#define DEBUG

#define VERSION 1.0

// Defining the type of display used (128x64)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const unsigned char logo[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xFC, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
  0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0x1F, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xE0, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF,
  0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0x03, 0x00, 0x00,
  0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x80, 0xFF, 0xFF,
  0xFF, 0x1F, 0x00, 0x00, 0x00, 0x80, 0xFF, 0xFF, 0xE7, 0x1F, 0x00, 0x00,
  0x00, 0x84, 0xFF, 0xFF, 0xF3, 0x1F, 0x04, 0x00, 0x00, 0x86, 0xFF, 0xFF,
  0xD1, 0x1F, 0x0C, 0x00, 0x80, 0x87, 0xFF, 0xFF, 0xF9, 0x1F, 0x3C, 0x00,
  0xC0, 0x87, 0xFF, 0xFF, 0xE8, 0x1F, 0x7C, 0x00, 0xE0, 0x87, 0xFF, 0x7F,
  0xFC, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0xFF, 0x3F, 0xF4, 0x1F, 0xFC, 0x00,
  0xE0, 0x87, 0xFF, 0x1F, 0xFC, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0xFF, 0x0B,
  0xFE, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0xFF, 0x05, 0xFE, 0x1F, 0xFC, 0x00,
  0xE0, 0x87, 0xFF, 0x02, 0xFF, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0x7F, 0x01,
  0xFF, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0xBF, 0x80, 0xEF, 0x1F, 0xFC, 0x00,
  0xE0, 0x87, 0xDF, 0x80, 0xFF, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0x6F, 0x00,
  0x80, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0x3F, 0x00, 0x00, 0x1C, 0xFC, 0x00,
  0xE0, 0x87, 0x1F, 0x00, 0x00, 0x1E, 0xFC, 0x00, 0xE0, 0x87, 0x0F, 0x00,
  0x00, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0x07, 0x00, 0x80, 0x1F, 0xFC, 0x00,
  0xE0, 0x87, 0x03, 0x00, 0x80, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0xFF, 0x3F,
  0x40, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0xFF, 0x3F, 0xA0, 0x1F, 0xFC, 0x00,
  0xE0, 0x87, 0xFF, 0x17, 0xD0, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0xFF, 0x1F,
  0xF8, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0xFF, 0x0B, 0xFC, 0x1F, 0xFC, 0x00,
  0xE0, 0x87, 0xFF, 0x0F, 0xFE, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0xFF, 0x07,
  0xFB, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0xFF, 0x07, 0xFD, 0x1F, 0xFC, 0x00,
  0xE0, 0x87, 0xFF, 0x83, 0xFF, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0xFF, 0xC3,
  0xFF, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0xFF, 0xE1, 0xFF, 0x1F, 0xFC, 0x00,
  0xE0, 0x87, 0x7F, 0xF1, 0xFF, 0x1F, 0xFC, 0x00, 0xE0, 0x87, 0xFF, 0xF9,
  0xFF, 0x1F, 0xFC, 0x00, 0xC0, 0x87, 0xBF, 0xFC, 0xFF, 0x1F, 0x7C, 0x00,
  0x80, 0x87, 0x7F, 0xFC, 0xFF, 0x1F, 0x3C, 0x00, 0x00, 0x86, 0x7F, 0xFF,
  0xFF, 0x1F, 0x0C, 0x00, 0x00, 0x84, 0xFF, 0xFF, 0xFF, 0x1F, 0x04, 0x00,
  0x00, 0x80, 0xFF, 0xFF, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x80, 0xFF, 0xFF,
  0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0x07, 0x00, 0x00,
  0x00, 0x00, 0xF8, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF,
  0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0x3F, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x80, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE,
  0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x01, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const unsigned char connectedIcon[] PROGMEM = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x09,
  0x33, 0x09, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const unsigned char noconnectionIcon[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x09,
  0x00, 0x09, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char eStopArmed[] PROGMEM = {
  0xFF, 0x0F, 0x01, 0x08, 0x7D, 0x0B, 0x3D, 0x0B, 0x3D, 0x0B, 0x3D, 0x0B,
  0x3D, 0x0B, 0x3D, 0x0B, 0x3D, 0x09, 0x39, 0x09, 0x32, 0x04, 0x34, 0x02,
  0x28, 0x01, 0xB0, 0x00, 0x60, 0x00,
};

// Transmit and receive package
struct debug {
  unsigned long cycleTime;
  unsigned long transmissionTime;
  int8_t rssi;
  unsigned long counterJoined;
  unsigned long counterSend;
  unsigned long counterReceived;
  unsigned long differenceJoinedSend;
  unsigned long differenceJoinedReceived;
  unsigned long longestCycleTime;
  unsigned long lastTransmissionAvaible;
} debugData;


// Defining struct to hold setting values while remote is turned on.
typedef struct {
  uint8_t boardID;                  // 0
  uint8_t triggerMode;              // 1
  uint8_t batteryType;              // 2
  uint8_t batteryCells;             // 3
  uint8_t motorPoles;               // 4
  uint8_t motorPulley;              // 5
  uint8_t wheelPulley;              // 6
  uint8_t wheelDiameter;            // 7
  uint8_t controlMode;              // 8
  short minHallValue;               // 9
  short centerHallValue;            // 10
  short maxHallValue;               // 11
  uint8_t eStopMode;                // 12
  uint8_t breaklightMode;           // 13
  uint8_t throttleDeath;            // 14
  uint8_t drivingMode;              // 15
  uint8_t pairNewBoard;             // 16
  uint8_t transmissionPower;        // 17
  uint8_t customEncryptionKey[16];  // 18
  float firmVersion;                // 19
  bool eStopArmed;                  // 20
} TxSettings;

TxSettings txSettings;

// Defining flash storage
FlashStorage(flash_TxSettings, TxSettings);

uint8_t currentSetting = 0;
const uint8_t numOfSettings = 23;

// Setting rules format: default, min, max.
const short rules[numOfSettings][3] {
  {1, 0, 9},          //0 boardID
  {1, 0, 1},          //1 0: Killswitch  | 1: Cruise control
  {1, 0, 1},          //2 0: Li-ion      | 1: LiPo
  {10, 6, 12},        //3 Cell count
  {14, 0, 250},       //4 Motor poles
  {14, 0, 250},       //5 Motor pully
  {38, 0, 250},       //6 Wheel pulley
  {80, 0, 250},       //7 Wheel diameter
  {1, 0, 2},          //8 0: PPM only   | 1: PPM and UART | 2: UART only
  {316, 0, 400},      //9 Min hall value
  {490, 300, 700},    //10 Center hall value
  {645, 600, 1023},   //11 Max hall value
  { 0, 0, 2},         //12 EStop mode |0soft|1hard|2off
  { 0, 0, 2},         //13 breaklight mode |0off|1alwaysOn|onWithheadlight
  { 10, 0, 30},      //14 throttle death center
  { 2, 0, 2},         //15 Driving Mode
  { -1, 0, 0},        //16 pair new board
  {20, 14, 20},       //17 transmission power
  { -1, 0, 0},        //18 show Key
  { -1, 0 , 0},       //19 Firmware
  { -1, 0, 0},        //20 Set default key
  { -1, 0, 0},        //21 Settings
  { -1, 0, 0}         //22 Exit
};

// Defining constants to hold the special settings, so it's easy changed though the code
#define TRIGGER     1
#define MODE        8
#define BOARDID     0
#define TXPOWER     13
#define DRIVINGMODE 15
#define KEY         18
#define FIRMWARE    19
#define PAIR        16
#define DEFAULTKEY  20
#define SETTINGS    21
#define EXIT        22

const char titles[numOfSettings][19] = {
  "Board ID", "Trigger use", "Battery type", "Battery cells", "Motor poles", "Motor pulley",
  "Wheel pulley", "Wheel diameter", "Control mode", "Throttle min", "Throttle center",
  "Throttle max", "Estop Mode", "Breaklight Mode", "Throttle Death", "Driving Mode", "Pair new Board", "Transmission Power", "Encyption key",
  "Firmware Version", "Set default key", "Settings", "Exit"
};

const uint8_t unitIdentifier[numOfSettings]  =  {4, 0, 0, 1, 0, 2, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0};
const uint8_t valueIdentifier[numOfSettings] =  {0, 1, 2, 0, 0, 0, 0, 0, 3, 0, 0, 0, 4, 5, 0, 6, 0, 0, 0, 0, 0, 0};

const char stringValues[6][3][15] = {
  {"Killswitch", "Cruise", ""},
  {"Li-ion", "LiPo", ""},
  {"PPM", "PPM and UART", "UART only"},
  {"soft", "hard", "off"},
  {"off", "Always on", "with headlight"},
  {"Beginner", "Intermidiate", "Pro"},
};
const char settingUnits[5][4] = {"S", "T", "mm", "#", "dBm"};

const char dataSuffix[7][4] = {"V", "KMH", "KM", "A","ms","dBm", ""};
const char dataPrefix[4][13] = {"SPEED", "POWER", "CYCLETIME", "CONNECT"};

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
  bool eStopArmed;
} returnData;

// defining button data
unsigned long buttonPrevMillis = 0;
const unsigned long buttonSampleIntervalsMs = 200;
byte longbuttonPressCountMax = 12;    // 80 * 25 = 2000 ms
byte mediumbuttonPressCountMin = 4;    // 20 * 25 = 500 ms
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
const uint8_t hallSensorPin = A5;
const uint8_t vibrationActuatorPin = A4;

// Definition for RFM69HW radio on Feather m0
#define RFM69_CS     8
#define RFM69_INT   3
#define RFM69_RST   4
#define DIAGLED     13
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
unsigned long sendFailCounterRow = 0;


// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";
uint8_t transmissionFailCounter = 0;
bool connectionLost = false;

// define variable for battery alert
bool alarmTriggered = false;

// Battery monitering
const float minVoltage = 3.1;
const float maxVoltage = 4.2;
const float refVoltage = 3.3;

// Defining variables for Hall Effect throttle.
uint16_t hallValue, throttle;
const uint16_t centerThrottle = 512;
uint8_t hallNoiseMargin = 10;
const uint8_t hallMenuMargin = 100;
uint8_t throttlePosition;

#define TOP 0
#define MIDDLE 1
#define BOTTOM 2

// Defining variables for OLED display
String tString;
uint8_t displayView = 6;
uint8_t x, y;

// Defiing varibales for signal
unsigned long lastSignalBlink;
bool signalBlink = false;

// Defining variables for alarm
unsigned long lastAlarmBlink;
bool alarmBlink = false;
bool batteryAlarmBlocked = false;
unsigned long lastBlockBlink;
bool blockBlink = false;

// Defining variables for vibration actuator
unsigned long alarmBlinkDuration = 0;
uint8_t alarmBlinkTimes = 0;
uint8_t alarmBlinkCounter = 0;
uint8_t alarmBlinkTimer = 0;


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
uint8_t headlightStatusUpdated = 0;

unsigned long announcmentTimer = 0;

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

  randomSeed(analogRead(A0));

  pinMode(DIAGLED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);

  digitalWrite(RFM69_RST, LOW);

  u8g2.setDisplayRotation(U8G2_R3);
  u8g2.begin();
  Serial.println("Draw start screen");
  drawStartScreen();

  loadFlashSettings();

  // check if default encryptionKey is still in use and create custom one if needed
  checkEncryptionKey();

  // Start Radio
  initiateTransmitter();

  // Enter settings on startup if trigger is hold down
  if (triggerActive()) {
      txSettings.eStopArmed = false;
      transmitSettingsToReceiver();
      changeSettings = true;
      u8g2.setDisplayRotation(U8G2_R0);
      drawTitle("Settings", 1500);
  }
}

// loop
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void loop() {

  cycleTimeStart = millis();
  detectButtonPress();
  calculateThrottlePosition();
  checkConnection();
  checkBatteryLevel();
  controlVib();

  if (changeSettings == true) {
    controlSettingsMenu();
    updateMainDisplay();
  } else {
    remPackage.type = 0;
    remPackage.trigger = triggerActive();
    remPackage.throttle = throttle;

    if (transmitToReceiver(1,30)) {
      updateMainDisplay();
      debugData.lastTransmissionAvaible = millis();
    }

    if (connectionLost) {
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

}

// check connection
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
 void checkConnection() {

    if (millis() - debugData.lastTransmissionAvaible > 300) {
      connectionLost = true;
      alarmBlinkDuration = 200;
      alarmBlinkTimes = 12;
      alarmTriggered = true;
      returnData.eStopArmed = false;
    } else {
      connectionLost = false;
    }
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
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
  delay(10);
}

// check encryptionKey
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void checkEncryptionKey() {

  for (uint8_t i = 0; i < 16; i++) {
    Serial.print("Stored encryptionKey: "); Serial.println(txSettings.customEncryptionKey[i]);
    //Serial.print("Default encryptionKey: "); Serial.println(encryptionKey[i]);

    if (txSettings.customEncryptionKey[i] == 0) {

      if (i == 15 ) {
        Serial.println("Default key detected => createCustomKey()");
        //createCustomKey();
        createTestKey();
      }

    } else {

      Serial.println("Custom key detected => setup/loop");
      break;

    }

  }

}

// create a test key
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void createTestKey() {
  uint8_t generatedCustomEncryptionKey[16];

  for (uint8_t i = 0; i < 16; i++) {
    generatedCustomEncryptionKey[i] = encryptionKey[i];
  }

  generatedCustomEncryptionKey[15] = 1;
  for (uint8_t i = 0; i < 16; i++) {
    txSettings.customEncryptionKey[i] = generatedCustomEncryptionKey[i];
  }

  updateFlashSettings();

}

// create a new custom encryptionKey and send it to receiver
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void createCustomKey() {
  uint8_t generatedCustomEncryptionKey[16];

  for (uint8_t i = 0; i < 16; i++) {
    generatedCustomEncryptionKey[i] = random(9);
    Serial.print(generatedCustomEncryptionKey[i]);
  }
  Serial.println("");

  generatedCustomEncryptionKey[15] = 1;
  for (uint8_t i = 0; i < 16; i++) {
    txSettings.customEncryptionKey[i] = generatedCustomEncryptionKey[i];
  }

  updateFlashSettings();

}

//Function used to transmit the remPackage and receive auto acknowledgment
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
bool transmitToReceiver(uint8_t retries, uint8_t timeout) {
  transmissionTimeStart = millis();

  rf69_manager.setRetries(retries);
  rf69_manager.setTimeout(timeout);

  debugData.counterJoined++;

  if (rf69_manager.sendtoWait((byte*)&remPackage, sizeof(remPackage), DEST_ADDRESS)) {
    uint8_t len = sizeof(returnData);
    uint8_t from;

    debugData.counterSend++;

    if (rf69_manager.recvfromAckTimeout((uint8_t*)&returnData, &len, 20, &from)) { // TEST!

      debugData.counterReceived++;
      transmissionTimeFinish = millis();
      debugData.transmissionTime = transmissionTimeFinish - transmissionTimeStart;
      debugData.rssi = rf69.lastRssi();
      return true;
    } else {
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
  txSettings.eStopArmed = false;

  rf69_manager.setRetries(3);
  rf69_manager.setTimeout(100);

  if ( transmitToReceiver(3,100)) {
    if (rf69_manager.sendtoWait((byte*)&txSettings, sizeof(txSettings), DEST_ADDRESS)) {
      uint8_t len = sizeof(remPackage);
      uint8_t from;
      if (rf69_manager.recvfromAckTimeout((uint8_t*)&remPackage, &len, 200, &from)) {
      } else {
        remPackage.type = 0;
        return false;
      }
    } else {
      remPackage.type = 0;
      return false;
    }
    return true;
  } else {
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
  txSettings.eStopArmed = false;

  if ( transmitToReceiver(3,100)) {
    if (rf69_manager.sendtoWait((byte*)&txSettings, sizeof(txSettings), DEST_ADDRESS)) {
      uint8_t len = sizeof(remPackage);
      uint8_t from;
      if (rf69_manager.recvfromAckTimeout((uint8_t*)&remPackage, &len, 200, &from)) {
      } else {
        remPackage.type = 0;
        useDefaultKeyForTransmission = 0;
        initiateTransmitter();
        return false;
      }
    } else {
      remPackage.type = 0;
      useDefaultKeyForTransmission = 0;
      initiateTransmitter();
      return false;
    }
    useDefaultKeyForTransmission = 0;
    initiateTransmitter();
    return true;
  } else {
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

  txSettings.customEncryptionKey[15] = receiverID;

  initiateTransmitter(); // restart receiver with new key

}

// pair with new board
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
bool pairNewBoard() {

  txSettings.customEncryptionKey[15] = txSettings.boardID;

  useDefaultKeyForTransmission = 1;

  transmitKeyToReceiver();

  useDefaultKeyForTransmission = 0;

  rf69.setEncryptionKey(txSettings.customEncryptionKey);

  delay(50);

  if (transmitToReceiver(5,100)) {
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
      u8g2.setDisplayRotation(U8G2_R3);
      u8g2.begin();
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
        currentSetting--;
        settingsLoopFlag = true;
      } else if (throttlePosition == BOTTOM && currentSetting < (numOfSettings - 1)) {
        currentSetting++;
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
        if ( ! transmitSettingsToReceiver()) {
          loadFlashSettings();
          drawMessage("Failed", "No communication", 2000);
        } else {
          drawMessage("Complete", "Trigger mode changed", 2000);
        }
      } else if (currentSetting == MODE) {
        txSettings.controlMode = getSettingValue(currentSetting);
        if ( ! transmitSettingsToReceiver()) {
          loadFlashSettings();
          drawMessage("Failed", "No communication", 2000);
        } else {
          drawMessage("Complete", "Trigger mode changed", 2000);
        }
      } else if (currentSetting == TXPOWER) {
        txSettings.transmissionPower = getSettingValue(currentSetting);
        if ( ! transmitSettingsToReceiver()) {
          loadFlashSettings();
          drawMessage("Failed", "No communication", 2000);
        } else {
          updateFlashSettings();
          initiateTransmitter();
          drawMessage("Complete", "Power changed", 2000);
        }
      } else if (currentSetting == BOARDID) {
        selectBoard(getSettingValue(currentSetting));
        drawMessage("Complete", "New board selected!", 2000);
      }
      updateFlashSettings();
    }

    if (triggerFlag == false) {
      if (currentSetting == SETTINGS) {
        setDefaultFlashSettings();
        drawMessage("Complete", "Default settings loaded!", 2000);
      } else if (currentSetting == KEY) {
        for (uint8_t i = 0; i <= 15; i++) {
          txSettings.customEncryptionKey[i] = encryptionKey[i];
        }
        updateFlashSettings();
        loadFlashSettings();
        initiateTransmitter();

        drawMessage("Complete", "Default encryption Key!", 2000);
      } else if (currentSetting == PAIR) {
        Serial.println("Settings menu - PAIR");
        pairNewBoard();
      } else {
      changeThisSetting = !changeThisSetting;
      triggerFlag = true;
      }
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

  for ( uint8_t i = 0; i < numOfSettings; i++ )
  {  if (rules[i][0] == -1) {
    } else {
    setSettingValue( i, rules[i][0] );
    Serial.println(rules[i][0]);
    }
  }

  txSettings.firmVersion = VERSION;

  updateFlashSettings();
}

// load settings from flash
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void loadFlashSettings() {

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
    case 0:     value = txSettings.boardID;         break;
    case 1:     value = txSettings.triggerMode;     break;
    case 2:     value = txSettings.batteryType;     break;
    case 3:     value = txSettings.batteryCells;    break;
    case 4:     value = txSettings.motorPoles;      break;
    case 5:     value = txSettings.motorPulley;     break;
    case 6:     value = txSettings.wheelPulley;     break;
    case 7:     value = txSettings.wheelDiameter;   break;
    case 8:     value = txSettings.controlMode;     break;
    case 9:     value = txSettings.minHallValue;    break;
    case 10:    value = txSettings.centerHallValue; break;
    case 11:    value = txSettings.maxHallValue;    break;
    case 12:    value = txSettings.eStopMode;       break;
    case 13:    value = txSettings.breaklightMode;  break;
    case 14:    value = txSettings.throttleDeath;   break;
    case 15:    value = txSettings.drivingMode;     break;
    case 17:    value = txSettings.transmissionPower; break;

    default: /* Do nothing */ break;
  }
  return value;
}

// Set a value of a specific setting by index.
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setSettingValue(uint8_t index, uint64_t value) {
  switch (index) {
    case 0:         txSettings.boardID = value;         break;
    case 1:         txSettings.triggerMode = value;     break;
    case 2:         txSettings.batteryType = value;     break;
    case 3:         txSettings.batteryCells = value;    break;
    case 4:         txSettings.motorPoles = value;      break;
    case 5:         txSettings.motorPulley = value;     break;
    case 6:         txSettings.wheelPulley = value;     break;
    case 7:         txSettings.wheelDiameter = value;   break;
    case 8:         txSettings.controlMode = value;     break;
    case 9:         txSettings.minHallValue = value;    break;
    case 10:        txSettings.centerHallValue = value; break;
    case 11:        txSettings.maxHallValue = value;    break;
    case 12:        txSettings.eStopMode = value;       break;
    case 13:        txSettings.breaklightMode = value;  break;
    case 14:        txSettings.throttleDeath = value;   break;
    case 15:        txSettings.drivingMode = value;     break;
    case 17:        txSettings.transmissionPower = value; break;

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
    if ( changeSettings == true )
    {
      drawSettingsMenu();
    }
    else
    {
      if (displayView >=7) {
        displayView = 2;
        }
        switch (displayView) {
    case 2:
      drawPage();
      drawThrottle();
      break;
    case 3: case 4: case 5: case 6:
      drawPageOLD();
      drawThrottleOLD();
      drawBatteryLevel();
      drawBattery();
      //drawSignal();
      drawHeadlightStatus();
      if (returnData.eStopArmed) {
        drawEStopArmed();
      }
    case 0:
      drawAnnouncement();
      break;
      }
    }
u8g2.nextPage();
}

// Measure the hall sensor output and calculate throttle posistion
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void calculateThrottlePosition()
{
  // Hall sensor reading can be noisy, lets make an average reading.
  uint16_t total = 0;
  uint8_t samples = 10;
  uint16_t throttleMax = 512; // override from drivingMode to calculate max throttle

  if (txSettings.drivingMode == 0) { // slow mode
    throttleMax = 700;
  } else if (txSettings.drivingMode == 1){ // Intermidiate mode
    throttleMax = 850;
  } else if (txSettings.drivingMode == 2){ // pro mode
    throttleMax = 1023;
  }

  for ( uint8_t i = 0; i < samples; i++ )
  {
    total += analogRead(hallSensorPin);
  }

  hallValue = total / samples;

  if ( hallValue >= txSettings.centerHallValue )
  {
    throttle = constrain( map(hallValue, txSettings.centerHallValue, txSettings.maxHallValue, centerThrottle, throttleMax), centerThrottle, 1023 );
  } else {
    throttle = constrain( map(hallValue, txSettings.minHallValue, txSettings.centerHallValue, 0, centerThrottle), 0, centerThrottle );
  }

  // Remove hall center noise
  hallNoiseMargin = txSettings.throttleDeath;

  if ( abs(throttle - centerThrottle) < hallNoiseMargin )
  {
    throttle = centerThrottle;
  }
#ifdef DEBUG
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
  if ((((boardBattery > 0) && (boardBattery <= 15)) || (remoteBattery <= 15)) && returnData.eStopArmed) {
    alarmBlinkDuration = 500;
    alarmBlinkTimes = 6;
    alarmTriggered = true;
  }
}

// Vibrate
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void controlVib() {
  if (alarmTriggered){
    if (alarmBlinkCounter < alarmBlinkTimes) {
      if ((millis() - alarmBlinkTimer > alarmBlinkDuration)) {
        digitalWrite(vibrationActuatorPin, !digitalRead(vibrationActuatorPin));
        alarmBlinkTimer = millis();
        alarmBlinkCounter++;
      }
    } else {
      alarmTriggered = 0;
      alarmTriggered = false;
    }
  }

}

// Prints the settings menu on the OLED display
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void drawSettingsMenu() {
  // Local variables to store the setting value and unit
  uint64_t value;
  uint8_t shiftTextPixel;

  x = 5;
  y = 10;

  // Print setting title
  u8g2.setFont(u8g2_font_profont12_tr);
  if (currentSetting != 0){
    u8g2.drawTriangle(5, 0, 5, 10, 0, 5);
    }
  shiftTextPixel = 64 - (u8g2.getStrWidth(titles[ currentSetting ])/2);
  u8g2.drawStr(shiftTextPixel, y, titles[ currentSetting ] );
  if (currentSetting != 18){
    u8g2.drawTriangle(123, 0, 123, 10, 128, 5);
  }

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
    for (uint8_t i = 10; i < 16; i++) {
      tString += String(txSettings.customEncryptionKey[i]);
    }

  Serial.print("Custom encryptionKey with boardID: ");
  for (uint8_t i = 0; i < 16; i++) {
    Serial.print(txSettings.customEncryptionKey[i]);
  }
  Serial.println("");
  }

  if ( currentSetting == SETTINGS ) {
    tString = F("Reset");
  }

  if ( currentSetting == FIRMWARE ) {
    tString = String(txSettings.firmVersion);
  }

  if ( changeThisSetting == true ) {

    drawString(tString, tString.length(), x + 10, y + 30, u8g2_font_10x20_tr );

    // If setting has something to do with the hallValue
    if ( inRange(currentSetting, 9, 11) ) {
      tString = "(" + String(hallValue) + ")";
      drawString(tString, tString.length(), x + 50, y + 30, u8g2_font_profont12_tr );
    }
  } else {
    drawString(tString, tString.length(), x, y + 30, u8g2_font_10x20_tr );
  }
}

// Print the startup screen
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void drawStartScreen() {
  u8g2.firstPage();

  do {

    u8g2.drawXBMP( 1, 22, 60, 64, logo);

//    u8g2.setFont(u8g2_font_10x20_tr);
//    u8g2.drawStr(20, 26, "FeatherFly");
//    u8g2.setFont(u8g2_font_t0_12_tr);
//    u8g2.drawStr(30, 41, "mod by StefanMe");

  } while ( u8g2.nextPage() );

  delay(1000);
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
  } else {

    remPackage.headlight = 1;
  }

}

void longbuttonPress() {
  txSettings.eStopArmed = false;
  transmitSettingsToReceiver();
  u8g2.setDisplayRotation(U8G2_R0);
  changeSettings = true;
  drawTitle("Settings", 1500);
}

// draw main page
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void drawPageOLD() {

  uint8_t decimalsMain, decimalsSecond, decimalsThird;
  float valueMain, valueSecond, valueThird;
  int unitMain, unitSecond, unitThird;
  uint16_t first, last, firstSecond, lastSecond, firstThird, lastThird;

  x = 15;
  y = 10;

  switch (displayView) {
    case 3:
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
    case 4:
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
    case 5:
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
    case 6:
      valueMain = debugData.longestCycleTime;
      decimalsMain = 0;
      unitMain = 6;
      valueSecond = debugData.differenceJoinedSend;
      decimalsSecond = 1;
      unitSecond = 6;
      valueThird = debugData.cycleTime;
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
  drawString(tString, 10, x -2, y + 29, u8g2_font_logisoso26_tn );

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
  drawString(tString, 10, x, y +55, u8g2_font_logisoso18_tn );
  // Display second units
  u8g2.setFont(u8g2_font_profont12_tr);
  u8g2.drawStr( x + 25, y +55, dataSuffix[unitSecond]);


  // Convert valueThird to string
  firstThird = abs( floor(valueThird) );
  if ( firstThird <= 9 ) {
    tString = "0" + String(firstThird);
  } else {
    tString = firstThird;
  }
  // Display third numbers
  drawString(tString, 10, x, y +75, u8g2_font_logisoso18_tn );
  // Display main units
  u8g2.setFont(u8g2_font_profont12_tr);
  u8g2.drawStr( x + 25, y +75, dataSuffix[unitThird]);
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

// Draw page
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void drawAnnouncement() {

  x = 0;
  y = 37;

  drawString("E-Stop Activated!", 10, x, y, u8g2_font_logisoso18_tn);

  if (millis() - announcmentTimer > 2000){
    displayView = 6;
  }

}

// Draw page
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void drawPage() {

  uint8_t decimals;
  float value;
  uint16_t first, last;

  String s;

  uint8_t offset = 38;
  x = 0;
  y = 37;
  uint8_t width;

//  u8g2.drawFrame(0,0,64,128);

  // --- Speed ---
  value = ratioRpmSpeed * abs(returnData.rpm);
  float speedMax = 30.0;

  drawStringCenter(String(value,0), "km/h", y);

  y = 48;
  // speedometer graph height array
  uint8_t a[16] = {3, 3, 4, 4, 5, 6, 7, 8, 10,
    11, 13, 15, 17, 20, 24, 28};
  uint8_t h;

  for (uint8_t i = 0; i < 16; i++) {
    h = a[i];
    if (speedMax / 16 * i <= value) {
      u8g2.drawVLine(x + i*4 + 2, y - h, h);
    } else {
      u8g2.drawPixel(x + i*4 + 2, y - h);
      u8g2.drawPixel(x + i*4 + 2, y - 1);
    }
  }

  // --- Battery ---
  value = batteryPackPercentage( returnData.inpVoltage );

  y = 73;

  int battery = (int) value;
  drawStringCenter(String(battery), "%", y);

  drawString(String(returnData.inpVoltage, 1), 50, 73, u8g2_font_blipfest_07_tr);

  y = 78;
  x = 0;

  // longboard body
  h = 12;
  uint8_t w = 41;
  u8g2.drawHLine(x + 10, y, w); // top line
  u8g2.drawHLine(x + 10, y + h, w); // bottom

  // nose
  u8g2.drawHLine(x + 2, y + 3, 5); // top line
  u8g2.drawHLine(x + 2, y + h - 3, 5); // bottom

  u8g2.drawPixel(x + 1, y + 4);
  u8g2.drawVLine(x, y + 5, 3); // nose
  u8g2.drawPixel(x + 1, y + h - 4);

  u8g2.drawLine(x + 6, y + 3, x + 9, y);          // /
  u8g2.drawLine(x + 6, y + h - 3, x + 9, y + h);  // \

  // tail
  u8g2.drawHLine(64 - 6 - 2, y + 3, 5); // top line
  u8g2.drawHLine(64 - 6 - 2, y + h - 3, 5); // bottom

  u8g2.drawPixel(64 - 3, y + 4);
  u8g2.drawVLine(64 - 2, y + 5, 3); // tail
  u8g2.drawPixel(64 - 3, y + h - 4);

  u8g2.drawLine(64 - 6 - 3, y + 3, 64 - 6 - 6, y);          // /
  u8g2.drawLine(64 - 6 - 3, y + h - 3, 64 - 6 - 6, y + h);  // \

  // longboard wheels
  u8g2.drawBox(x + 3, y, 3, 2); // left
  u8g2.drawBox(x + 3, y + h - 1, 3, 2);
  u8g2.drawBox(64 - 7, y, 3, 2); // right
  u8g2.drawBox(64 - 7, y + h - 1, 3, 2);

  // battery sections
  for (uint8_t i = 0; i < 14; i++) {
    if (round((100 / 14) * i) <= value) {
      u8g2.drawBox(x + i*3 + 10, y + 2, 1, h - 3);
    }
  }

  // --- Distance in km ---
  value = ratioPulseDistance * returnData.tachometerAbs;
  String km;

  y = 118;

  if (value >= 1) {
    km = String(value, 0);
    drawStringCenter(String(km), "km", y);
  } else {
    km = String(value * 1000, 0);
    drawStringCenter(String(km), "m", y);
  }

  // max distance
  int range = 30;

  drawString(String(range), 56, 118, u8g2_font_blipfest_07_tr); // u8g2_font_prospero_bold_nbp_tn

  // dots
  y = 122;
  for (uint8_t i = 0; i < 16; i++) {
    u8g2.drawBox(x + i * 4, y + 4, 2, 2);
  }

  // start end
  u8g2.drawBox(0, y, 2, 6);
  u8g2.drawBox(62, y, 2, 6);
  u8g2.drawBox(31, y, 2, 6);
}

void drawStringCenter(String value, String caption, uint8_t y){

  static char cache[10];

  // draw digits
  int x = 0;
  value.toCharArray(cache, value.length() + 1);
  u8g2.setFont(u8g2_font_ncenB18_tn); //u8g2_font_t0_18b_tr);
  u8g2.drawStr(x, y, cache);

  // draw caption km/%
  x += u8g2.getStrWidth(cache) + 4;
  y -= 9;
  caption.toCharArray(cache, caption.length() + 1);
  u8g2.setFont(u8g2_font_crox1h_tf);
  u8g2.drawStr(x, y, cache);
}

void drawString(String string, int x, int y, const uint8_t *font){

  static char cache[20];
  string.toCharArray(cache, string.length() + 1);
  u8g2.setFont(font);

  if (x == -1) {
    x = (64 - u8g2.getStrWidth(cache)) / 2;
  }

  u8g2.drawStr(x, y, cache);
}

/*
   Print the throttle value as a bar on the OLED
*/
void drawThrottle() {

uint16_t width;

  if (throttle >= txSettings.centerHallValue) {
    width = map(throttle, txSettings.centerHallValue, txSettings.maxHallValue, 0, 14); //todo
    u8g2.drawBox(32 - width, 123, width, 5 );


  } else {
    width = map(throttle, txSettings.minHallValue, txSettings.centerHallValue, 14, 0); //todo
    u8g2.drawBox(32, 123, width, 5 );
  }

}


void drawThrottleOLD() {

  x = 0;
  y = 0;

  uint8_t width;

  // Draw throttle
  u8g2.drawVLine(x + 4, y , 128);
  u8g2.drawHLine(x, y, 10);
  u8g2.drawHLine(x, y +15, 2);
  u8g2.drawHLine(x, y +31, 4);
  u8g2.drawHLine(x, y +46, 2);
  u8g2.drawHLine(x, y +63, 4);
  u8g2.drawHLine(x, y +78, 2);
  u8g2.drawHLine(x, y +94, 4);
  u8g2.drawHLine(x, y +110, 2);
  u8g2.drawHLine(x, y +127, 10);


  if (throttle >= 512) {
    width = map(remPackage.throttle, 512, 1023, 0, 62);

    for (uint8_t i = 0; i < width; i++)
    {
      u8g2.drawHLine(x, y + 64 -i, 4);
    }
  } else {
    width = map(remPackage.throttle, 0, 511, 62, 0);

    for (uint8_t i = 0; i < width; i++)
    {
      u8g2.drawHLine(x, y + 64 + i, 4);
    }
  }
}

void drawBattery() {

  x = 6;
  y = 0;

  u8g2.drawVLine(x + 4, y , 128);

}

void drawSignal() {

  x = 26;
  y = 112;

  if (connectionLost) {

    if (millis() - lastSignalBlink > 500) {
      signalBlink = !signalBlink;
      lastSignalBlink = millis();
    }

    if (signalBlink == true) {
      u8g2.drawXBMP(x, y, 12, 12, connectedIcon);
    } else {
      u8g2.drawXBMP(x, y, 12, 12, noconnectionIcon);
    }

    } else {

      u8g2.drawXBMP(x, y, 12, 12, connectedIcon);
    }

}

/*
   Print the remotes battery level as a battery on the OLED
*/
void drawBatteryLevel() {

  x = 43;
  y = 115;

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

  x = 34;
  y = 117;

  u8g2.drawDisc(x , y , 5, U8G2_DRAW_LOWER_RIGHT);
  u8g2.drawDisc(x , y , 5, U8G2_DRAW_LOWER_LEFT);
  u8g2.drawLine(x - 1 , y - 3, x - 1, y + 3);

  if (remPackage.headlight == 1) {
    u8g2.drawLine(x     , y - 3, x    , y - 5);
    u8g2.drawLine(x + 3 , y - 3, x + 4, y - 5);
    u8g2.drawLine(x - 3 , y - 3, x - 4, y - 5);
  }
}

String uint64ToString(uint64_t number) {
  unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
  unsigned long part2 = (unsigned long)((number));

  if (part1 == 0) {
    return String(part2, DEC);
  }
  return String(part1, DEC) + String(part2, DEC);
}

// Draw eStop
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void drawEStopArmed(){

  x = 14;
  y = 112;

  u8g2.drawXBMP(x, y, 12, 15, eStopArmed);

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
