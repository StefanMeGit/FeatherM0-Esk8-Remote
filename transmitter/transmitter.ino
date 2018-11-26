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

const unsigned char policeMode[] PROGMEM = {
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
};

// Transmit and receive package
struct debug {
  unsigned long cycleTime = 0;
  unsigned long transmissionTime = 0;
  int8_t rssi;
  unsigned long counterJoined = 0;
  unsigned long counterSend = 0;
  unsigned long counterReceived = 0;
  unsigned long differenceJoinedSend = 0;
  unsigned long differenceJoinedReceived = 0;
  unsigned long longestCycleTime = 0;
  unsigned long lastTransmissionAvaible = 0;
  unsigned short transmissionTimeStart = 0;
  unsigned short transmissionTimeFinish = 0;
  unsigned short transmissionTimeDuration = 0;
  unsigned long cycleTimeStart = 0;
  unsigned long cycleTimeFinish = 0;
  unsigned long cycleTimeDuration = 0;
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
  short Frequency;                  // 21
  uint8_t standbyMode;              // 22
  uint8_t metricImperial;           // 23
  uint8_t policeMode;               // 24
} TxSettings;

TxSettings txSettings;

// Defining flash storage
FlashStorage(flash_TxSettings, TxSettings);

uint8_t currentSetting = 0;
const uint8_t numOfSettings = 27;

struct menuItems{
  uint8_t ID;
  short standart;
  short minimum;
  short maximum;
  char name[19];
  uint8_t unitIdentifier;
  uint8_t valueIdentifier;
} menuItems[] = {

  {0,   1,    0,    9,    "Board ID",       4 , 0},          //0 boardID
  {16,  -1,   0,    0,    "Pair new Board", 0 , 0},        //16 pair new board
  {15,  2,    0,    2,    "Driving Mode",   0 , 6},         //15 Driving Mode
  {12,  0,    0,    2,    "Estop Mode",     0 , 4},         //12 EStop mode |0soft|1hard|2off
  {1,   1,    0,    1,    "Trigger use",    0 , 1},          //1 0: Killswitch
  {2,   1,    0,    1,    "Battery type",   0 , 2},          //2 0: Li-ion      | 1: LiPo
  {3,   10,   6,    12,   "Battery cells",  1 , 0},        //3 Cell count
  {4,   14,   0,    250,  "Motor poles",    0 , 0},       //4 Motor poles
  {5,   14,   0,    250,  "Motor pulley",   2 , 0},       //5 Motor pully
  {6,   38,   0,    250,  "Wheel pulley",   2 , 0},       //6 Wheel pulley
  {7,   80,   0,    250,  "Wheel diameter", 3 , 0},       //7 Wheel diameter
  {8,   1,    0,    2,    "Control mode",   0 , 3},          //8 0: PPM only   | 1: PPM and UART | 2: UART only
  {9,   275,  0,    400,  "Throttle min",   0 , 0},      //9 Min hall value
  {10,  510,  400,  600,  "Throttle center", 0 , 0},    //10 Center hall value
  {11,  794,  600,  1023, "Throttle max",   0 , 0},   //11 Max hall value
  {13,  0,    0,    2,    "Breaklight Mode", 0 , 5},         //13 breaklight mode |0off|1alwaysOn|onWithheadlight
  {14,  10,   0,    30,   "Deathband",  0 , 0},       //14 throttle death center
  {15,  2,    0,    2,    "Driving Mode",   0 , 6},         //15 Driving Mode
  {25,  0,    0,    1,    "Unit selection", 0 , 8},         //22 Metric/Imperial
  {17,  20,   14,   20,   "Transmission Power", 5 , 0},       //17 transmission power
  {18,  -1,   0,    0,    "Encyption key",  0 , 0},        //18 show Key
  {19,  433,  424,  442,  "Frequency",      6 , 0},            //19 Frequency
  {24,  1,    0,    2,    "Standby mode", 0 , 7},         //24 Standby Mode
  {26,  0,    0,    2,    "Police mode",     0 , 9},         //26 Police mode
  {20,  -1,   0,    0,    "Firmware Version", 0 , 0},       //19 Firmware
  {21,  -1,   0,    0,    "Set default key", 0 , 0},        //20 Set default key
  {22,  -1,   0,    0,    "Settings",       0 , 0},        //21 Settings
  {23,  -1,   0,    0,    "Exit",           0 , 0}         //22 Exit
};

// Defining constants to hold the special settings, so it's easy changed though the code
#define BOARDID     0
#define TRIGGER     1
#define MODE        8
#define ESTOP       12
#define BREAKLIGHT  13
#define DRIVINGMODE 15
#define PAIR        16
#define TXPOWER     17
#define KEY         18
#define FREQUENCY   19
#define FIRMWARE    20
#define DEFAULTKEY  21
#define SETTINGS    22
#define EXIT        23

const char stringValues[9][3][15] = {
  {"Killswitch", "Cruise", ""},
  {"Li-ion", "LiPo", ""},
  {"PPM", "PPM and UART", "UART only"},
  {"soft", "hard", "off"},
  {"off", "Always on", "with headlight"},
  {"Beginner", "Intermidiate", "Pro"},
  {"off", "10 minutes", "30 minutes"},
  {"Metric", "Imperial", ""},
  {"off", "startup", "activation"}
};

const char settingUnits[6][4] = {"S", "T", "mm", "#", "dBm", "Mhz"};

const char dataSuffix[9][4] = {"V", "KMH", "km", "A","ms","dBm", "", "MPH", "mi."};
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
const unsigned long buttonSampleIntervalsMs = 100;
uint8_t longbuttonPressCountMax = 12;    // 80 * 25 = 2000 ms
uint8_t mediumbuttonPressCountMin = 5;    // 20 * 25 = 500 ms
uint8_t buttonPressCount = 0;
byte prevButtonState = HIGH;         // button is active low

// Transmit and receive package
struct package {      // | Normal   | Setting   | Dummy
  uint8_t type;       // | 0        | 1         | 2
  uint16_t throttle;  // | Throttle |           |
  uint8_t trigger;    // | Trigger  |           |
  uint8_t headlight;  // | ON       | OFF       |
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
#define RF69_FREQ   433
#define DEST_ADDRESS   1
#define MY_ADDRESS     2

RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

uint8_t encryptionKey[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x01};

uint8_t useDefaultKeyForTransmission = 0;

// Dont put this on the stack:
uint8_t transmissionFailCounter = 0;
bool connectionLost = false;


// Battery monitering
const float minVoltage = 3.1;
const float maxVoltage = 4.2;
const float refVoltage = 3.3;
unsigned long overchargeTimer = 0;
unsigned long underVoltageTimer = 0;

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
uint8_t displayView = 0;
uint8_t x, y;

// Defiing varibales for signal
unsigned long lastSignalBlink;
bool signalBlink = false;

// Defining variables for vibration actuator
unsigned long vibIntervalDuration = 0;
uint8_t vibIntervalCounterTarget = 0;
uint8_t vibIntervalCounter = 0;
uint8_t vibIntervalTimer = 0;

// Defining variables for Settings menu
bool changeSettings     = false; // Global flag for whether or not one is editing the settings
bool changeThisSetting  = false;
bool settingsLoopFlag   = false;
bool triggerFlag = false;
bool settingScrollFlag  = false;
bool settingsChangeValueFlag = false;
unsigned short settingScrollWait = 800;
unsigned long settingChangeMillis = 0;

//announcment
unsigned long announcementTimer = 0;
long announcementDuration = 0;
String announcementStringLine1 = "";
String announcementStringLine2 = "";
bool activateAnnouncement = false;
bool announcementFade = false;
bool blockAnnouncement = false;

bool policeModeActive = false;

short throttleMax = 512;

uint8_t boardBatteryWarningLevel = 0;
uint8_t remoteBatteryWanringLevel = 0;

bool eStopAnnounced = false;

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

  drawStartScreen();

  loadFlashSettings();

  checkEncryptionKey();

  initiateTransmitter();

  if (txSettings.policeMode >= 1 ){
    policeModeActive = true;
  }

  calculateThrottlePosition();

  if ((txSettings.policeMode == 1) && (throttlePosition == BOTTOM) && triggerActive() ){
    policeModeActive = false;
  } else {
    if (txSettings.policeMode == 1) {
      policeModeActive = true;
    }
  }

  if (extraButtonActive()) {
      txSettings.eStopArmed = false;
      transmitSettingsToReceiver();
      changeSettings = true;
      u8g2.setDisplayRotation(U8G2_R0);
      drawTitle("Settings", 1500);
  }

  updateLastTransmissionTimer();

}

// loop
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void loop() {

  debugData.cycleTimeStart = millis();
  calculateThrottlePosition();

  if (changeSettings == true) {
    controlSettings();
    updateMainDisplay();
  } else {
    remPackage.type = 0;
    remPackage.trigger = triggerActive();
    remPackage.throttle = throttle;
    if (transmitToReceiver(1,50)) {
      updateMainDisplay();
    }

    if (connectionLost) {
      updateMainDisplay();
      }

    debugData.differenceJoinedSend = debugData.counterJoined - debugData.counterSend;
    debugData.differenceJoinedReceived = debugData.counterJoined - debugData.counterReceived;
  }

  debugData.cycleTimeFinish = millis();
  debugData.cycleTime = debugData.cycleTimeFinish - debugData.cycleTimeStart;
  if (debugData.cycleTime > debugData.longestCycleTime) {
      debugData.longestCycleTime = debugData.cycleTime;
    }

  detectButtonPress();
  checkConnection();
  checkBatteryLevel();
  controlVib();

}

// Sleep mode handling
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------

void ISR (){}

void sleep() {

  txSettings.eStopArmed = false; // dissable eStop on connected Board
  transmitSettingsToReceiver(); // transmitt Settings

  //sleeping
  digitalWrite(13, LOW); //swtich off LED
  rf69.sleep(); // switch off radio
  drawMessage("See You!", "Switching off...", 2000);
  updateMainDisplay();
  u8g2.setPowerSave(1); // set OLED into sleep
  delay(100);
  attachInterrupt(6, ISR, LOW);
  delay(100);
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __WFI();

  //wake up
  u8g2.setPowerSave(0);
  drawMessage("Lets Ride!", "Switching on...", 2000);
  updateMainDisplay();
  detachInterrupt(6);
  u8g2.setDisplayRotation(U8G2_R3);
  updateLastTransmissionTimer();

}

// check connection
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
 void checkConnection() {

   if (returnData.eStopArmed && !eStopAnnounced) {
     setAnnouncement("EStop Armed!", "Have a safe ride!", 1000, true);
     eStopAnnounced = true;
   }

    if (millis() - debugData.lastTransmissionAvaible > 350) {

      if (!connectionLost && returnData.eStopArmed) {
        String lastTranmissionDurationStr = "Time: ";
        lastTranmissionDurationStr += String(millis() - debugData.lastTransmissionAvaible);
        setAnnouncement("E-Stop!!!", lastTranmissionDurationStr, 10000, false);
      }
      returnData.eStopArmed = false;

      connectionLost = true;
      eStopAnnounced = false;
    } else {
      connectionLost = false;
    }

    if (txSettings.standbyMode == 1){

      if (millis() - debugData.lastTransmissionAvaible > 600000) {
        sleep();
      }

    } else if (txSettings.standbyMode == 2) {

        if (millis() - debugData.lastTransmissionAvaible > 1800000) {
        sleep();

      }
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

  rf69.setTxPower(20);

  if (useDefaultKeyForTransmission == 1) {
    Serial.println("useDefaultKeyForTransmission == 1");
    rf69.setFrequency(RF69_FREQ);
    rf69.setEncryptionKey(encryptionKey);
    Serial.println(RF69_FREQ);
    for (uint8_t i = 0; i <=15; i++){
      Serial.print(encryptionKey[i]);
    }
    Serial.println("");

  } else if (useDefaultKeyForTransmission == 2){
    Serial.println("useDefaultKeyForTransmission == 2");
    rf69.setEncryptionKey(txSettings.customEncryptionKey);
    rf69.setFrequency(txSettings.Frequency);
    Serial.println(txSettings.Frequency);
    for (uint8_t i = 0; i <=15; i++){
      Serial.print(encryptionKey[i]);
    }
    Serial.println("");

  } else if (useDefaultKeyForTransmission == 0){
    Serial.println("useDefaultKeyForTransmission == 0");
    rf69.setFrequency(txSettings.Frequency);
    rf69.setEncryptionKey(txSettings.customEncryptionKey);
    Serial.println(txSettings.Frequency);
    for (uint8_t i = 0; i <=15; i++){
      Serial.print(txSettings.customEncryptionKey[i]);
    }
    Serial.println("");
  }

  delay(20);
}

// check encryptionKey
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void updateLastTransmissionTimer (){

  debugData.lastTransmissionAvaible = millis();

}

// check encryptionKey
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void checkEncryptionKey() {

  for (uint8_t i = 0; i < 16; i++) {
    Serial.print("Stored encryptionKey: "); Serial.println(txSettings.customEncryptionKey[i]);

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

  txSettings.Frequency = RF69_FREQ;//random(424, 442);
  Serial.print(txSettings.Frequency);

  updateFlashSettings();

}

//Function used to transmit the remPackage and receive auto acknowledgment
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
bool transmitToReceiver(uint8_t retries, uint8_t timeout) {
  debugData.transmissionTimeStart = millis();

  rf69_manager.setRetries(retries);
  rf69_manager.setTimeout(timeout);

  debugData.counterJoined++;

  uint8_t len = sizeof(remPackage);

  if (rf69_manager.sendtoWait((uint8_t*)&remPackage, len, DEST_ADDRESS)) {
    updateLastTransmissionTimer();
    uint8_t len = sizeof(returnData);
    uint8_t from;

    debugData.counterSend++;

    if (rf69_manager.recvfromAckTimeout((uint8_t*)&returnData, &len, 20, &from)) { // TEST!
      updateLastTransmissionTimer();
      debugData.counterReceived++;
      debugData.transmissionTimeFinish = millis();
      debugData.transmissionTime = debugData.transmissionTimeFinish - debugData.transmissionTimeStart;
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

  rf69_manager.setRetries(1);
  rf69_manager.setTimeout(200);

  if ( transmitToReceiver(1,200)) {
    if (rf69_manager.sendtoWait((byte*)&txSettings, sizeof(txSettings), DEST_ADDRESS)) {
      uint8_t len = sizeof(returnData);
      uint8_t from;
      if (rf69_manager.recvfromAckTimeout((uint8_t*)&returnData, &len, 200, &from)) {
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

  rf69_manager.setRetries(1);
  rf69_manager.setTimeout(200);

  if ( transmitToReceiver(1,200)) {

    if (rf69_manager.sendtoWait((byte*)&txSettings, sizeof(txSettings), DEST_ADDRESS)) {
      uint8_t len = sizeof(returnData);
      uint8_t from;
      Serial.println("Told Receiver next package are Settings");
      if (rf69_manager.recvfromAckTimeout((uint8_t*)&returnData, &len, 200, &from)) {
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

//Function used to transmit settings to receiver
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
bool transmitFreqToReceiver() {

  useDefaultKeyForTransmission = 2;

  initiateTransmitter();

  remPackage.type = 1;
  txSettings.eStopArmed = false;

  if ( transmitToReceiver(1,200)) {
    Serial.println("FirstTrans ok");
    if (rf69_manager.sendtoWait((byte*)&txSettings, sizeof(txSettings), DEST_ADDRESS)) {
      Serial.println("SettingsSend ok");
      uint8_t len = sizeof(remPackage);
      uint8_t from;
      Serial.println("Told Receiver next package are Settings");
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

  transmitKeyToReceiver();

  rf69.setEncryptionKey(txSettings.customEncryptionKey);
  rf69.setFrequency(txSettings.Frequency);

  initiateTransmitter();

  if (transmitToReceiver(1,200)) {
    drawMessage("Complete", "New board paired!", 1000);
  } else {
    drawMessage("Fail", "Board not paired", 2000);
  }

  Serial.print("Exit pairNewBoard()");
}

// Uses the throttle and trigger to navigate and change settings
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void controlSettings() {


  if (changeThisSetting == true) {

    if (menuItems[currentSetting].ID == EXIT) {
      u8g2.setDisplayRotation(U8G2_R3);
      u8g2.begin();
      changeSettings = false;
    }

    if (settingsLoopFlag == false && menuItems[currentSetting].standart != -1) {
      short value = getSettingValue(menuItems[currentSetting].ID);
      if ( throttlePosition == TOP ) {
        value++;
      } else if ( throttlePosition == BOTTOM ) {
        value--;
      }

      if (inRange(value, menuItems[currentSetting].minimum, menuItems[currentSetting].maximum)) {
        setSettingValue(menuItems[currentSetting].ID, value);
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
      } else if (throttlePosition == BOTTOM && currentSetting < (numOfSettings)) {
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
      if (menuItems[currentSetting].ID == TRIGGER) {
        txSettings.triggerMode = getSettingValue(menuItems[currentSetting].ID);
        if ( ! transmitSettingsToReceiver()) {
          loadFlashSettings();
          drawMessage("Failed", "No communication", 2000);
        } else {
          drawMessage("Complete", "Trigger mode changed", 2000);
        }
      } else if (menuItems[currentSetting].ID == MODE) {
        txSettings.controlMode = getSettingValue(menuItems[currentSetting].ID);
        if ( ! transmitSettingsToReceiver()) {
          loadFlashSettings();
          drawMessage("Failed", "No communication", 2000);
        } else {
          updateFlashSettings();
          drawMessage("Complete", "Trigger mode changed", 2000);
        }
      } else if (menuItems[currentSetting].ID == TXPOWER) {
        txSettings.transmissionPower = getSettingValue(menuItems[currentSetting].ID);
        if ( ! transmitSettingsToReceiver()) {
          loadFlashSettings();
          drawMessage("Failed", "No communication", 2000);
        } else {
          updateFlashSettings();
          initiateTransmitter();
          drawMessage("Complete", "Power changed", 2000);
        }
      } else if (menuItems[currentSetting].ID == BOARDID) {
        selectBoard(getSettingValue(menuItems[currentSetting].ID));
        drawMessage("Complete", "New board selected!", 2000);
      } else if (menuItems[currentSetting].ID == FREQUENCY) {
        if ( ! transmitFreqToReceiver()) {
          loadFlashSettings();
          drawMessage("Failed", "No communication", 2000);
        } else {
          updateFlashSettings();
          initiateTransmitter();
          drawMessage("Complete", "Frequency changed", 2000);
        }
      } else if (menuItems[currentSetting].ID == BREAKLIGHT) {
        if ( ! transmitSettingsToReceiver()) {
          loadFlashSettings();
          drawMessage("Failed", "No communication", 2000);
        } else {
          updateFlashSettings();
          drawMessage("Complete", "Changed!", 2000);
        }
      } else if (menuItems[currentSetting].ID == ESTOP) {
        if ( ! transmitSettingsToReceiver()) {
          loadFlashSettings();
          drawMessage("Failed", "No communication", 2000);
        } else {
          updateFlashSettings();
          drawMessage("Complete", "Changed!", 2000);
        }
      }
      updateFlashSettings();
    }

    if (triggerFlag == false) {
      if (menuItems[currentSetting].ID == SETTINGS) {
        setDefaultFlashSettings();
        drawMessage("Complete", "Default settings loaded!", 2000);
      } else if (menuItems[currentSetting].ID == KEY) {
        for (uint8_t i = 0; i <= 15; i++) {
          txSettings.customEncryptionKey[i] = encryptionKey[i];
        }
        updateFlashSettings();
        loadFlashSettings();
        initiateTransmitter();

        drawMessage("Complete", "Default encryption Key!", 2000);
      } else if (menuItems[currentSetting].ID == PAIR) {
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
  {  if (menuItems[i].standart == -1) {
    } else {
    setSettingValue( menuItems[i].ID, menuItems[i].standart );
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
    case 17:    value = txSettings.transmissionPower;break;
    case 19:    value = txSettings.Frequency;       break;
    case 20:    value = txSettings.firmVersion;     break;
    case 24:    value = txSettings.standbyMode;     break;
    case 25:    value = txSettings.metricImperial;  break;
    case 26:    value = txSettings.policeMode;      break;


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
    case 19:        txSettings.Frequency = value;       break;
    case 20:        txSettings.firmVersion = value;     break;
    case 24:        txSettings.standbyMode = value;     break;
    case 25:        txSettings.metricImperial = value;  break;
    case 26:        txSettings.policeMode = value;      break;

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
  if (digitalRead(triggerPin) == LOW) {
    return true;
  } else {
    return false;
  }
}

// Return true if extra Button is activated, false otherwise
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
bool extraButtonActive() {
  if (digitalRead(extraButtonPin) == LOW)
    return true;
  else
    return false;
}

// setAnnouncement with vibration
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setAnnouncement(String stringLine1, String stringLine2, short duration, bool fade) {

  if (!activateAnnouncement) {

    announcementStringLine1 = stringLine1;
    announcementStringLine2 = stringLine2;
    announcementDuration = duration;
    announcementFade = fade;

    vibIntervalDuration = 500; // interval duration for normal alarm
    vibIntervalCounterTarget = abs(duration / vibIntervalDuration);

    activateAnnouncement = true;
    announcementTimer = millis();

  }

}

// Draw announcment
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void drawAnnouncement(){

  if ((millis() - announcementTimer < announcementDuration) || !announcementFade) {

    x = 23;
    y = 0;

    //u8g2.drawBox(16,0,50,110);

    u8g2.setFontDirection(1);
    u8g2.setFontMode(1);
    u8g2.setDrawColor(1);
    drawString(announcementStringLine1, announcementStringLine1.length(), x + 23 , y , u8g2_font_10x20_tr ); //u8g2_font_7x14B_tr smaller alternative
    drawString(announcementStringLine2, announcementStringLine2.length(), x , y , u8g2_font_7x14_tr ); //u8g2_font_7x14B_tr smaller alternative
    u8g2.setFontDirection(0);
    u8g2.setDrawColor(1);

    if(triggerActive() && throttlePosition == MIDDLE){ // reset message when fade is off by trigger and throttle in middle pos
      activateAnnouncement = false;
    }
  } else {
    if (announcementFade) {
      activateAnnouncement = false;
    }
  }
}

// vibrate corresponding to announcment
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void controlVib() {
  if (activateAnnouncement){
    if (vibIntervalCounter < vibIntervalCounterTarget) {
      if ((millis() - vibIntervalTimer > vibIntervalDuration)) {
        digitalWrite(vibrationActuatorPin, !digitalRead(vibrationActuatorPin));
        vibIntervalTimer = millis();
        vibIntervalCounter++;
      }
    }
  } else {
    vibIntervalCounter = 0;
    digitalWrite(vibrationActuatorPin, false);
  }

}

// Update the OLED for each loop
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void updateMainDisplay() {

u8g2.clearBuffer();
    if ( changeSettings == true ) {
      drawSettingsMenu();
    } else {
      if (displayView >= 4) {
        displayView = 0;
      }
      if (activateAnnouncement){
        drawAnnouncement();
      } else {
        drawPage();
      }
      drawThrottle();
      drawBatteryRemote();
      drawBatteryBoard();
      drawHeadlightStatus();
      if (connectionLost){
        drawSignal();
      } else if ((txSettings.policeMode >= 1) && (policeModeActive == true)){
        drawPoliceMode();
      } else if (returnData.eStopArmed) {
        drawEStopArmed();
      }
    }
u8g2.sendBuffer();
}

// Measure the hall sensor output and calculate throttle posistion
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void calculateThrottlePosition()
{
  // Hall sensor reading can be noisy, lets make an average reading.
  uint16_t total = 0;
  uint8_t samples = 10;

  if ((txSettings.policeMode > 0) && policeModeActive){
    throttleMax = 600;
  } else if (txSettings.drivingMode == 0){ // slow mode
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
  uint8_t samples = 20;

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
  String device;

  boardBattery = batteryPackPercentage( returnData.inpVoltage );
  boardBatteryAbs = abs( floor(boardBattery) );
  remoteBattery = batteryLevel();
  if ((((boardBattery > 0) && (boardBattery <= 20)) || (remoteBattery <= 15)) && returnData.eStopArmed) {
    if (millis() - underVoltageTimer >= 1000) {
      if (boardBattery <= 10 && boardBatteryWarningLevel <= 1) {
        device = "Board: ";
        device += String(boardBatteryAbs);
        device += "%";
        setAnnouncement("Low Battery!", device, 10000, true);
        boardBatteryWarningLevel = 2;
      } else if (boardBattery <= 20 && boardBatteryWarningLevel <= 0) {
        device = "Board: ";
        device += String(boardBatteryAbs);
        device += "%";
        setAnnouncement("Low Battery!!!", device, 5000, true);
        boardBatteryWarningLevel = 1;
      } else {
        device = "Remote: ";
        device += String(remoteBattery);
        device += "%";
        setAnnouncement("Low Battery!", device, 5000, true);
        remoteBatteryWanringLevel = 1;
      }
    }
  } else if (returnData.inpVoltage >= 42.0 && throttlePosition == BOTTOM){
      if ((millis() - overchargeTimer) >= 5000) {
        setAnnouncement("Overcharge!", "Caution!", 3000, true);
      }
  } else {
    overchargeTimer = millis();
    underVoltageTimer = millis();
  }

  if (boardBattery >= 25 ) {
    boardBatteryWarningLevel = 0;
  } else if (boardBattery >= 15 ) {
    boardBatteryWarningLevel = 1;
  }

  if (remoteBattery >= 15 ) {
    remoteBatteryWanringLevel = 0;
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
  //shiftTextPixel = 64 - (u8g2.getStrWidth(menuItems[ currentSetting ].name)/2);
  shiftTextPixel = 64 - (u8g2.getStrWidth(menuItems[ currentSetting ].name)/2);
  u8g2.drawStr(shiftTextPixel, y, menuItems[ currentSetting ].name);
  if (currentSetting != 22){
    u8g2.drawTriangle(123, 0, 123, 10, 128, 5);
  }

  // Get current setting value
  switch (menuItems[ currentSetting ].ID) {
    default:
      value = getSettingValue(menuItems[ currentSetting ].ID);
      break;

  }

  // Check if there is a text string for the setting value
  if ( menuItems[currentSetting].valueIdentifier != 0 )
  {
    uint8_t index = menuItems[currentSetting].valueIdentifier - 1;
    tString = stringValues[ index ][ value ];
  } else { // else normal value from txSettings
    tString = uint64ToString(value);
  }


  if ( menuItems[currentSetting].unitIdentifier != 0 ) {
    tString += settingUnits[ menuItems[currentSetting].unitIdentifier - 1 ];
  }

  if ( menuItems[ currentSetting ].ID == EXIT ) {
    tString = F("Exit");
  }

  if ( menuItems[ currentSetting ].ID == PAIR ) {
    tString = F("Pair now");
  }

  if ( menuItems[ currentSetting ].ID == DEFAULTKEY ) {
    tString = F("Restore");
  }

  if ( menuItems[ currentSetting ].ID == KEY ) {
    for (uint8_t i = 8; i < 16; i++) {
      tString += String(txSettings.customEncryptionKey[i]);
    }
  }

  if ( menuItems[ currentSetting ].ID == SETTINGS ) {
    tString = F("Reset");
  }

  if ( changeThisSetting == true ) {

    drawString(tString, tString.length(), x + 10, y + 30, u8g2_font_10x20_tr );

    // If setting has something to do with the hallValue
    if ( inRange(menuItems[ currentSetting ].ID, 9, 11) ) {
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
  u8g2.setDisplayRotation(U8G2_R0);
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
  if (changeSettings) {
    changeSettings = false;
    u8g2.setDisplayRotation(U8G2_R3);
  } else {
    displayView++;
  }
}

// called when button is kept pressed for more than 2 seconds
void mediumbuttonPress() {

  if ((throttlePosition == BOTTOM) && triggerActive()){

    if (!policeModeActive){
      setAnnouncement("Hmm :(", "Cold day today...", 1000, true);
      policeModeActive = true;
      txSettings.policeMode = 1;
      updateFlashSettings();
    } else {
      setAnnouncement("Yeahh!", "Go baby, go!", 1000, true);
      policeModeActive = false;
    }

  } else if ( returnData.headlightActive == 1 ) {

    remPackage.headlight = 0;

  } else {

    remPackage.headlight = 1;

  }

}

void longbuttonPress() {

  if ((throttlePosition == BOTTOM) && !triggerActive()){
    txSettings.eStopArmed = false;
    transmitSettingsToReceiver();
    u8g2.setDisplayRotation(U8G2_R0);
    changeSettings = true;
    drawTitle("Settings", 1500);
  } else {
    sleep();
  }

}

// draw main page
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void drawPage() {

  uint8_t decimalsMain, decimalsSecond, decimalsThird;
  float valueMain, valueSecond, valueThird, speedValue, distanceValue;
  int unitMain, unitSecond, unitThird;
  uint16_t first, last, firstSecond, lastSecond, firstThird, lastThird, speedValueUnit, distanceValueUnit;

  x = 15;
  y = 10;

  if (txSettings.metricImperial == 1){
    speedValue = (ratioRpmSpeed * returnData.rpm) * 0.621371;
    speedValueUnit = 7;
    distanceValue = (ratioPulseDistance * returnData.tachometerAbs) * 0.621371;
    distanceValueUnit = 8;
  } else {
    speedValue = ratioRpmSpeed * returnData.rpm;
    speedValueUnit = 1;
    distanceValue = ratioPulseDistance * returnData.tachometerAbs;
    distanceValueUnit = 8;
  }

  switch (displayView) {
    case 0:
      valueMain = speedValue;
      decimalsMain = 1;
      unitMain = speedValueUnit;
      valueSecond = returnData.inpVoltage;
      decimalsSecond = 2;
      unitSecond = 0;
      valueThird = distanceValue;
      decimalsThird = 2;
      unitThird = distanceValueUnit;
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

// Draw stringCenter
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
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

void drawThrottle() {

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
    width = map(remPackage.throttle, 512, throttleMax, 0, 62);

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

void drawBatteryBoard() {

  x = 6;
  y = 0;

  uint8_t height;
  uint8_t boardBattery;
  uint8_t boardBatteryAbs;
  uint8_t shift;

  boardBattery = abs( floor( batteryPackPercentage(returnData.inpVoltage)));
  boardBatteryAbs = abs( floor(boardBattery) );

  u8g2.drawVLine(x + 4, y , 128);

  height = map(boardBatteryAbs, 0, 100, 0, 128);

  shift = 128 - height;

  u8g2.drawBox(x, y + shift, 5, height);


}

void drawSignal() {

  x = 14;
  y = 113;

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
void drawBatteryRemote() {

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
  //u8g2.drawLine(x - 1 , y - 3, x - 1, y + 3);

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

// Draw police mode
//---------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------
void drawPoliceMode(){

  x = 15;
  y = 114;

  u8g2.drawXBMP(x, y, 12, 12, policeMode);

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
