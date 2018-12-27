// FeatherFly Receiver - eSk8 Remote

#include <SPI.h>
#include <Servo.h>
#include <FlashStorage.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <VescUart.h>

//#define DEBUG

#define VERSION 1.0

struct debug {
  unsigned long lastTransmissionAvaible = 0;
} debugData;

// Transmit and receive package
struct package {        // | Normal   | Setting   | Confirm
  uint8_t type = 0;       // | 0      | 1     | 2
  uint16_t throttle = 0;    // | Throttle   | ---   | ---
  uint8_t trigger = 0;      // | Trigger  | ---     | ---
  uint8_t headlight = 0;
} remPackage;

// Transmit and receive package
struct packageBackup {        // | Normal   | Setting   | Confirm
  uint8_t type = 0;       // | 0      | 1     | 2
  uint16_t throttle = 0;    // | Throttle   | ---   | ---
  uint8_t trigger = 0;      // | Trigger  | ---     | ---
  uint8_t headlight = 0;
} remPackageBackup;

// Transmit and receive package
struct packageEStop {
  bool armed = false;
  bool triggered = false;
  bool fullBreakDone = false;
} dataEStop;

#define NORMAL 0
#define SETTING 1
#define CONFIRM 2

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
  bool eStopArmed = false;          // 20
  short Frequency;                  // 21
  uint8_t standbyMode;              // 22
  uint8_t metricImperial;           // 23
  uint8_t policeMode;               // 24
  uint8_t homeScreen;                 // 25
} RxSettings;

RxSettings rxSettings;

//Defining flash storage
FlashStorage(flash_RxSettings, RxSettings);

const uint8_t numOfSettings = 25;

// Setting rules format: default, min, max.
const short settingRules[numOfSettings][3] {
  {1, 0, 9},          //0 boardID
  {1, 0, 1},          //1 0: Killswitch  | 1: Cruise control
  {1, 0, 1},          //2 0: Li-ion      | 1: LiPo
  {10, 6, 12},        //3 Cell count
  {14, 0, 250},       //4 Motor poles
  {14, 0, 250},       //5 Motor pully
  {38, 0, 250},       //6 Wheel pulley
  {80, 0, 250},       //7 Wheel diameter
  {1, 0, 2},          //8 0: PPM only   | 1: PPM and UART | 2: UART only
  {300, 0, 400},      //9 Min hall value
  {520, 300, 700},    //10 Center hall value
  {730, 600, 1023},   //11 Max hall value
  { 0, 0, 2},         //12 EStop mode |0soft|1hard|2off
  { 0, 0, 2},         //13 breaklight mode |0off|1alwaysOn|onWithheadlight
  { 10, 0, 100},      //14 throttle death center
  { 0, 0, 2},         //15 drivingMode
  { -1, 0, 0},        //16 pair new board
  {20, 14, 20},       //17 transmission power
  { -1, 0, 0},        //18 show Key
  { 433, 433 , 433},  //19 Frequency
  { 0, 0 , 1},       //19 Stanby mode
  { -1, 0 , 0},       //20 Firmware
  { -1, 0, 0},        //22 Set default key
  { -1, 0, 0},        //22 Settings
  { -1, 0, 0},        //23 Exit
};

// Definition for RFM69HW radio on Feather m0
#define RFM69_CS    8
#define RFM69_INT   3
#define RFM69_RST   4
#define RF69_FREQ   433.0
#define MY_ADDRESS  1

RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

uint8_t encryptionKey[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x01};

// Current mode of receiver - 0: Connected | 1: Timeout | 2: Updating settings
#define CONNECTED 0
#define TIMEOUT 1
#define COMPLETE 2
#define FAILED 3
#define ESTOP 4

// Last time data was pulled from VESC
unsigned long lastUartPull;
uint16_t uartPullInterval = 200;

// Cruise control
uint16_t cruiseThrottle;
uint16_t cruiseRPM;
bool cruising;

// Status blink LED
uint8_t statusCode = 0;
bool statusLedState = false;
short statusCycleTime = 0;
unsigned long previousStatusMillis, currentMillis, startCycleMillis = 0;

const uint16_t defaultThrottle = 512;

// Defining receiver pins
const uint8_t resetPin = 6;
const uint8_t statusLedPin = 9;
const uint8_t throttlePin = 10;
const uint8_t breakLightPin = 13;
const uint8_t headlightPin = 12;

// Defining headlight/breaklight
unsigned long lastBreakLightBlink = 0;
bool breaklightBlinkOn = false;
uint8_t breaklightMargin = 15;

bool valueHigher = false;
uint16_t ledValue = 100;

// Defining alarm handling
bool alarmActivated = false;
uint16_t goodTransmissions = 0;
unsigned long goodTransissionsTimer = 0;

//Estop start
uint8_t goodTransmissionsEstop = 0;
unsigned long goodTransissionsTimerEstop = 0;
uint16_t eStopThrottlePos = 512;
unsigned long estopTimerDecrese = 0;
unsigned long releaseBreakTimer = 0;

// Initiate Servo class
Servo esc;

// Initiate VescUart class for UART communication
VescUart UART;
uint8_t uartFailCounter = 0;
bool ignoreUartPull = false;

// SETUP
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setup() {

  #ifdef DEBUG
    UART.setDebugPort(&Serial1);
    Serial.begin(115200);
    while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
  #endif

  UART.setSerialPort(&Serial1);
  Serial1.begin(115200);

  while (!Serial1) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  loadFlashSettings();

  pinMode(statusLedPin, OUTPUT);
  pinMode(breakLightPin, OUTPUT);
  pinMode(headlightPin, OUTPUT);
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(RFM69_RST, OUTPUT);

  esc.attach(throttlePin);

  digitalWrite(RFM69_RST, LOW);

  initiateReceiver();

delay(50);

Serial.println(rxSettings.Frequency);
for (uint8_t i = 0; i <=15; i++){
  Serial.print(rxSettings.customEncryptionKey[i]);
}
Serial.println("");
Serial.println("Setup end");

}


// LOOP
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void loop() {

  if (!dataEStop.triggered) {
    Serial.prtin("Mode: "); Serial.prtinln(remPackage.type);
    if (rf69_manager.available()) {
      if (remPackage.type == 0) {
        if (analyseMessage()) {
          if (validateRemPackageEstop()) { // make shure the data is valid
            armEstop();
            speedControl(remPackage.throttle, remPackage.trigger);
            getUartData();
          } else { // if data is not valid, rescue data!
            rescueRemPackage();
          }
        } else {
          if (!validateRemPackageEstop()) {
            rescueRemPackage();
          }
        }
      } else if (remPackage.type == 1) { // join settings transmission
        analyseSettingsMessage();
      } else {
        activateESTOP(0);
      }
    }
    if (millis() - debugData.lastTransmissionAvaible >= 400) {
      if (remPackage.type == 0 && rxSettings.eStopMode < 2 && dataEStop.armed && rxSettings.eStopArmed) {
        Serial.println("Estop Activated!");
        activateESTOP(0);
      } else {
        speedControl(512, 0);
      }
    }
  }  else {
    activateESTOP(0);
  }
  //controlStatusLed();
  headLight();
  breakLight();
  resetAdress();
}

// validateRemPackage
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
bool validateRemPackageEstop(){

  if (remPackage.type > 1 || remPackage.throttle > 1024 || remPackage.trigger > 1 || remPackage.headlight > 1) {

      #ifdef DEBUG
      Serial.println("Shit package?");
      Serial.print("type ");Serial.println(remPackage.type);
      Serial.print("throttle ");Serial.println(remPackage.throttle);
      Serial.print("Trigger ");Serial.println(remPackage.trigger);
      Serial.print("headlight ");Serial.println(remPackage.headlight);
      #endif
      return false;
  } else {
    return true;
  }
}

// rescue received data
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void rescueRemPackage() {

  Serial.println(millis());

  #ifdef DEBUG
  Serial.println("Shit package?");
  Serial.print("type ");Serial.println(remPackage.type);
  Serial.print("throttle ");Serial.println(remPackage.throttle);
  Serial.print("Trigger ");Serial.println(remPackage.trigger);
  Serial.print("headlight ");Serial.println(remPackage.headlight);
  #endif

  remPackage.type = remPackageBackup.type;
  remPackage.throttle = remPackageBackup.throttle;
  remPackage.trigger = remPackageBackup.trigger;
  remPackage.headlight = remPackageBackup.headlight;

  #ifdef DEBUG
  Serial.println("rescue to default");
  Serial.print("type ");Serial.println(remPackage.type);
  Serial.print("throttle ");Serial.println(remPackage.throttle);
  Serial.print("Trigger ");Serial.println(remPackage.trigger);
  Serial.print("headlight ");Serial.println(remPackage.headlight);
  #endif

}


// checkConnection for ESTOP
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void activateESTOP(uint8_t mode) {
  uint8_t decreseThrottleValue;

  if (!dataEStop.triggered) {
      eStopThrottlePos = 512;
      returnData.eStopArmed = false;
      rxSettings.eStopArmed = false;
      dataEStop.triggered = true;
  }

  if (rxSettings.eStopMode == 0){ // slow estop with recover
    decreseThrottleValue = 2;
  } else if (rxSettings.eStopMode == 1) { // hard estop with recover
    decreseThrottleValue = 4;
  } else if (rxSettings.eStopMode == 2) { // not used at the moment
    mode = 1;
    decreseThrottleValue = 0;
  }

  if (!dataEStop.fullBreakDone){

    if (mode == 0) {

      if (millis() - estopTimerDecrese >= 20){
        if (eStopThrottlePos >= 256) {
          eStopThrottlePos = eStopThrottlePos - decreseThrottleValue;
          speedControl( eStopThrottlePos, 0);
          Serial.println(eStopThrottlePos);
          estopTimerDecrese = millis();
        } else {
          if (millis() - estopTimerDecrese >= 2000){
            dataEStop.fullBreakDone = true;
            releaseBreakTimer = millis();
            Serial.println("Full break + 2sec wait done!");
          }
        }
      }
    } else if (mode == 1) { // not used at the moment
      dataEStop.fullBreakDone = true;
    }
  } else {

    if ((millis() - releaseBreakTimer) > 10000) {
      speedControl( 512, 0);
      goodTransmissions = 0;
      dataEStop.triggered = false;
      dataEStop.fullBreakDone = false;
      returnData.eStopArmed = false;
      rxSettings.eStopArmed = false;
      dataEStop.armed = false;
      updateLastTransmissionTimer();
      Serial.println("release break and go on");
    }

    if (rf69_manager.available()) {
      //if ((millis() - goodTransissionsTimer) <= 2000) {
        if (remPackage.type == 1) {
          analyseSettingsMessage();
        } else {

          if (analyseMessage()) {
            Serial.println("Got valid message...");
            if ((millis() - goodTransissionsTimer) <= 2000) {
            if (goodTransmissions >= 15) {
              goodTransmissions = 0;
              dataEStop.triggered = false;
              dataEStop.fullBreakDone = false;
              returnData.eStopArmed = true;
              rxSettings.eStopArmed = true;
              Serial.println("Recovered");
              Serial.print("Recover time: "); Serial.println(millis() - goodTransissionsTimer);
              updateLastTransmissionTimer();
            } else {
              if (remPackage.throttle <= 560){
                goodTransmissions++;
              }
            }
            } else {
              goodTransmissions = 0;
              goodTransissionsTimer = millis();
            }
          }
        }
      //} else {
      //  goodTransmissions = 0;
      //  goodTransissionsTimer = millis();
      //}
    }
  }
}

// reset Adress
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void armEstop(){

  if (!dataEStop.armed) {

    if (millis() - goodTransissionsTimerEstop <= 3000 && remPackage.throttle <= 560){
      goodTransmissionsEstop++;
      if (goodTransmissionsEstop > 10) {
        dataEStop.armed = true;
        returnData.eStopArmed = true;
        rxSettings.eStopArmed = true;
        Serial.print("Arm Estop time: "); Serial.println(goodTransissionsTimer);
      }
    } else {
      goodTransmissionsEstop = 0;
      goodTransissionsTimerEstop = millis();
    }
  }

}

// reset Adress
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
bool resetAdress() {

  if (millis() - debugData.lastTransmissionAvaible > 5000) {
    if (!digitalRead(resetPin)) {

      for (int i = 0; i <=15; i++) {
        rxSettings.customEncryptionKey[i] = encryptionKey[i];
      }
      rxSettings.Frequency = RF69_FREQ;
      dataEStop.armed = false;
      dataEStop.triggered = false;

      updateFlashSettings();
      initiateReceiver();

      Serial.println("Reset");
       Serial.println(rxSettings.Frequency);
       for (uint8_t i = 0; i <=15; i++){
         Serial.print(rxSettings.customEncryptionKey[i]);
       }
       Serial.println("");
    }
  }
}

// analyse transmission
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
bool analyseMessage() {

  remPackageBackup.type = remPackage.type;
  remPackageBackup.throttle = remPackage.throttle;
  remPackageBackup.trigger = remPackage.trigger;
  remPackageBackup.headlight = remPackage.headlight;

  uint8_t len = sizeof(remPackage);
  uint8_t from;
  if (rf69_manager.recvfromAck((uint8_t*)&remPackage, &len, &from)) {

  rf69_manager.setRetries(0);
  rf69_manager.setTimeout(20);

    if (!rf69_manager.sendtoWait((uint8_t*)&returnData, sizeof(returnData), from)) {
    } else {
      updateLastTransmissionTimer();
      return true;
    }

  } else {
    rescueRemPackage();
  }
}

// check settings message
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void analyseSettingsMessage() {

  uint8_t len = sizeof(rxSettings);
  uint8_t from;

  Serial.println("Jump into Settings");

  remPackage.type = 0;

  if (rf69_manager.recvfromAck((uint8_t*)&rxSettings, &len, &from)) {
    Serial.println("New Settings loaded: ");
    Serial.print("rxSettings.Frequency");Serial.println(rxSettings.Frequency);
    for (uint8_t i = 0; i <=15; i++){
      Serial.print(rxSettings.customEncryptionKey[i]);
    }
    Serial.println("");
    Serial.print("boardID: "); Serial.println(rxSettings.boardID);
    Serial.print("triggerMode: "); Serial.println(rxSettings.triggerMode);


    if (!rf69_manager.sendtoWait((uint8_t*)&returnData, sizeof(returnData), from)) {
      updateLastTransmissionTimer();
    }

    updateFlashSettings();
    initiateReceiver();
  }

  Serial.println("Going out of settings");
   Serial.println(rxSettings.Frequency);
   for (uint8_t i = 0; i <=15; i++){
     Serial.print(rxSettings.customEncryptionKey[i]);
   }
   Serial.println("");
}

// update last transmission
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void updateLastTransmissionTimer() {
  debugData.lastTransmissionAvaible = millis();
}

// set status
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setStatus(uint8_t code) {

  short cycle = 0;

  switch (code) {
    case TIMEOUT:  cycle = 600;    break;
    case FAILED:    cycle = 1400;   break;
    case COMPLETE:  cycle = 100;    break;
    case ESTOP:    cycle = 4000;   break;
  }

  currentMillis = millis();

  if (currentMillis - startCycleMillis >= statusCycleTime) {
    statusCode = code;
    statusCycleTime = cycle;
    startCycleMillis = currentMillis;
  }
}

// control status LED TIMEOUT | COMPLETE | FAIL
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void controlStatusLed() {

  short oninterval, offinterval, cycle;

  switch (statusCode) {
    case TIMEOUT:   oninterval = 300;   offinterval = 300;  break;
    case COMPLETE:  oninterval = 50;    offinterval = 50;   break;
    case FAILED:    oninterval = 500;   offinterval = 200;  break;
    case ESTOP:    oninterval = 1000;   offinterval = 200;  break;
  }

  currentMillis = millis();

  if (currentMillis - previousStatusMillis >= offinterval && statusLedState == false ) {

    previousStatusMillis = currentMillis;
    statusLedState = !statusLedState;

  } else if (currentMillis - previousStatusMillis >= oninterval && statusLedState == true) {

    previousStatusMillis = currentMillis;
    statusLedState = !statusLedState;

  }

  digitalWrite(statusLedPin, statusLedState);

}

// initiate receiver radio
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void initiateReceiver() {
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init()) {
    while (1);
  }

  if (!rf69.setFrequency(rxSettings.Frequency)) {
  }
  rf69.setTxPower(20);
  rf69.setEncryptionKey(rxSettings.customEncryptionKey);

  Serial.println(rxSettings.Frequency);
  for (uint8_t i = 0; i <=15; i++){
    Serial.print(rxSettings.customEncryptionKey[i]);
  }
  Serial.println("");


}

// cruise control
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setCruise ( bool cruise = true, uint16_t setPoint = defaultThrottle ) {
  if ( rxSettings.controlMode == 0 ) {

    setThrottle( setPoint );

  }
  else if ( rxSettings.controlMode == 1 ) {

    setThrottle( setPoint );

  }
  else if ( rxSettings.controlMode == 2 ) {

    // Setpoint not used (PID by VESC)
    UART.nunchuck.lowerButton = cruise;
    esc.detach();

    // Make sure the motor doesn't begin to spin wrong way under high load (and don't allow cruise backwards)
    if ( returnData.rpm < 0 ) {

      UART.nunchuck.lowerButton = false;
      UART.nunchuck.valueY = 127;
      UART.setNunchuckValues();
      UART.setCurrent(0.0);

    } else {

      UART.nunchuck.valueY = 127;
      UART.setNunchuckValues();

    }
  }
}

// throttle
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setThrottle( uint16_t throttle ) {
  if ( rxSettings.controlMode == 0 ) {

    esc.attach(throttlePin);
    esc.writeMicroseconds( map(throttle, 0, 1023, 1000, 2000) );
  }
  else if ( rxSettings.controlMode == 1 ) {

    esc.attach(throttlePin);
    esc.writeMicroseconds( map(throttle, 0, 1023, 1000, 2000) );

  }
  else if ( rxSettings.controlMode == 2 ) {

    UART.nunchuck.valueY = map(throttle, 0, 1023, 0, 255);
    UART.nunchuck.lowerButton = false;
    esc.detach();
    UART.setNunchuckValues();

  }
}

// speed control
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void speedControl( uint16_t throttle , bool trigger ) {

  // Kill switch
  if ( rxSettings.triggerMode == 0 ) {
    if ( trigger == true || throttle < 512 ) {
      setThrottle( throttle );
    }
    else {
      setThrottle( defaultThrottle );
    }
  }

  //Cruise control
  else if ( rxSettings.triggerMode == 1 ) {
    if ( trigger == true ) {

      if ( cruising == false ) {
        cruiseThrottle = throttle;
        cruiseRPM = returnData.rpm;
        cruising = true;
      }

      setCruise( true, cruiseThrottle );

    } else {
      cruising = false;
      setThrottle( throttle );
    }
  }
}

// headlight
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void headLight(){
  if (remPackage.headlight == 1) {
    returnData.headlightActive = 1;
    digitalWrite(headlightPin, HIGH);
  } else {
    returnData.headlightActive = 0;
    digitalWrite(headlightPin, LOW);
    }
}

// breakLight
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void breakLight() {

  if (!dataEStop.armed) {
    Serial.print("valueHigher: "); Serial.println(valueHigher);
    if (valueHigher) {
        ledValue = ledValue + 1;
        if (ledValue >= 250){
          valueHigher = false;
        }
    } else {
        ledValue = ledValue - 1;
        if (ledValue <= 5){
          valueHigher = true;
        }
    }
    delay(3);
    analogWrite(breakLightPin, ledValue);
    Serial.print("ledValue: "); Serial.println(ledValue);
  } else {

    if ((remPackage.throttle <= breaklightMargin) || dataEStop.triggered) {
      if (breaklightBlinkOn == true) {
          analogWrite(breakLightPin, 255);
          if (millis() - lastBreakLightBlink >= 50) {
              lastBreakLightBlink = millis();
              breaklightBlinkOn = false;
              }
          } else if (breaklightBlinkOn == false){
              analogWrite(breakLightPin, 0);
              if (millis() - lastBreakLightBlink >= 50) {
                lastBreakLightBlink = millis();
                breaklightBlinkOn = true;
                }
            }

      }else if (remPackage.throttle <= 400) {
        analogWrite(breakLightPin, 255);
      } else {
        analogWrite(breakLightPin, 100);
      }
  }

}

// Uart handling
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void getUartData() {

if (rxSettings.controlMode > 0 && !ignoreUartPull) {
  if ( millis() - lastUartPull >= uartPullInterval ) {

    lastUartPull = millis();

    if ( UART.getVescValues() )
    {
      returnData.ampHours         = UART.data.ampHours;
      returnData.inpVoltage       = UART.data.inpVoltage;
      returnData.rpm              = UART.data.rpm;
      returnData.tachometerAbs    = UART.data.tachometerAbs;
      returnData.avgInputCurrent  = UART.data.avgInputCurrent;
      returnData.avgMotorCurrent  = UART.data.avgMotorCurrent;
      returnData.dutyCycleNow     = UART.data.dutyCycleNow;
      uartFailCounter = 0;
    }
    else
    {
      returnData.ampHours           = 0.0;
      returnData.inpVoltage         = 0.0;
      returnData.rpm                = 0;
      returnData.tachometerAbs      = 0;
      returnData.avgInputCurrent    = 0.0;
      returnData.avgMotorCurrent    = 0.0;
      returnData.dutyCycleNow       = 0.0;

      uartFailCounter++;
      if (uartFailCounter > 20){
        Serial.println("UART pull deactivated");
        ignoreUartPull = true;
      }

    }

  }
}
}

// set default settings
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setDefaultFlashSettings() {
  for ( int i = 0; i < numOfSettings; i++ ) {
    setSettingValue(i, settingRules[i][0]);
    Serial.print("Setting number: "); Serial.print(i); Serial.print(" value: "); Serial.println(settingRules[i][0]);
  }

  rxSettings.firmVersion = VERSION;
  for (int i = 0; i < 16; i++) {
    rxSettings.customEncryptionKey[i] = encryptionKey[i];
  }

  rxSettings.Frequency = RF69_FREQ;

  updateFlashSettings();
}

// load flash settings
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void loadFlashSettings() {

  rxSettings = flash_RxSettings.read();

  if (rxSettings.firmVersion != VERSION) {
    setDefaultFlashSettings();
  }
}

// update flash settings
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void updateFlashSettings() {

  flash_RxSettings.write(rxSettings);

}

// setSettingValue
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setSettingValue(uint8_t index, uint64_t value) {
  switch (index) {
    case 0:         rxSettings.boardID = value;         break;
    case 1:         rxSettings.triggerMode = value;     break;
    case 2:         rxSettings.batteryType = value;     break;
    case 3:         rxSettings.batteryCells = value;    break;
    case 4:         rxSettings.motorPoles = value;      break;
    case 5:         rxSettings.motorPulley = value;     break;
    case 6:         rxSettings.wheelPulley = value;     break;
    case 7:         rxSettings.wheelDiameter = value;   break;
    case 8:         rxSettings.controlMode = value;     break;
    case 9:         rxSettings.minHallValue = value;    break;
    case 10:        rxSettings.centerHallValue = value; break;
    case 11:        rxSettings.maxHallValue = value;    break;
    case 12:        rxSettings.eStopMode = value;       break;
    case 13:        rxSettings.breaklightMode = value;  break;
    case 14:        rxSettings.throttleDeath = value;   break;
    case 15:        rxSettings.drivingMode = value;     break;
    case 17:        rxSettings.transmissionPower = value; break;

    default: /* Do nothing */ break;
  }
}

bool inRange(int val, int minimum, int maximum) {
  return ((minimum <= val) && (val <= maximum));
}

String uint64ToString(uint64_t number) {
  unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
  unsigned long part2 = (unsigned long)((number));

  if (part1 == 0) {
    return String(part2, DEC);
  }

  return String(part1, DEC) + String(part2, DEC);
}
