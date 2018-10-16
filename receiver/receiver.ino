// FeatherFly Receiver - eSk8 Remote 
//
// basic code by SolidGeek | https://github.com/SolidGeek/nRF24-Esk8-Remote
// modified to run on an Feather M0 with RFM69 | https://github.com/StefanMeGit/FeatherM0-Esk8-Remote
//
// ATTENTION!!! This is not for daily use!! Still under hard development!

#include <SPI.h>
#include <Servo.h>
#include <FlashStorage.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include "VescUart.h"

#define DEBUG

#define VERSION 0.1

#ifdef DEBUG
	#define DEBUG_PRINT(x)  Serial.println (x)
	#include "printf.h"
#else
	#define DEBUG_PRINT(x)
#endif

// Transmit and receive package
struct package {		  	// | Normal 	| Setting 	| Confirm
	uint8_t type;		    // | 0 			| 1 		| 2
	uint16_t throttle;		// | Throttle 	| ---		| ---
	uint8_t trigger;	  	// | Trigger 	| --- 		| ---
	uint8_t headlight;
} remPackage;

#define NORMAL 0
#define SETTING 1
#define CONFIRM 2

// When receiving a "type: 1" package save the next transmission (a new setting) in this struct 
struct settingPackage {
	uint8_t setting;
	uint64_t value; 
} setPackage;

// Defining struct to handle callback data (auto ack)
struct callback {
	float ampHours = 11.11;
	float inpVoltage = 22.22;
	long rpm  = 33333;
	long tachometerAbs = 4000;
	uint8_t headlight = 5;
	float avgInputCurrent = 6.6;
	float avgMotorCurrent = 7.7;
	float dutyCycleNow = 8.8;
} returnData;

// Defining struct to hold setting values while remote is turned on.
typedef struct {
  uint8_t triggerMode;  		// 0
  uint8_t batteryType;  		// 1
  uint8_t batteryCells;   		// 2
  uint8_t motorPoles;   		// 3
  uint8_t motorPulley;  		// 4
  uint8_t wheelPulley;  		// 5
  uint8_t wheelDiameter;  		// 6
  uint8_t controlMode;  		// 7
  short minHallValue;     		// 8
  short centerHallValue; 		// 9
  short maxHallValue;     		// 10
  uint8_t boardID; 				// 11
  uint8_t transmissionPower;			// 12
  float firmVersion;          // 13
  uint8_t customEncryptionKey[16];   // 14
} RxSettings;

RxSettings rxSettings;

//Defining flash storage
FlashStorage(flash_RxSettings, RxSettings);

const uint8_t numOfSettings = 3;
// Setting rules format: default, min, max.
const short settingRules[numOfSettings][3] {
	{0, 0, 1}, // 0: Killswitch | 1: Cruise   
	{1,	0, 2}, // 0: PPM only   | 1: PPM and UART | 2: UART only
 	{-1, 0, 0}
};

// Definition for RFM69HW radio on Feather m0
#define RFM69_CS     8
#define RFM69_INT   3
#define RFM69_RST   4
#define RF69_FREQ   433.0
#define MY_ADDRESS     1

RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

uint8_t encryptionKey[16] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

// Current mode of receiver - 0: Connected | 1: Timeout | 2: Updating settings
#define CONNECTED 0
#define TIMEOUT 1
#define COMPLETE 2
#define FAILED 3

// Last time data was pulled from VESC
unsigned long lastUartPull;
uint16_t uartPullInterval = 250;

// Cruise control
uint16_t cruiseThrottle;
uint16_t cruiseRPM;
bool cruising;

// Address reset button
unsigned long resetButtonTimer;
bool resetButtonState = LOW;

// Status blink LED
uint8_t statusCode = 0;
bool statusLedState = false;
short statusCycleTime = 0;
unsigned long previousStatusMillis, currentMillis, startCycleMillis = 0;

const uint16_t defaultThrottle = 512;
const short timeoutMax = 500;

// Defining receiver pins
const uint8_t statusLedPin = 13;
const uint8_t throttlePin = 5;

// Initiate Servo class
Servo esc;

// Initiate VescUart class for UART communication
VescUart UART;

// SETUP
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setup()
{
	#ifdef DEBUG
    	UART.setDebugPort(&Serial);
		Serial.begin(115200);
		 while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
		DEBUG_PRINT("** Esk8-remote receiver **");
		printf_begin();
	#endif

	loadFlashSettings();

	pinMode(statusLedPin, OUTPUT);
	esc.attach(throttlePin);
	
	pinMode(RFM69_RST, OUTPUT);
	
	digitalWrite(RFM69_RST, LOW);
	
  	initiateReceiver();
	#ifdef DEBUG
		DEBUG_PRINT("Setup complete");
	#endif
		
}

// LOOP
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void loop()
{
  // check if message is available
  if (rf69_manager.available()) {
    if (remPackage.type = 0) { // join normal transmission
	    analyseMessage();
	  } else {
	    analyseSettingsMessage(); // join settings transmission
    }
  }
}

// check transmission
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void analyseMessage() {
	Serial.println(" join analyseMessage EncryptionKey: ");
	for(int i = 1; i < 16; i++)
	{
		Serial.print(rxSettings.customEncryptionKey[i]);
	}
	Serial.println("");

    uint8_t len = sizeof(remPackage);
    uint8_t from;
    if (rf69_manager.recvfromAck((uint8_t*)&remPackage, &len, &from)) {
  
      #ifdef DEBUG
        Serial.print("Received valid transmission from remote with ID: "); Serial.print(from);
        Serial.print(" [RSSI :");
        Serial.print(rf69.lastRssi());
        Serial.println("] : ");
        Serial.print("Type: ");Serial.println(remPackage.type);
        Serial.print("Throttle: ");Serial.println(remPackage.throttle);
        Serial.print("Trigger: ");Serial.println(remPackage.trigger);
        Serial.print("Headlight: ");Serial.println(remPackage.headlight);
      #endif
      
      if (!rf69_manager.sendtoWait((uint8_t*)&returnData, sizeof(returnData), from))
      #ifdef DEBUG
        Serial.println("Sending failed (no ack)");
      #endif
	  
    }
}

// check settings message
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void analyseSettingsMessage() {
	
	Serial.println("join analyseSettingsMessage EncryptionKey: ");

    uint8_t len = sizeof(rxSettings);
    uint8_t from;
	if (rf69_manager.recvfromAck((uint8_t*)&rxSettings, &len, &from)) {
  
      #ifdef DEBUG
        Serial.print("Received valid transmission from remote with ID: "); Serial.print(from);
        Serial.print(" [RSSI :");
        Serial.print(rf69.lastRssi());
        Serial.println("] : ");
        Serial.print("triggerMode: ");Serial.println(rxSettings.triggerMode);
        Serial.print("controlMode: ");Serial.println(rxSettings.controlMode);
        Serial.print("boardID: ");Serial.println(rxSettings.boardID);
      #endif
      
      if (!rf69_manager.sendtoWait((uint8_t*)&remPackage, sizeof(remPackage), from)) {
		  
      #ifdef DEBUG
        Serial.println("Sending failed (no ack)");
      #endif
	  
	  }
	  Serial.println("exit analyseSettingsMessage EncryptionKey: ");
	
	  for(int i = 1; i < 16; i++) {
		
		  Serial.print(rxSettings.customEncryptionKey[i]);
	
	  }
	
	Serial.println("");
	
    remPackage.type = 0;
	}
}

// set status
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setStatus(uint8_t code){

  short cycle = 0;

  switch(code){
    case COMPLETE:  cycle = 500;    break;
    case FAILED:    cycle = 1400;   break;
  }

  currentMillis = millis();

  if(currentMillis - startCycleMillis >= statusCycleTime){
    statusCode = code;
    statusCycleTime = cycle; 
    startCycleMillis = currentMillis;
  }
}

// control status LED TIMEOUT | COMPLETE | FAIL
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void controlStatusLed(){

  short oninterval, offinterval, cycle;

  switch(statusCode){
    case TIMEOUT:   oninterval = 300;   offinterval = 300;  break;
    case COMPLETE:  oninterval = 50;    offinterval = 50;   break;
    case FAILED:    oninterval = 500;   offinterval = 200;  break;
  }

  currentMillis = millis();

  if (currentMillis - previousStatusMillis >= offinterval && statusLedState == false ) {

    previousStatusMillis = currentMillis;
    statusLedState = !statusLedState;
    
  }else if(currentMillis - previousStatusMillis >= oninterval && statusLedState == true){

    previousStatusMillis = currentMillis;
    statusLedState = !statusLedState;
    
  }

  if(statusCode == CONNECTED){
    analogWrite(statusLedPin, map(remPackage.throttle, 0, 1023, 0, 255)); 
  }else{
    digitalWrite(statusLedPin, statusLedState);
  }  
}

// initiate receiver radio 
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void initiateReceiver(){
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69_manager.init()) {
    DEBUG_PRINT( F("RFM69 radio init failed") );
    while (1);
  }  
  
  DEBUG_PRINT( F("RFM69 radio init ok") );
  
  if (!rf69.setFrequency(RF69_FREQ)) {
    DEBUG_PRINT( F("setFrequency failed") );
  }
  Serial.print("Receiver set frequency to: "); Serial.println(RF69_FREQ);
  
  rf69.setTxPower(20, true);
  rf69.setEncryptionKey(rxSettings.customEncryptionKey);
  Serial.print("Receiver set customEncryptionKey to: ");
	for(int i = 0; i < 16; i++) {
		Serial.print(rxSettings.customEncryptionKey[i]);
	}
	Serial.println("");
}

// update a single setup value
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void updateSetting( uint8_t setting, uint64_t value) {
	// Map remote setting indexes to receiver settings
	switch( setting ){
		case 0: setting = 0; break;  // TriggerMode
		case 7: setting = 1; break;  // ControlMode
	}
	
	setSettingValue( setting, value);
	
	updateFlashSettings(); 

}

// cruise control
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setCruise ( bool cruise = true, uint16_t setPoint = defaultThrottle ){
  if( rxSettings.controlMode == 0 ){

    setThrottle( setPoint );
    
  }
  else if( rxSettings.controlMode == 1 ){
    
    setThrottle( setPoint );
    
  }
  else if( rxSettings.controlMode == 2 ){

    // Setpoint not used (PID by VESC)
    UART.nunchuck.lowerButton = cruise;
    esc.detach();

    // Make sure the motor doesn't begin to spin wrong way under high load (and don't allow cruise backwards)
    if( returnData.rpm < 0 ){

      UART.nunchuck.lowerButton = false;
      UART.nunchuck.valueY = 127;
      UART.setNunchuckValues();
      UART.setCurrent(0.0);
 
    } else{

      UART.nunchuck.valueY = 127;
      UART.setNunchuckValues();
      
    }
  }
}

// throttle
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setThrottle( uint16_t throttle ) {
  if( rxSettings.controlMode == 0 ){
    
    esc.attach(throttlePin);
    esc.writeMicroseconds( map(throttle, 0, 1023, 1000, 2000) );  
  }
  else if( rxSettings.controlMode == 1 ){

    esc.attach(throttlePin);
    esc.writeMicroseconds( map(throttle, 0, 1023, 1000, 2000) ); 

  }
  else if( rxSettings.controlMode == 2 ){
    
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
	if( rxSettings.triggerMode == 0 ){
		if ( trigger == true || throttle < 512 ){
			setThrottle( throttle );
		}
		else{
			setThrottle( defaultThrottle );
		}
	}

	// Cruise control
	else if( rxSettings.triggerMode == 1 ){ 
    if( trigger == true ){
      
      if( cruising == false ){
        cruiseThrottle = throttle;
        cruiseRPM = returnData.rpm;
        cruising = true;
      }

      setCruise( true, cruiseThrottle );
      
    }else{
      cruising = false;
      setThrottle( throttle );
    }
	}
} 

// Uart handling
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void getUartData() {

	if ( millis() - lastUartPull >= uartPullInterval ) {

		lastUartPull = millis();
	#ifdef DEBUG
    	DEBUG_PRINT("Getting the DATA");
    #endif	

		// Only get what we need
		if ( UART.getVescValues() )
		{
			returnData.ampHours 		 	= UART.data.ampHours;
			returnData.inpVoltage		  	= UART.data.inpVoltage;
			returnData.rpm 				    = UART.data.rpm;
			returnData.tachometerAbs 		= UART.data.tachometerAbs;
			returnData.avgInputCurrent 		= UART.data.avgInputCurrent;
			returnData.avgMotorCurrent 		= UART.data.avgMotorCurrent;
			returnData.dutyCycleNow			= UART.data.dutyCycleNow;
			
			#ifdef DEBUG // TODO make this nice viewable
				DEBUG_PRINT(returnData.ampHours);
				DEBUG_PRINT(returnData.inpVoltage);
				DEBUG_PRINT(returnData.rpm);
				DEBUG_PRINT(returnData.tachometerAbs);
				DEBUG_PRINT(returnData.avgInputCurrent);
				DEBUG_PRINT(returnData.avgMotorCurrent);
				DEBUG_PRINT(returnData.dutyCycleNow);
			#endif
		} 
		else
		{
			returnData.ampHours 			= 0.0;
			returnData.inpVoltage     		= 0.0;
			returnData.rpm 				   	= 0;
			returnData.tachometerAbs  		= 0;
			returnData.avgInputCurrent 		= 0.0;
			returnData.avgMotorCurrent 		= 0.0;
			returnData.dutyCycleNow			= 0.0;
		}
	}
}

// set default settings
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setDefaultFlashSettings() {
	for ( int i = 0; i < numOfSettings; i++ ) {
		setSettingValue(i, settingRules[i][0]);
	}

  	rxSettings.firmVersion = VERSION;
	for (int i = 0; i < 16; i++) {
		rxSettings.customEncryptionKey[i] = encryptionKey[i];
	}
	
	#ifdef DEBUG
		DEBUG_PRINT("Default settings loaded, update flash");
	#endif
	updateFlashSettings();
}

// load flash settings
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void loadFlashSettings(){
	
   rxSettings = flash_RxSettings.read();

	#ifdef DEBUG
		DEBUG_PRINT("Settings loaded");
	#endif
	
  if(rxSettings.firmVersion != VERSION){
  	#ifdef DEBUG
		DEBUG_PRINT("Firmware Version is invalid, load default settings");
	#endif
    setDefaultFlashSettings();
  }
}

// update flash settings
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void updateFlashSettings() {
    flash_RxSettings.write(rxSettings);
    
    #ifdef DEBUG
		DEBUG_PRINT("Settings updated to flash");
	#endif
   
}
// set setting value
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
void setSettingValue(int index, uint64_t value) {
	switch (index) {
		case 0: rxSettings.triggerMode = value; break;
		case 1: rxSettings.controlMode = value; break;
    
    default: /* Do nothing */ break;
	}
}

// Get settings value by index (usefull when iterating through settings).
int getSettingValue(uint8_t index) {
	int value;
	switch (index) {
		case 0: value = rxSettings.triggerMode; break;
		case 1: value = rxSettings.controlMode; break;
    
    default: /* Do nothing */ break;
	}
	return value;
}

bool inRange(int val, int minimum, int maximum) {
	return ((minimum <= val) && (val <= maximum));
}

String uint64ToString(uint64_t number) {
	unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
	unsigned long part2 = (unsigned long)((number));

	if(part1 == 0){
		return String(part2, DEC);
	}
	
	return String(part1, DEC) + String(part2, DEC);
}
