/*
    Allows your TV IR remote to control other devices like Audio Amplifier that are equipped with the 3.5mm IR repeater/extender port.
    Imagine you can turn the volume up or down using your TV remote and the amplifier will have the volume adjusted accordingly.
    This project is an alternative to the universal remote (that some people - like me - don't like very much). You can record up to 16 codes.

    Copyright by Stellars Henson 2020

                     +-----+
        +------------| USB |------------+
        |            +-----+            |
   B5   | [ ]D13/SCK        MISO/D12[ ] |   B4
        | [ ]3.3V           MOSI/D11[ ]~|   B3
        | [ ]V.ref     ___    SS/D10[ ]~|   B2
   C0   | [ ]A0       / N \       D9[ ]~|   B1
   C1   | [ ]A1      /  A  \      D8[ ] |   B0
   C2   | [ ]A2      \  N  /      D7[ ] |   D7
   C3   | [ ]A3       \_0_/       D6[ ]~|   D6
   C4   | [ ]A4/SDA               D5[ ]~|   D5
   C5   | [ ]A5/SCL               D4[ ] |   D4
        | [ ]A6              INT1/D3[ ]~|   D3
        | [ ]A7              INT0/D2[ ] |   D2
        | [ ]5V                  GND[ ] |
   C6   | [ ]RST                 RST[ ] |   C6
        | [ ]GND   5V MOSI GND   TX1[ ] |   D0
        | [ ]Vin   [ ] [ ] [ ]   RX1[ ] |   D1
        |          [ ] [ ] [ ]          |
        |          MISO SCK RST         |
        | NANO-V3                       |
        +-------------------------------+


*/

#include <avr/wdt.h>
#include <IRremote.h>
#include <EEPROM.h>
#include <Fsm.h>
#include <JC_Button.h>
#include <jled.h>
#include <EnableInterrupt.h>


/*****************************************************************************************
  ___ ___ _  _ ___
  | _ \_ _| \| / __|
  |  _/| || .` \__ \
  |_| |___|_|\_|___/
*****************************************************************************************/
#define SEND_PIN 3
#define RECV_PIN 2
#define BUTTON_RECORD_PIN 12 //turns recording on, pullup and button connects to gnd
#define LED_STATUS_PIN 4

#define TRIGGER_IRCODE_RECORD 1
#define TRIGGER_IRCODE_RECORDED 2
#define TRIGGER_IRCODE_DETECTED 3
#define TRIGGER_IRCODE_CANCELED 4

#define DEBUG_LEVEL 2 //0 - debug off, 1 - essential messages, 2 - full diagnostics

//saved ir codes - we only need the code from SRC, not length or type
#define EEPROM_ADDR_INIT     0
#define EEPROM_ADDR_IRCODES  1
#define IRCODES_ARRAY_SIZE   64 //number of codes stored in memory
#define IRCODES_SRC_CODE0    0
#define IRCODES_SRC_CODE1    1
#define IRCODES_SRC_CODE2    2
#define IRCODES_SRC_CODE3    3
#define IRCODES_TGT_CODETYPE 4
#define IRCODES_TGT_CODELEN  5
#define IRCODES_TGT_CODE0    6
#define IRCODES_TGT_CODE1    7
#define IRCODES_TGT_CODE2    8
#define IRCODES_TGT_CODE3    9


//IR receiver setup
IRrecv irrecv(RECV_PIN);
IRsend irsend;
decode_results results;

// Storage for the recorded code
void sendCode(uint8_t repeat, uint8_t codeType, uint8_t codeLen, unsigned long codeValue);
void storeCode(decode_results *results, int8_t &codeType, uint8_t &codeLen, unsigned long &codeValue);

static uint8_t g_irCodes[IRCODES_ARRAY_SIZE][10] = {0}; //static initialisation to 0 of all elements

uint8_t g_rcvCodeType = 255; // The type of code, 255 is unsigned equivalent of -1
uint8_t g_rcvCodeLen = 0; // The length of the code
unsigned long g_rcvCodeValue = 0; // The code value if not raw
int g_codeIndex = -1; //index of the code in the translation table, used for recording

//codes sent after translation
uint8_t g_tgtCodeType = 255;
uint8_t g_tgtCodeLen = 0;
unsigned long g_tgtCodeValue = 0;

uint16_t g_rawCodes[RAW_BUFFER_LENGTH]; // The durations if raw
uint8_t g_irCodesAvailable = 0; //how many g_irCodes were recorded and saved, initiated during setup
uint8_t g_irCodeRecordingStatus = 0; //code recording status used during recording session. 0 - not recorded, 1 - src recorded, 2 - tgt recorded

//interrupt driver
void on_ircode_record_irq();
void on_ircode_reset_irq();
volatile uint8_t g_ircode_record_detected = 0;
volatile uint8_t g_ircode_reset_detected = 0;


//LEDs
//nonblocking LED setup
auto led_noconfig = JLed(LED_STATUS_PIN).Breathe(1000).Forever().DelayAfter(1000);
auto led_configok =  JLed(LED_STATUS_PIN).On();
auto led_ir_recording_1 = JLed(LED_STATUS_PIN).Blink(100, 100).Forever();
auto led_ir_recording_2 = JLed(LED_STATUS_PIN).Blink(500, 500).Forever();
JLed led_active = led_noconfig;


//Buttons
Button cfgButton(BUTTON_RECORD_PIN);
#define BTN_LONGPRESS_DURATION 3000


//FSM - set up of the state machine
void on_ircode_record_enter();
void on_ircode_record_loop();
void on_ircode_record_exit();
void on_ircode_sense_enter();
void on_ircode_sense_loop();
void on_ircode_sense_exit();
State state_ircode_sense(on_ircode_sense_enter, on_ircode_sense_loop, on_ircode_sense_exit);
State state_ircode_record(on_ircode_record_enter, on_ircode_record_loop, on_ircode_record_exit);
Fsm   fsm(&state_ircode_sense);





/***********************************************************************************************
     ___      _
    / __| ___| |_ _  _ _ __
    \__ \/ -_)  _| || | '_ \
    |___/\___|\__|\_,_| .__/
                      |_|
***********************************************************************************************/
void setup() {
  Serial.begin(9600);

  //set up pins
  pinMode(BUTTON_RECORD_PIN, INPUT_PULLUP);
  pinMode(LED_STATUS_PIN, OUTPUT);

  //load saved codes
  uint8_t _initStatus = EEPROM.read(EEPROM_ADDR_INIT); //if initStatus=1 initialised, initStatus=0 uninitialised

  //system not initialised, writing zeros to eeprom
  if (_initStatus == 255 || _initStatus == 0) {
    Serial.println(F("[INIT] System first run, bootstrapping initial codes in EEPROM"));
    saveIRCodesToEEPROM();
    EEPROM.update(EEPROM_ADDR_INIT, 1); //bootstrap flag
  }
  else {
    Serial.println(F("[INIT] System already bootstrapped"));
    getIRCodesFromEEPROM();
  }

  //find out if we have any codes recorded - and count them
  for (int i = 0; i < IRCODES_ARRAY_SIZE; i++) g_irCodesAvailable = (g_irCodes[i][IRCODES_SRC_CODE0] != 0 && g_irCodes[i][IRCODES_SRC_CODE0] != 255 ? ++g_irCodesAvailable : g_irCodesAvailable);

  //initialise state machine
  fsm.add_transition(&state_ircode_sense, &state_ircode_record, TRIGGER_IRCODE_RECORD, NULL);
  fsm.add_transition(&state_ircode_record, &state_ircode_sense, TRIGGER_IRCODE_RECORDED, NULL);
  fsm.add_transition(&state_ircode_record, &state_ircode_sense, TRIGGER_IRCODE_CANCELED, NULL);

  if (DEBUG_LEVEL == 2) {
    printIRCodesSerial();
    Serial.print(F("[INIT] System intiated with "));
    Serial.print(g_irCodesAvailable, DEC);
    Serial.print(F(" recorded IR codes, system initiation status: "));
    Serial.println(_initStatus, DEC);
  }

  //status LEDs
  if (g_irCodesAvailable > 0) led_active = led_configok;
  else led_active = led_noconfig;

  //set up button listeners
  cfgButton.begin();

}

/********************************************************************************************************************
     _
    | |   ___  ___ _ __
    | |__/ _ \/ _ \ '_ \
    |____\___/\___/ .__/
                  |_|
  /*********************************************************************************************************************/
void loop() {
  fsm.run_machine(); //run state loops
  led_active.Update(); //run led driver
}

/*  ====================================================================================================================
     ___ _        _         __  __         _    _
    / __| |_ __ _| |_ ___  |  \/  |__ _ __| |_ (_)_ _  ___
    \__ \  _/ _` |  _/ -_) | |\/| / _` / _| ' \| | ' \/ -_)
    |___/\__\__,_|\__\___| |_|  |_\__,_\__|_||_|_|_||_\___|
    ====================================================================================================================*/
// STATE_IRCODE_SENSE

/**
    only announces the transition
*/
void on_ircode_sense_enter() {
  if (DEBUG_LEVEL) Serial.println(F("[IRCODE SENSE] Start. Listening for ircode and ircode record button"));

  //reset codes
  g_rcvCodeValue, g_rcvCodeLen, g_rcvCodeType = 255; // The type of code, 255 is unsigned equivalent of -1
  g_tgtCodeValue, g_tgtCodeLen, g_tgtCodeType = 255; // The type of code, 255 is unsigned equivalent of -1

  // enable receiver
  irrecv.enableIRIn();
}

/**
    waits for either ircode record button trigger or for ircode signal
    ircode button -> goes to ircode recording
*/
void on_ircode_sense_loop() {
  int _foundIRCodeIndex = -1;
   cfgButton.read();  // read the button

  // button held for longpress to perform reset
  if (cfgButton.pressedFor(BTN_LONGPRESS_DURATION)) {
    if (DEBUG_LEVEL) Serial.println(F("[IRCODE SENSE] reset detected"));
    resetIRCodes();    
    //go back to unconfigured blinking
    led_active = led_noconfig;
  } else
  // click button to enable recording ircode
  if (cfgButton.wasReleased()) {
    delay(250); //avoid bounce
    if (DEBUG_LEVEL) Serial.println(F("[IRCODE SENSE] IRCode recording detected"));
    fsm.trigger(TRIGGER_IRCODE_RECORD);
  }

  //check the IR code and send response code
  if ( irrecv.decode(&results) ) {
    //read the code and find the translation code
    g_rcvCodeType, g_rcvCodeLen, g_rcvCodeValue = 0;
    storeCode(&results, g_rcvCodeType, g_rcvCodeLen, g_rcvCodeValue);

    Serial.print(F("[IRCODE SENSE] IR Signal detected with value: "));
    if (results.value == REPEAT) {
      Serial.print(F("NEC REPEAT (repeating code:  "));
      Serial.print(g_tgtCodeValue, HEX);
      Serial.println(F(")"));
    }
    else Serial.println(g_rcvCodeValue, HEX);

    //if repeat, ignore translation, send the last code and follow with repeat afterwards
    if (results.value == REPEAT) {
      Serial.print(F("[IRCODE SENSE] REPEATING IR code: "));
      Serial.println(g_tgtCodeValue, HEX);
      sendCode(0, g_tgtCodeType, g_tgtCodeLen, g_tgtCodeValue); //send NEC repeat
    }
    //if no repeat, use translation table to send the right code
    else {
      _foundIRCodeIndex = findIRCodeIndex(g_rcvCodeValue);
      if (_foundIRCodeIndex != -1) {
        g_tgtCodeValue = ((unsigned long)g_irCodes[_foundIRCodeIndex][IRCODES_TGT_CODE0]) |
                         ((unsigned long)g_irCodes[_foundIRCodeIndex][IRCODES_TGT_CODE1] << 8) |
                         ((unsigned long)g_irCodes[_foundIRCodeIndex][IRCODES_TGT_CODE2] << 16) |
                         ((unsigned long)g_irCodes[_foundIRCodeIndex][IRCODES_TGT_CODE3] << 24);
        g_tgtCodeType = g_irCodes[_foundIRCodeIndex][IRCODES_TGT_CODETYPE];
        g_tgtCodeLen = g_irCodes[_foundIRCodeIndex][IRCODES_TGT_CODELEN];
        Serial.print(F("[IRCODE SENSE] IR code translation found with index: "));
        Serial.println(_foundIRCodeIndex, DEC);
        Serial.print(F("[IRCODE SENSE] sending translated IR code: "));
        Serial.println(g_tgtCodeValue, HEX);

        //send code and resume operation
        sendCode(0, g_tgtCodeType, g_tgtCodeLen, g_tgtCodeValue); //send recorded or saved IR code without repeat
      } else {
        Serial.println(F("[IRCODE SENSE][ERROR] IR code translation not found"));
        blink(LED_STATUS_PIN, 3, 250);
      }
    }

    //resume listening and reset buffer
    irrecv.resume();
    irrecv.enableIRIn();
  }
}

/**
    signals exit from the state
*/
void on_ircode_sense_exit() {
  if (DEBUG_LEVEL == 2) Serial.println(F("[IRCODE SENSE] Exit. Disabling ir signal detection"));
}

// IRCODE RECORD STATE

/**
    enables IR receiver and turns recording LED on
    it also resets codes recorded previously
*/
void on_ircode_record_enter() {
  // enable receiver
  irrecv.enableIRIn();

  //reset stored codes to allow saving new ones
  g_rcvCodeValue, g_rcvCodeLen, g_rcvCodeType = 255; // The type of code, 255 is unsigned equivalent of -1
  g_irCodeRecordingStatus = 0;

  //enable button interrupts. record button will now cancel the recording
  //and the reset button will reset the codes
  g_ircode_record_detected = 0;
  g_ircode_reset_detected = 0;
  enableInterrupt(BUTTON_RECORD_PIN, on_ircode_record_irq, RISING);

  if (DEBUG_LEVEL) Serial.println(F("[IRCODE RECORD] Start. Enabling IR receiver"));
}

/**
    waits until IR remote sends the IR code
*/
void on_ircode_record_loop() {
  //signal recording status with the status LED
  if (g_irCodeRecordingStatus == 0) blink(LED_STATUS_PIN, 1, 250);
  if (g_irCodeRecordingStatus == 1) blink(LED_STATUS_PIN, 1, 800);

  // button to cancel ircode recording
  if (g_ircode_record_detected == 1) {
    delay(250); //avoid bounce
    if (DEBUG_LEVEL) Serial.println(F("[IRCODE RECORD] IRCode recording canceled"));
    g_ircode_record_detected = 0;
    fsm.trigger(TRIGGER_IRCODE_CANCELED);
  }

  // button to perform reset but only before first code was recorded
  if (g_ircode_reset_detected == 1 && g_irCodeRecordingStatus == 0) {
    delay(250); //avoid bounce
    if (DEBUG_LEVEL) Serial.println(F("[IRCODE RECORD] reset detected"));
    blink(LED_STATUS_PIN, 3, 300);
    resetIRCodes();
    g_ircode_reset_detected = 0;
  }


  //read and decode the src code
  if (irrecv.decode(&results) && results.value != REPEAT && g_irCodeRecordingStatus == 0) {
    storeCode(&results, g_rcvCodeType, g_rcvCodeLen, g_rcvCodeValue);
    if (DEBUG_LEVEL) {
      Serial.print(F("[IRCODE RECORD][1/2] - SRC ircode captured with value: "));
      Serial.println(g_rcvCodeValue, HEX);
    }
    g_irCodeRecordingStatus++;

    //check if code exists already
    g_codeIndex = findIRCodeIndex(g_rcvCodeValue);
    if (g_codeIndex != -1) {
      if (DEBUG_LEVEL) {
        Serial.print(F("[IRCODE RECORD] existing code found with index ["));
        Serial.print(g_codeIndex, DEC);
        Serial.println(F("], overwriting code mapping"));
      }
    }

    //check if need to find next free slot
    if (g_codeIndex == -1) {
      g_codeIndex = findFreeIRCodeIndex();
      if (DEBUG_LEVEL == 2 && g_codeIndex != -1) {
        Serial.print(F("[IRCODE RECORD] next available recording slot found ["));
        Serial.print(g_codeIndex, DEC);
        Serial.println(F("]"));
      }
    }

    //check if index available
    if (g_codeIndex == -1) {
      Serial.println(F("[IRCODE RECORD] no free slots found, exiting"));

      //blink stored LED and set status to ok
      blink(LED_STATUS_PIN, 5, 400);
      led_active = led_configok;

      fsm.trigger(TRIGGER_IRCODE_RECORDED);
      return;
    }

    //store src code in the g_irCodes table
    g_irCodes[g_codeIndex][IRCODES_SRC_CODE0] = g_rcvCodeValue;
    g_irCodes[g_codeIndex][IRCODES_SRC_CODE1] = g_rcvCodeValue >> 8;
    g_irCodes[g_codeIndex][IRCODES_SRC_CODE2] = g_rcvCodeValue >> 16;
    g_irCodes[g_codeIndex][IRCODES_SRC_CODE3] = g_rcvCodeValue >> 24;

    irrecv.resume(); //resume recording for the stop code
  }

  //read and decode the tgt code. slot must have been activated by now
  if (irrecv.decode(&results) && results.value != REPEAT && g_irCodeRecordingStatus == 1) {
    storeCode(&results, g_rcvCodeType, g_rcvCodeLen, g_rcvCodeValue);
    if (DEBUG_LEVEL) {
      Serial.print(F("[IRCODE RECORD][2/2] - TGT ircode captured with value: "));
      Serial.println(g_rcvCodeValue, HEX);
    }
    g_irCodeRecordingStatus++;

    //put target codes in the ircode rable
    g_irCodes[g_codeIndex][IRCODES_TGT_CODE0] = g_rcvCodeValue;
    g_irCodes[g_codeIndex][IRCODES_TGT_CODE1] = g_rcvCodeValue >> 8;
    g_irCodes[g_codeIndex][IRCODES_TGT_CODE2] = g_rcvCodeValue >> 16;
    g_irCodes[g_codeIndex][IRCODES_TGT_CODE3] = g_rcvCodeValue >> 24;
    g_irCodes[g_codeIndex][IRCODES_TGT_CODELEN] = g_rcvCodeLen;
    g_irCodes[g_codeIndex][IRCODES_TGT_CODETYPE] = g_rcvCodeType;

    //save results to EEPROM and show codes
    unsigned long timeStamp = millis();
    Serial.print(F("[IRCODE RECORD] Saving g_irCodes back to EEPROM..."));
    saveIRCodesToEEPROM();
    Serial.print(F(" done, took "));
    Serial.print(millis() - timeStamp, DEC);
    Serial.println(F(" milliseconds"));
    Serial.print(F("[IRCODE RECORD] Readong g_irCodes back from EEPROM..."));
    timeStamp = millis();
    getIRCodesFromEEPROM();
    Serial.print(F(" done, took "));
    Serial.print(millis() - timeStamp, DEC);
    Serial.println(F(" milliseconds"));
    printIRCodesSerial();

    //blink and turn STORED LED on
    blink(LED_STATUS_PIN, 3, 300);
    led_active = led_configok;

    //reset ir receiver and trigger transition
    irrecv.resume(); //resume recording for the stop code
    fsm.trigger(TRIGGER_IRCODE_RECORDED);  //trigger transition only after target code was captured
  }
}

/**
    announces exit from ircode recording state and turns off recording LED
*/
void on_ircode_record_exit() {
  if (DEBUG_LEVEL == 2) Serial.println(F("[IRCODE RECORD] Exit. Disabling receiver"));
  disableInterrupt(BUTTON_RECORD_PIN);
}

/*==================================================================================================================
     _   _ _   _ _ _ _   _
    | | | | |_(_) (_) |_(_)___ ___
    | |_| |  _| | | |  _| / -_|_-<
     \___/ \__|_|_|_|\__|_\___/__/
  ====================================================================================================================*/

/**
    Stores the code for later playback
    Most of this code is just logging

    @param results - pointer to the structure with the results decoded
    @param codeType - type of the IR code (NEC, SONY etc..)
    @codeLen - length of the code in bytes
    @codeValue - 4 bytes long code value
*/
void storeCode(decode_results *results, uint8_t &codeType, uint8_t &codeLen, unsigned long &codeValue) {
  codeType = results->decode_type;
  uint8_t count = results->rawlen;
  if (codeType == UNKNOWN) {
    if (DEBUG_LEVEL) Serial.println(F("Received unknown code, saving as raw"));
    codeLen = results->rawlen - 1;
    // To store raw codes:
    // Drop first value (gap)
    // Convert from ticks to microseconds
    // Tweak marks shorter, and spaces longer to cancel out IR receiver distortion
    for (int i = 1; i <= codeLen; i++) {
      if (i % 2) {
        // Mark
        g_rawCodes[i - 1] = results->rawbuf[i] * MICROS_PER_TICK - MARK_EXCESS_MICROS;
        if (DEBUG_LEVEL == 2) Serial.print(" m");
      }
      else {
        // Space
        g_rawCodes[i - 1] = results->rawbuf[i] * MICROS_PER_TICK - MARK_EXCESS_MICROS;
        if (DEBUG_LEVEL == 2) Serial.print(" s");
      }
      if (DEBUG_LEVEL == 2) Serial.print(g_rawCodes[i - 1], DEC);
    }
    if (DEBUG_LEVEL == 2) Serial.println("");
  }
  else {
    if (codeType == NEC) {
      if (DEBUG_LEVEL == 2) Serial.print(F("Received NEC: "));
      if (results->value == REPEAT) {
        // Don't record a NEC repeat value as that's useless.
        if (DEBUG_LEVEL == 2) Serial.println(F("repeat; ignoring."));
        return;
      }
    }
    else if (codeType == SONY) {
      if (DEBUG_LEVEL == 2) Serial.print(F("Received SONY: "));
    }
    else {
      if (DEBUG_LEVEL) Serial.print(F("Unexpected"));
      if (DEBUG_LEVEL) Serial.print(codeType, DEC);
      if (DEBUG_LEVEL) Serial.println("");
    }
    if (DEBUG_LEVEL) Serial.println(results->value, HEX);
    codeValue = results->value;
    codeLen = results->bits;
  }
}

/**
    sends the IR code recorded in the global variables: codeType, codeValue, codeLen
    the IR code is sent using PWM pin and IR LED

*/
void sendCode(uint8_t repeat, uint8_t codeType, uint8_t codeLen, unsigned long codeValue) {
  if (codeType == NEC) {
    if (repeat) {
      irsend.sendNEC(REPEAT, codeLen);
      if (DEBUG_LEVEL) Serial.println(F("Sent NEC repeat"));
    }
    else {
      irsend.sendNEC(codeValue, codeLen);
      if (DEBUG_LEVEL) Serial.print(F("Sent NEC "));
      if (DEBUG_LEVEL) Serial.println(codeValue, HEX);
    }
  }
  else if (codeType == SONY) {
    irsend.sendSony(codeValue, codeLen);
    if (DEBUG_LEVEL) Serial.print(F("Sent Sony "));
    if (DEBUG_LEVEL) Serial.println(codeValue, HEX);
  }
  else if (codeType == UNKNOWN /* i.e. raw */) {
    // Assume 38 KHz
    irsend.sendRaw(g_rawCodes, codeLen, 38);
    if (DEBUG_LEVEL) Serial.println(F("Sent raw"));
  }
}

/**
    interrupt handler for ircode record button
    pin has pullup enabled and the button shorts the pin to GND
    interrupt must be enabled on the raising edge
*/
void on_ircode_record_irq() {
  g_ircode_record_detected = 1;
}

/**
    interrupt handler for ircode reset button
	pin has pullup enabled and the button shorts the pin to GND
    interrupt must be enabled on the raising edge
*/
void on_ircode_reset_irq() {
  g_ircode_reset_detected = 1;
}



/**
    blinks pin with a led #cycles number of times, each within the duration
*/
void  blink( const byte pin, const byte cycles, const unsigned int duration) {
  blink(pin, cycles, duration, -1);
}

/**
    blinks pin with a led #cycles number of times, each within the duration
*/
void  blink( const byte pin, const byte cycles, const unsigned int duration, const byte initialState) {
  byte _initialState;
  if (initialState != -1) _initialState = digitalRead(pin);
  else _initialState = initialState;

  for (unsigned short i = 0; i < cycles * 2; i++) {
    digitalWrite(pin, ~(i + _initialState) & 1);
    delay(duration / 2);
  }
}


/**
    this function saves the irCodes to eeprom
*/
void saveIRCodesToEEPROM() {
  int c = sizeof(g_irCodes[0]) / sizeof(g_irCodes[0][0]); //number of columns
  uint8_t _eepromAddr = EEPROM_ADDR_IRCODES; //start with 1st eeprom cell, number 0 is reserved for status
  for (int i = 0; i < IRCODES_ARRAY_SIZE; i++) for (int j = 0; j < c; j++) EEPROM.update(_eepromAddr++, g_irCodes[i][j]);
  EEPROM.update(EEPROM_ADDR_INIT, 1); //set initialised status
}


/**
    this function gets the codes from eeprom
*/
void getIRCodesFromEEPROM() {
  int c = sizeof(g_irCodes[0]) / sizeof(g_irCodes[0][0]); //number of columns
  uint8_t _eepromAddr = EEPROM_ADDR_IRCODES; //start with 1st eeprom cell, number 0 is reserved for status
  for (int i = 0; i < IRCODES_ARRAY_SIZE; i++) for (int j = 0; j < c; j++) g_irCodes[i][j] = EEPROM.read(_eepromAddr++);
}


/**
    reset all codes and write them to eeprom
*/
void resetIRCodes() {
  int c = sizeof(g_irCodes[0]) / sizeof(g_irCodes[0][0]); //number of columns

  Serial.println("[RESET] resetting all IR codes and writing them to EEPROM");
  for (int i = 0; i < IRCODES_ARRAY_SIZE; i++) for (int j = 0; j < c; j++) g_irCodes[i][j] = 0;
  saveIRCodesToEEPROM();
  printIRCodesSerial();

  //set led to initial state
  led_active = led_noconfig;
}


/*
    finds index of the given codeValue in the ircodes table
    @return index when found, -1 otherwise
*/
int findIRCodeIndex(unsigned long findCodeValue) {
  static int _index = -1;
  static unsigned long _findCodeValue = 0;
  unsigned long _srcCodeValue = 0;

  //return cached result
  if (findCodeValue == _findCodeValue && _index > 0) return _index;
  else _findCodeValue = findCodeValue;

  //reinitialise values
  _index = -1;
  _findCodeValue = 0;

  //find the tgt code in the saved codes table
  for (int i = 0; i < IRCODES_ARRAY_SIZE; i++) {
    _srcCodeValue = ((unsigned long)g_irCodes[i][IRCODES_SRC_CODE0]) | ((unsigned long)g_irCodes[i][IRCODES_SRC_CODE1] << 8) | ((unsigned long)g_irCodes[i][IRCODES_SRC_CODE2] << 16) | ((unsigned long)g_irCodes[i][IRCODES_SRC_CODE3] << 24);
    if ( findCodeValue == _srcCodeValue ) {
      _index = i;
      break;
    }
  }

  //return index of the found code
  return _index;
}

/**
    returns free ircode index to write the codes to
    when no free slot was found, return -1
*/
int findFreeIRCodeIndex() {
  int _index = -1;
  unsigned long _srcCodeValue = 0;

  //find first code with value 0
  for (int i = 0; i < IRCODES_ARRAY_SIZE; i++) {
    _srcCodeValue = ((unsigned long)g_irCodes[i][IRCODES_SRC_CODE0]) | ((unsigned long)g_irCodes[i][IRCODES_SRC_CODE1] << 8) | ((unsigned long)g_irCodes[i][IRCODES_SRC_CODE2] << 16) | ((unsigned long)g_irCodes[i][IRCODES_SRC_CODE3] << 24);
    if ( _srcCodeValue == 0 || _srcCodeValue == 0xff ) {
      _index = i;
      break;
    }
  }

  //return index of the found code
  return _index;
}

/**
    prints recorded ircodes and their translations
*/
void printIRCodesSerial() {
  unsigned long _srcCodeValue = 0;
  unsigned long _tgtCodeValue = 0;

  //go through the table and print codes
  for (int i = 0; i < IRCODES_ARRAY_SIZE; i++) {
    _srcCodeValue = ((unsigned long)g_irCodes[i][IRCODES_SRC_CODE0]) | ((unsigned long)g_irCodes[i][IRCODES_SRC_CODE1] << 8) | ((unsigned long)g_irCodes[i][IRCODES_SRC_CODE2] << 16) | ((unsigned long)g_irCodes[i][IRCODES_SRC_CODE3] << 24);
    _tgtCodeValue = ((unsigned long)g_irCodes[i][IRCODES_TGT_CODE0]) | ((unsigned long)g_irCodes[i][IRCODES_TGT_CODE1] << 8) | ((unsigned long)g_irCodes[i][IRCODES_TGT_CODE2] << 16) | ((unsigned long)g_irCodes[i][IRCODES_TGT_CODE3] << 24);
    Serial.print(F("IR Codes ["));
    Serial.print(i + 1, DEC);
    Serial.print(F("]: "));
    Serial.print(_srcCodeValue, HEX);
    Serial.print(F(" -> "));
    Serial.println(_tgtCodeValue, HEX);
  }
}
