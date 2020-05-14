/*
    Allows your TV IR remote to control other devices like Audio Amplifier that are equipped with the 3.5mm IR repeater/extender port.
    Imagine you can turn the volume up or down using your TV remote and the amplifier will have the volume adjusted accordingly.
    This project is an alternative to the universal remote (that some people - like me - don't like very much). You can record up to 16 codes.

    Copyright by Stellars Henson 2020
*/

#include <avr/wdt.h>
#include <IRremote.h>
#include <EEPROM.h>
#include <Fsm.h>
#include <EnableInterrupt.h>
#include <Thread.h>
#include <ThreadRunOnce.h>
#include <ThreadController.h>

#define SEND_PIN 3
#define RECV_PIN 2
#define IRCODE_RECORD_PIN 12
#define STATUS_LED_PIN 4
#define IRCODE_STORED_LED_PIN 10
#define IRCODE_RECORD_LED_PIN 9

#define TRIGGER_IRCODE_RECORD 1
#define TRIGGER_IRCODE_RECORDED 2
#define TRIGGER_IRCODE_DETECTED 3
#define TRIGGER_IRCODE_CANCELED 4

#define DEBUG_LEVEL 2 //0 - debug off, 1 - essential messages, 2 - full diagnostics

//saved ir codes - we only need the code from SRC, not length or type
#define IRCODES_NUMBER       16 //number of codes stored in memory
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

static uint8_t irCodes[IRCODES_NUMBER][10] = {0}; //static initialisation to 0 of all elements

uint8_t codeType = 255; // The type of code, 255 is unsigned equivalent of -1
uint8_t codeLen = 0; // The length of the code
uint32_t codeValue = 0; // The code value if not raw

uint8_t toggle = 0; // The RC5/6 toggle state
uint16_t rawCodes[RAWBUF]; // The durations if raw
uint8_t irCodesAvailable = 0; //if ircodes were recorded

//interrupt driver
void on_ircode_record_irq();
void on_ircode_reset_irq();
volatile uint8_t ircode_record_detected = 0;
volatile uint8_t ircode_reset_detected = 0;

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

//threads
ThreadController controll = ThreadController();
ThreadRunOnce runOnceThread = ThreadRunOnce();

/***********************************************************************************************
  ___      _             
 / __| ___| |_ _  _ _ __ 
 \__ \/ -_)  _| || | '_ \
 |___/\___|\__|\_,_| .__/
                   |_|   
***********************************************************************************************/
void setup() {
    Serial.begin(9600);

    pinMode(IRCODE_RECORD_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(IRCODE_STORED_LED_PIN, OUTPUT);
    pinMode(IRCODE_RECORD_LED_PIN, OUTPUT);

    //load saved codes
    uint8_t initStatus = EEPROM.read(0); //if initStatus=1 initialised, initStatus=0 uninitialised

    //system not initialised, writing zeros to eeprom
    if (initStatus == 255) {
        Serial.println(F("[INIT] System first run, bootstrapping initial codes in EEPROM"));
        saveIRCodesToEEPROM();
        EEPROM.update(0, 1); //bootstrap flag
    }
    else {
        Serial.println(F("[INIT] System already bootstrapped"));
        getIRCodesFromEEPROM();
    }

    //find out if we have any codes recorded - and count them
    for (int i = 0; i < IRCODES_NUMBER; i++) irCodesAvailable = (irCodesAvailable > 0 || irCodes[i][IRCODES_SRC_CODE0] != 0 ? ++irCodesAvailable : irCodesAvailable);

    //initialise state machine
    fsm.add_transition(&state_ircode_sense, &state_ircode_record, TRIGGER_IRCODE_RECORD, NULL);
    fsm.add_transition(&state_ircode_record, &state_ircode_sense, TRIGGER_IRCODE_RECORDED, NULL);
    fsm.add_transition(&state_ircode_record, &state_ircode_sense, TRIGGER_IRCODE_CANCELED, NULL);

    if (DEBUG_LEVEL == 2) {
        printIRCodesSerial();
        Serial.print(F("[INIT] System intiated with "));
        Serial.print(irCodesAvailable, DEC);
        Serial.print(F(" recorded IR codes, system initiation status: "));
        Serial.println(initStatus, DEC);
    }
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
}

/*==================================================================================================================== 
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

    //interrupt initialisation
    ircode_record_detected = 0;
    enableInterrupt(IRCODE_RECORD_PIN, on_ircode_record_irq, CHANGE);

    // enable receiver
    irrecv.enableIRIn();

    // reboot();
}

/**
    waits for either ircode record button trigger or for ircode signal
    ircode button -> goes to ircode recording
*/
void on_ircode_sense_loop() {
    int foundCodeIndex = -1;

    // button to enable recording ircode
    // driven by interrupts now on IRCODE_RECORD_PIN
    if (ircode_record_detected == 1) {
        if (DEBUG_LEVEL) Serial.println(F("[IRCODE SENSE] IRCode recording detected"));
        blink(STATUS_LED_PIN, 3, 300);
        fsm.trigger(TRIGGER_IRCODE_RECORD);
    }

    //check the IR code and send response code
    if ( irrecv.decode(&results) ) {
        //read the code and find the translation code
        storeCode(&results, codeType, codeLen, codeValue);

        foundCodeIndex = findIRCodeIndex(codeValue);
        if (foundCodeIndex > 0) {
            codeValue = (unsigned long)(irCodes[foundCodeIndex][IRCODES_TGT_CODE0]) |
                        (unsigned long)(irCodes[foundCodeIndex][IRCODES_TGT_CODE1] << 8) |
                        (unsigned long)(irCodes[foundCodeIndex][IRCODES_TGT_CODE2] << 16) |
                        (unsigned long)(irCodes[foundCodeIndex][IRCODES_TGT_CODE3] << 24);
            codeType = irCodes[foundCodeIndex][IRCODES_TGT_CODETYPE];
            codeLen = irCodes[foundCodeIndex][IRCODES_TGT_CODELEN];

            //send code and resume operation
            sendCode(0, codeType, codeLen, codeValue); //send recorded or saved IR code without repeat
            irrecv.resume(); //resume for the next code
        }
    }
}

/**
    signals exit from the state
*/
void on_ircode_sense_exit() {
    if (DEBUG_LEVEL == 2) Serial.println(F("[IRCODE SENSE] Exit. Disabling ir signal detection"));
    disableInterrupt(IRCODE_RECORD_PIN);
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
    codeType = 255; // The type of code, 255 is unsigned equivalent of -1

    digitalWrite(IRCODE_STORED_LED_PIN, LOW);
    digitalWrite(IRCODE_RECORD_LED_PIN, HIGH);
    if (DEBUG_LEVEL) Serial.println(F("[IRCODE RECORD] Start.  Enabling IR receiver"));
}

/**
    waits until IR remote sends the IR code
*/
void on_ircode_record_loop() {

    //blink pattern for recording: twice for the first code, once for the second code
    //if(startAudioCodeType == 255 && stopAudioCodeType == 255) { blink(IRCODE_RECORD_LED_PIN, 2, 500); delay(500);  }
    //if(startAudioCodeType != 255 && stopAudioCodeType == 255) { blink(IRCODE_RECORD_LED_PIN, 1, 500); delay(500);  }

    //read and decode the ircode, ircode was received with the interrupts
    if (irrecv.decode(&results)) {
        //record start audio code
        if ( codeType == 255 ) {
            if (DEBUG_LEVEL) Serial.println(F("[IRCODE RECORD][1/2] - SIGNAL ircode detected"));
            blink(STATUS_LED_PIN, 3, 300);
            storeCode(&results, codeType, codeLen, codeValue);
            irrecv.resume(); //resume recording for the stop code

            //trigger transition
            fsm.trigger(TRIGGER_IRCODE_RECORDED);  //trigger transition only after audio stop code was recorded
        }
    }
}

/**
    announces exit from ircode recording state and turns off recording LED
*/
void on_ircode_record_exit() {
    if (DEBUG_LEVEL == 2) Serial.println(F("[IRCODE RECORD] Exit. Disabling receiver"));
    digitalWrite(IRCODE_RECORD_LED_PIN, LOW);
    digitalWrite(IRCODE_STORED_LED_PIN, HIGH);
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
        if (DEBUG_LEVEL) Serial.println("Received unknown code, saving as raw");
        codeLen = results->rawlen - 1;
        // To store raw codes:
        // Drop first value (gap)
        // Convert from ticks to microseconds
        // Tweak marks shorter, and spaces longer to cancel out IR receiver distortion
        for (int i = 1; i <= codeLen; i++) {
            if (i % 2) {
                // Mark
                rawCodes[i - 1] = results->rawbuf[i] * USECPERTICK - MARK_EXCESS;
                if (DEBUG_LEVEL == 2) Serial.print(" m");
            }
            else {
                // Space
                rawCodes[i - 1] = results->rawbuf[i] * USECPERTICK + MARK_EXCESS;
                if (DEBUG_LEVEL == 2) Serial.print(" s");
            }
            if (DEBUG_LEVEL == 2) Serial.print(rawCodes[i - 1], DEC);
        }
        if (DEBUG_LEVEL == 2) Serial.println("");
    }
    else {
        if (codeType == NEC) {
            if (DEBUG_LEVEL == 2) Serial.print("Received NEC: ");
            if (results->value == REPEAT) {
                // Don't record a NEC repeat value as that's useless.
                if (DEBUG_LEVEL == 2) Serial.println("repeat; ignoring.");
                return;
            }
        }
        else if (codeType == SONY) {
            if (DEBUG_LEVEL == 2) Serial.print("Received SONY: ");
        }
        else {
            if (DEBUG_LEVEL) Serial.print("Unexpected");
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
            if (DEBUG_LEVEL) Serial.println("Sent NEC repeat");
        }
        else {
            irsend.sendNEC(codeValue, codeLen);
            if (DEBUG_LEVEL) Serial.print("Sent NEC ");
            if (DEBUG_LEVEL) Serial.println(codeValue, HEX);
        }
    }
    else if (codeType == SONY) {
        irsend.sendSony(codeValue, codeLen);
        if (DEBUG_LEVEL) Serial.print("Sent Sony ");
        if (DEBUG_LEVEL) Serial.println(codeValue, HEX);
    }
    else if (codeType == UNKNOWN /* i.e. raw */) {
        // Assume 38 KHz
        irsend.sendRaw(rawCodes, codeLen, 38);
        if (DEBUG_LEVEL) Serial.println("Sent raw");
    }
}

/**
    interrupt handler for ircode record button
*/
void on_ircode_record_irq() {
    ircode_record_detected = 1;
}

/**
    interrupt handler for ircode reset button
*/
void on_ircode_reset_irq(){
	ircode_reset_detected = 1;
}


/**
    blinks pin with a led #cycles number of times, each within the duration
*/
static void  blink( const byte pin, const byte cycles, const unsigned int duration) {
    static uint8_t initialState = digitalRead(pin);
    for (unsigned short i = 0; i < cycles * 2; i++) {
        digitalWrite(pin, ~(i + initialState) & 1);
        delay(duration / 2);
    }
}


/**
    this function saves the ircodes to eeprom
*/
void saveIRCodesToEEPROM() {
    int c = sizeof(irCodes[0]) / sizeof(irCodes[0][0]); //number of columns
    uint8_t eepromAddr = 1; //start with 1st eeprom cell, number 0 is reserved for status
    for (int i = 0; i < IRCODES_NUMBER; i++) for (int j = 0; j < c; j++) EEPROM.update(eepromAddr++, irCodes[i][j]);
    EEPROM.update(0, 1); //set initialised status
}


/**
    this function gets the codes from eeprom
*/
void getIRCodesFromEEPROM() {
    int c = sizeof(irCodes[0]) / sizeof(irCodes[0][0]); //number of columns
    uint8_t eepromAddr = 1; //start with 1st eeprom cell, number 0 is reserved for status
    for (int i = 0; i < IRCODES_NUMBER; i++) for (int j = 0; j < c; j++) irCodes[i][j] = EEPROM.read(eepromAddr++);
}

/*
    finds index of the given codeValue in the ircodes table
    @return index when found, -1 otherwise
*/
int findIRCodeIndex(unsigned long findCodeValue) {
    static int _index = -1;
    static unsigned long _findCodeValue = 0;

    //return cached result
    if (findCodeValue == _findCodeValue && _index > 0) return _index;
    else _findCodeValue = findCodeValue;

    //find the tgt code in the saved codes table
    for (int i = 0; i < IRCODES_NUMBER; i++) {
        unsigned long srcCodeValue = (unsigned long)(irCodes[i][IRCODES_SRC_CODE0]) | (unsigned long)(irCodes[i][IRCODES_SRC_CODE1] << 8) | (unsigned long)(irCodes[i][IRCODES_SRC_CODE2] << 16) | (unsigned long)(irCodes[i][IRCODES_SRC_CODE3] << 24);
        if ( findCodeValue == srcCodeValue ) {
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

    //find first code with value 0
    for (int i = 0; i < IRCODES_NUMBER; i++) {
        unsigned long srcCodeValue = (unsigned long)(irCodes[i][IRCODES_SRC_CODE0]) | (unsigned long)(irCodes[i][IRCODES_SRC_CODE1] << 8) | (unsigned long)(irCodes[i][IRCODES_SRC_CODE2] << 16) | (unsigned long)(irCodes[i][IRCODES_SRC_CODE3] << 24);
        if ( srcCodeValue == 0 ) {
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
    //go through the table and print codes
    for (int i = 0; i < IRCODES_NUMBER; i++) {
        unsigned long srcCodeValue = (unsigned long)(irCodes[i][IRCODES_SRC_CODE0]) | (unsigned long)(irCodes[i][IRCODES_SRC_CODE1] << 8) | (unsigned long)(irCodes[i][IRCODES_SRC_CODE2] << 16) | (unsigned long)(irCodes[i][IRCODES_SRC_CODE3] << 24);
        unsigned long tgtCodeValue = (unsigned long)(irCodes[i][IRCODES_TGT_CODE0]) | (unsigned long)(irCodes[i][IRCODES_TGT_CODE1] << 8) | (unsigned long)(irCodes[i][IRCODES_TGT_CODE2] << 16) | (unsigned long)(irCodes[i][IRCODES_TGT_CODE3] << 24);
        Serial.print(F("IR Codes ["));
        Serial.print(i + 1, DEC);
        Serial.print(F("]: "));
        Serial.print(srcCodeValue, HEX);
        Serial.print(" -> ");
        Serial.println(tgtCodeValue, HEX);
    }
}
