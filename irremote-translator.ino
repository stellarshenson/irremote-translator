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


/*****************************************************************************************
  ___ ___ _  _ ___ 
 | _ \_ _| \| / __|
 |  _/| || .` \__ \
 |_| |___|_|\_|___/
*****************************************************************************************/
#define SEND_PIN 3
#define RECV_PIN 2
#define BUTTON_RECORD_PIN 12 //turns recording on, pullup and button connects to gnd
#define BUTTON_RESET_PIN 11 //resets ir codes, pullup and button connects to gnd
#define LED_STATUS_PIN 4
#define LED_STORED_PIN 5


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

uint8_t rcvCodeType = 255; // The type of code, 255 is unsigned equivalent of -1
uint8_t rcvCodeLen = 0; // The length of the code
unsigned long rcvCodeValue = 0; // The code value if not raw
int codeIndex = -1; //index of the code in the translation table, used for recording

//codes sent after translation
uint8_t tgtCodeType = 255; 
uint8_t tgtCodeLen = 0;
unsigned long tgtCodeValue = 0;

uint8_t toggle = 0; // The RC5/6 toggle state
uint16_t rawCodes[RAWBUF]; // The durations if raw
uint8_t irCodesAvailable = 0; //how many ircodes were recorded and saved, initiated during setup
uint8_t irCodeRecordingStatus = 0; //code recording status used during recording session. 0 - not recorded, 1 - src recorded, 2 - tgt recorded

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

    pinMode(BUTTON_RECORD_PIN, INPUT_PULLUP);
    pinMode(BUTTON_RESET_PIN, INPUT_PULLUP);
    pinMode(LED_STATUS_PIN, OUTPUT);
	pinMode(LED_STORED_PIN, OUTPUT);

    //load saved codes
    uint8_t initStatus = EEPROM.read(0); //if initStatus=1 initialised, initStatus=0 uninitialised

    //system not initialised, writing zeros to eeprom
    if (initStatus == 255 || initStatus == 0) {
        Serial.println(F("[INIT] System first run, bootstrapping initial codes in EEPROM"));
        saveIRCodesToEEPROM();
        EEPROM.update(0, 1); //bootstrap flag
    }
    else {
        Serial.println(F("[INIT] System already bootstrapped"));
        getIRCodesFromEEPROM();
    }

    //find out if we have any codes recorded - and count them
    for (int i = 0; i < IRCODES_NUMBER; i++) irCodesAvailable = (irCodes[i][IRCODES_SRC_CODE0] != 0 ? ++irCodesAvailable : irCodesAvailable);

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

    //status LEDs
    if(irCodesAvailable > 0) digitalWrite(LED_STORED_PIN, HIGH);
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

    //interrupt initialisation
    ircode_record_detected = 0;
    ircode_reset_detected = 0;
    enableInterrupt(BUTTON_RECORD_PIN, on_ircode_record_irq, RISING);
    enableInterrupt(BUTTON_RESET_PIN, on_ircode_reset_irq, RISING);

	//reset codes
	rcvCodeValue, rcvCodeLen, rcvCodeType = 255; // The type of code, 255 is unsigned equivalent of -1
	tgtCodeValue, tgtCodeLen, tgtCodeType = 255; // The type of code, 255 is unsigned equivalent of -1

    // enable receiver
    irrecv.enableIRIn();
}

/**
    waits for either ircode record button trigger or for ircode signal
    ircode button -> goes to ircode recording
*/
void on_ircode_sense_loop() {
    int _foundIRCodeIndex = -1;

    // button to enable recording ircode
    if (ircode_record_detected == 1) {
        delay(250); //avoid bounce
        if (DEBUG_LEVEL) Serial.println(F("[IRCODE SENSE] IRCode recording detected"));
        fsm.trigger(TRIGGER_IRCODE_RECORD);
        ircode_record_detected = 0;
    }

    // button to perform reset
    if (ircode_reset_detected == 1) {
        if (DEBUG_LEVEL) Serial.println(F("[IRCODE SENSE] reset detected"));
        delay(1250); //avoid bounce
        blink(LED_STATUS_PIN, 3, 300);
        resetIRCodes();
        ircode_reset_detected = 0;
    }


    //check the IR code and send response code
    if ( irrecv.decode(&results) ) {
        //read the code and find the translation code
        rcvCodeType, rcvCodeLen, rcvCodeValue = 0;
        storeCode(&results, rcvCodeType, rcvCodeLen, rcvCodeValue);
        
        Serial.print(F("[IRCODE SENSE] IR Signal detected with value: "));
        if (results.value == REPEAT) { Serial.print(F("NEC REPEAT (repeating code:  ")); Serial.print(tgtCodeValue, HEX); Serial.println(F(")")); }
        else Serial.println(rcvCodeValue, HEX);
        
        //if repeat, ignore translation, send the last code and follow with repeat afterwards
        if (results.value == REPEAT) {
        	Serial.print(F("[IRCODE SENSE] REPEATING IR code: "));
        	Serial.println(tgtCodeValue, HEX);
            sendCode(0, tgtCodeType, tgtCodeLen, tgtCodeValue); //send NEC repeat
        } 
        //if no repeat, use translation table to send the right code
        else {
            _foundIRCodeIndex = findIRCodeIndex(rcvCodeValue);
            if (_foundIRCodeIndex != -1) {
                tgtCodeValue = ((unsigned long)irCodes[_foundIRCodeIndex][IRCODES_TGT_CODE0]) |
                            ((unsigned long)irCodes[_foundIRCodeIndex][IRCODES_TGT_CODE1] << 8) |
                            ((unsigned long)irCodes[_foundIRCodeIndex][IRCODES_TGT_CODE2] << 16) |
                            ((unsigned long)irCodes[_foundIRCodeIndex][IRCODES_TGT_CODE3] << 24);
                tgtCodeType = irCodes[_foundIRCodeIndex][IRCODES_TGT_CODETYPE];
                tgtCodeLen = irCodes[_foundIRCodeIndex][IRCODES_TGT_CODELEN];
                Serial.print(F("[IRCODE SENSE] IR code translation found with index: "));
                Serial.println(_foundIRCodeIndex, DEC);
                Serial.print(F("[IRCODE SENSE] sending translated IR code: "));
                Serial.println(tgtCodeValue, HEX);

                //send code and resume operation
                sendCode(0, tgtCodeType, tgtCodeLen, tgtCodeValue); //send recorded or saved IR code without repeat
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
    disableInterrupt(BUTTON_RECORD_PIN);
    disableInterrupt(BUTTON_RESET_PIN);
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
    rcvCodeValue, rcvCodeLen, rcvCodeType = 255; // The type of code, 255 is unsigned equivalent of -1
    irCodeRecordingStatus = 0;

    //enable button interrupts. record button will now cancel the recording
    //and the reset button will reset the codes
    ircode_record_detected = 0;
    ircode_reset_detected = 0;
    enableInterrupt(BUTTON_RECORD_PIN, on_ircode_record_irq, RISING);
    enableInterrupt(BUTTON_RESET_PIN, on_ircode_reset_irq, RISING);

    if (DEBUG_LEVEL) Serial.println(F("[IRCODE RECORD] Start. Enabling IR receiver"));
}

/**
    waits until IR remote sends the IR code
*/
void on_ircode_record_loop() {
	//signal recording status with the status LED
	if(irCodeRecordingStatus == 0) blink(LED_STATUS_PIN, 1, 250);
	if(irCodeRecordingStatus == 1) blink(LED_STATUS_PIN, 1, 800);
	
    // button to cancel ircode recording
    if (ircode_record_detected == 1) {
        delay(250); //avoid bounce
        if (DEBUG_LEVEL) Serial.println(F("[IRCODE RECORD] IRCode recording canceled"));
        ircode_record_detected = 0;
        fsm.trigger(TRIGGER_IRCODE_CANCELED);
    }

    // button to perform reset but only before first code was recorded
    if (ircode_reset_detected == 1 && irCodeRecordingStatus == 0) {
        delay(250); //avoid bounce
        if (DEBUG_LEVEL) Serial.println(F("[IRCODE RECORD] reset detected"));
        blink(LED_STATUS_PIN, 3, 300);
        resetIRCodes();
        ircode_reset_detected = 0;
    }


    //read and decode the src code
    if (irrecv.decode(&results) && irCodeRecordingStatus == 0) {
        storeCode(&results, rcvCodeType, rcvCodeLen, rcvCodeValue);
        if (DEBUG_LEVEL) { Serial.print(F("[IRCODE RECORD][1/2] - SRC ircode captured with value: ")); Serial.println(rcvCodeValue, HEX); }
        irCodeRecordingStatus++;

        //check if code exists already
        codeIndex = findIRCodeIndex(rcvCodeValue);
        if (codeIndex != -1){
        	if (DEBUG_LEVEL) {
        		Serial.print(F("[IRCODE RECORD] existing code found with index ["));
        		Serial.print(codeIndex, DEC);
        		Serial.println(F("], overwriting code mapping"));
    		}
        } 

        //check if need to find next free slot
        if (codeIndex == -1){
        	codeIndex = findFreeIRCodeIndex();
        	if (DEBUG_LEVEL == 2 && codeIndex != -1) { Serial.print(F("[IRCODE RECORD] next available recording slot found [")); Serial.print(codeIndex, DEC); Serial.println(F("]")); }
        } 

        //check if index available
        if (codeIndex == -1) {
        	Serial.println(F("[IRCODE RECORD] no free slots found, exiting"));

			//blink stored LED
			blink(LED_STORED_PIN, 5, 400);
        	        	
            fsm.trigger(TRIGGER_IRCODE_RECORDED);
            return;
        }

        //store src code in the ircodes table
        irCodes[codeIndex][IRCODES_SRC_CODE0] = rcvCodeValue;
        irCodes[codeIndex][IRCODES_SRC_CODE1] = rcvCodeValue >> 8;
        irCodes[codeIndex][IRCODES_SRC_CODE2] = rcvCodeValue >> 16;
        irCodes[codeIndex][IRCODES_SRC_CODE3] = rcvCodeValue >> 24;

        irrecv.resume(); //resume recording for the stop code
    }

    //read and decode the tgt code. slot must have been activated by now
    if (irrecv.decode(&results) && irCodeRecordingStatus == 1) {
        storeCode(&results, rcvCodeType, rcvCodeLen, rcvCodeValue);
        if (DEBUG_LEVEL) { Serial.print(F("[IRCODE RECORD][2/2] - TGT ircode captured with value: ")); Serial.println(rcvCodeValue, HEX) }
        irCodeRecordingStatus++;

        //put target codes in the ircode rable
        irCodes[codeIndex][IRCODES_TGT_CODE0] = rcvCodeValue;
        irCodes[codeIndex][IRCODES_TGT_CODE1] = rcvCodeValue >> 8;
        irCodes[codeIndex][IRCODES_TGT_CODE2] = rcvCodeValue >> 16;
        irCodes[codeIndex][IRCODES_TGT_CODE3] = rcvCodeValue >> 24;
        irCodes[codeIndex][IRCODES_TGT_CODELEN] = rcvCodeLen;
        irCodes[codeIndex][IRCODES_TGT_CODETYPE] = rcvCodeType;

        //save results to EEPROM and show codes
        unsigned long timeStamp = millis();
        Serial.print(F("[IRCODE RECORD] Saving ircodes back to EEPROM..."));
        saveIRCodesToEEPROM();
        Serial.print(F(" done, took "));
        Serial.print(millis() - timeStamp, DEC);
        Serial.println(" milliseconds");
        Serial.print(F("[IRCODE RECORD] Readong ircodes back from EEPROM..."));
        timeStamp = millis();
        getIRCodesFromEEPROM();
        Serial.print(F(" done, took "));
        Serial.print(millis() - timeStamp, DEC);
        Serial.println(" milliseconds");
        printIRCodesSerial();

        //blink and turn STORED LED on
        blink(LED_STORED_PIN, 3, 300);
        digitalWrite(LED_STORED_PIN, HIGH);

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
    disableInterrupt(BUTTON_RESET_PIN);
}

/*  ==================================================================================================================
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
    pin has pullup enabled and the button shorts the pin to GND
    interrupt must be enabled on the raising edge
*/
void on_ircode_record_irq() {
    ircode_record_detected = 1;
}

/**
    interrupt handler for ircode reset button
	pin has pullup enabled and the button shorts the pin to GND
    interrupt must be enabled on the raising edge
*/
void on_ircode_reset_irq() {
    ircode_reset_detected = 1;
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
    if(initialState != -1) _initialState = digitalRead(pin);
    else _initialState = initialState;
    
    for (unsigned short i = 0; i < cycles * 2; i++) {
        digitalWrite(pin, ~(i + _initialState) & 1);
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


/**
    reset all codes and write them to eeprom
*/
void resetIRCodes() {
    int c = sizeof(irCodes[0]) / sizeof(irCodes[0][0]); //number of columns
    Serial.println("[RESET] resetting all IR codes and writing them to EEPROM");
    for (int i = 0; i < IRCODES_NUMBER; i++) for (int j = 0; j < c; j++) irCodes[i][j] = 0;
    saveIRCodesToEEPROM();
    printIRCodesSerial();

	//turn ready led off
    digitalWrite(LED_STORED_PIN, LOW);
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

    //reinitialise values
    _index = -1;
    _findCodeValue = 0;

    //find the tgt code in the saved codes table
    for (int i = 0; i < IRCODES_NUMBER; i++) {
        unsigned long srcCodeValue = ((unsigned long)irCodes[i][IRCODES_SRC_CODE0]) | ((unsigned long)irCodes[i][IRCODES_SRC_CODE1] << 8) | ((unsigned long)irCodes[i][IRCODES_SRC_CODE2] << 16) | ((unsigned long)irCodes[i][IRCODES_SRC_CODE3] << 24);
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
        unsigned long srcCodeValue = ((unsigned long)irCodes[i][IRCODES_SRC_CODE0]) | ((unsigned long)irCodes[i][IRCODES_SRC_CODE1] << 8) | ((unsigned long)irCodes[i][IRCODES_SRC_CODE2] << 16) | ((unsigned long)irCodes[i][IRCODES_SRC_CODE3] << 24);
        if ( srcCodeValue == 0 || srcCodeValue == 0xff ) {
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
        unsigned long srcCodeValue = ((unsigned long)irCodes[i][IRCODES_SRC_CODE0]) | ((unsigned long)irCodes[i][IRCODES_SRC_CODE1] << 8) | ((unsigned long)irCodes[i][IRCODES_SRC_CODE2] << 16) | ((unsigned long)irCodes[i][IRCODES_SRC_CODE3] << 24);
        unsigned long tgtCodeValue = ((unsigned long)irCodes[i][IRCODES_TGT_CODE0]) | ((unsigned long)irCodes[i][IRCODES_TGT_CODE1] << 8) | ((unsigned long)irCodes[i][IRCODES_TGT_CODE2] << 16) | ((unsigned long)irCodes[i][IRCODES_TGT_CODE3] << 24);
        Serial.print(F("IR Codes ["));
        Serial.print(i + 1, DEC);
        Serial.print(F("]: "));
        Serial.print(srcCodeValue, HEX);
        Serial.print(" -> ");
        Serial.println(tgtCodeValue, HEX);
    }
}
