/*
 * Audio detector that listens for an audio signal with the SPDIF TOSLINK or COAX receiver and controls audio components with the 12V trigger and IR signal
 * The board used in the project is the Arduino Nano and the libraries:
 * State Machine was implemented with *arduino_fsm* state machine library https://github.com/jonblack/arduino-fsm
 * IR code recording and sending was implemented with *IRRemote* library https://github.com/z3t0/Arduino-IRremote
 * 
 * Update on 26 Mar 2020: code has been tested and all works with NAD326bee amplifier and Yamaha WXAD-10 streamer
 * 
 * Copyright by Stellars Henson 2020
 */

#include <IRremote.h>
#include <EEPROM.h>
#include <Fsm.h>
#include <EnableInterrupt.h>

#define SEND_PIN 3
#define RECV_PIN 2
#define IRCODE_RECORD_PIN 12
#define STATUS_LED_PIN 4
#define IRCODE_STORED_LED_PIN 10
#define IRCODE_RECORD_LED_PIN 9
#define AUDIOSENSE_DIGITAL_PIN 8 //output from the detector circuit. MK1 has it as a SPDIF decoder serial output
#define AUDIOTRIGGER_PIN 7 //connected to the optocoupler that detects the 12V trigger from the amp
#define AUDIOTRIGGER_LED_PIN 6 //lights the LED if the amp is up and 12V trigger is high

#define TRIGGER_IRCODE_RECORD 1
#define TRIGGER_IRCODE_RECORDED 2
#define TRIGGER_AUDIO_DETECTED 3
#define TRIGGER_AUDIO_ENABLED 4
#define TRIGGER_AUDIO_DISABLED 5

#define DEBUG_LEVEL 2 //0 - debug off, 1 - essential messages, 2 - full diagnostics
#define AUDIO_START_TIMEOUT 10000 //timeout for the audio start detection
#define AUDIO_STOP_TIMEOUT 10*60000 //timeout for the audio shutdown if no signal

//IR receiver setup
IRrecv irrecv(RECV_PIN);
IRsend irsend;
decode_results results;

// Storage for the recorded code
void sendCode(uint8_t repeat, uint8_t codeType, uint8_t codeLen, unsigned long codeValue);
void storeCode(decode_results *results, int8_t &codeType, uint8_t &codeLen, unsigned long &codeValue);


uint8_t startAudioCodeType = 255; // The type of code
uint8_t startAudioCodeLen; // The length of the code
unsigned long startAudioCodeValue; // The code value if not raw

uint8_t stopAudioCodeType = 255; // The type of code
uint8_t stopAudioCodeLen; // The length of the code
uint32_t stopAudioCodeValue; // The code value if not raw

uint8_t toggle = 0; // The RC5/6 toggle state
uint16_t rawCodes[RAWBUF]; // The durations if raw
uint8_t irCodesAvailable = 0; //if ircodes were recorded

uint8_t audioTriggerAvailable = 0; //if 12V trigger is available. Audio enable timeout sets this to 0, detection of the 12V trigger sets this to 1. We can also detect this with an audio jack (it has detect input feature)

//interrupt driver
void on_audiosense_digital_irq();
void on_ircode_record_irq();
volatile uint8_t audiosense_digital_detected = 0;
volatile uint8_t ircode_record_detected = 0;



//FSM - set up of the state machine
void on_ircode_record_enter(); 
void on_ircode_record_loop();
void on_ircode_record_exit();
void on_audio_sense_enter();
void on_audio_sense_loop();
void on_audio_sense_exit();
void on_audio_start_enter();
void on_audio_start_loop();
void on_audio_start_exit();
void on_audio_enabled_enter();
void on_audio_enabled_loop();
void on_audio_enabled_exit();
void on_audio_start_timed_trans_audio_enabled();

State state_audio_sense(on_audio_sense_enter, on_audio_sense_loop, on_audio_sense_exit);
State state_ircode_record(on_ircode_record_enter, on_ircode_record_loop, on_ircode_record_exit);
State state_audio_start(on_audio_start_enter, on_audio_start_loop, on_audio_start_exit);
State state_audio_enabled(on_audio_enabled_enter, on_audio_enabled_loop, on_audio_enabled_exit);
Fsm   fsm(&state_audio_sense);

void setup()
{
  Serial.begin(9600);
  
  pinMode(IRCODE_RECORD_PIN, INPUT_PULLUP);
  pinMode(AUDIOSENSE_DIGITAL_PIN, INPUT_PULLUP);
  pinMode(AUDIOTRIGGER_PIN, INPUT_PULLUP);
  pinMode(AUDIOTRIGGER_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(IRCODE_STORED_LED_PIN, OUTPUT);
  pinMode(IRCODE_RECORD_LED_PIN, OUTPUT);

  //read ircode from EEPROM
  startAudioCodeValue = ( (unsigned long)EEPROM.read(0)) | ( (unsigned long)EEPROM.read(1)<<8) | ((unsigned long) EEPROM.read(2)<<16) | ( (unsigned long)EEPROM.read(3)<<24);
  startAudioCodeLen = EEPROM.read(4);
  startAudioCodeType = EEPROM.read(5);

  stopAudioCodeValue = ( (unsigned long)EEPROM.read(6)) | ( (unsigned long)EEPROM.read(7)<<8) | ((unsigned long) EEPROM.read(8)<<16) | ( (unsigned long)EEPROM.read(9)<<24);
  stopAudioCodeLen = EEPROM.read(10);
  stopAudioCodeType = EEPROM.read(11);

  //light "ircode stored" LED only if both start and stop codes were retrieved
  if(startAudioCodeType != 255 && stopAudioCodeType != 255) { 
    digitalWrite(IRCODE_STORED_LED_PIN, HIGH);
    irCodesAvailable = 1;
  }
  
  if(DEBUG_LEVEL == 2) Serial.print("[INIT] Restoring IR codes, AUDIO START: ");
  if(DEBUG_LEVEL == 2) Serial.print(startAudioCodeValue, HEX);
  if(DEBUG_LEVEL == 2) Serial.print(" , AUDIO STOP: ");
  if(DEBUG_LEVEL == 2) Serial.println(stopAudioCodeValue, HEX);
  if(DEBUG_LEVEL && irCodesAvailable != 1) Serial.println("[INIT] IR codes not available");

  //initialise state machine
  fsm.add_transition(&state_audio_sense, &state_ircode_record, TRIGGER_IRCODE_RECORD, NULL);
  fsm.add_transition(&state_ircode_record, &state_audio_sense, TRIGGER_IRCODE_RECORDED, NULL);
  fsm.add_transition(&state_audio_sense, &state_audio_start, TRIGGER_AUDIO_DETECTED, NULL);
  fsm.add_transition(&state_audio_start, &state_audio_enabled, TRIGGER_AUDIO_ENABLED, NULL);
  fsm.add_transition(&state_audio_enabled, &state_audio_sense, TRIGGER_AUDIO_DISABLED, NULL);
  fsm.add_timed_transition(&state_audio_start, &state_audio_enabled, AUDIO_START_TIMEOUT, on_audio_start_timed_trans_audio_enabled);
  fsm.add_timed_transition(&state_audio_enabled, &state_audio_sense, AUDIO_STOP_TIMEOUT, on_audio_enabled_timed_trans_audio_sense);
  
  
}

void loop() {
  fsm.run_machine(); //run state loops
  delay(10);
}


// *******************************
// STATE MACHINE 
// *******************************


// STATE_AUDIO_SENSE

/**
* only announces the transition
*/
void on_audio_sense_enter() {
  if(DEBUG_LEVEL) Serial.println("[AUDIO SENSE] Start. Listening for audio and ircode record button");  
  
  //interrupt initialisation
  audiosense_digital_detected = 0;
  ircode_record_detected = 0;
  enableInterrupt(AUDIOSENSE_DIGITAL_PIN, on_audiosense_digital_irq, CHANGE);
  enableInterrupt(IRCODE_RECORD_PIN, on_ircode_record_irq, CHANGE);
}

/**
* waits for either ircode record button trigger or for audio signal
* ircode button -> goes to ircode recording
* audiosense signal -> goes to start audio
* audio and ircode button detection is driven by interrupts
*/
void on_audio_sense_loop() {
  //blink status 2hz to indicate listening for audio
  blink(STATUS_LED_PIN, 1, 1000);

  // button to enable recording ircode
  // driven by interrupts now on IRCODE_RECORD_PIN
  if (ircode_record_detected == 1) {
    if(DEBUG_LEVEL) Serial.println("[AUDIO SENSE] IRCode recording detected");
    blink(STATUS_LED_PIN, 3, 300);
    fsm.trigger(TRIGGER_IRCODE_RECORD);
  } 

  // optocoupler connected to GND and AUDIOSENSE_PIN
  // this is detected with an interrupt on AUDIOSENSE_PIN
  if (audiosense_digital_detected == 1) {
    if(DEBUG_LEVEL) Serial.println("[AUDIO SENSE] Audio signal detected");
    blink(STATUS_LED_PIN, 3, 300);
    fsm.trigger(TRIGGER_AUDIO_DETECTED);
  }
}

/**
* signals exit from the state
*/
void on_audio_sense_exit() {
  if(DEBUG_LEVEL == 2) Serial.println("[AUDIO SENSE] Exit. Disabling audio signal detection");  
  disableInterrupt(AUDIOSENSE_DIGITAL_PIN);
  disableInterrupt(IRCODE_RECORD_PIN);
}


// IRCODE RECORD STATE

/**
* enables IR receiver and turns recording LED on
* it also resets codes recorded previously
*/
void on_ircode_record_enter() {
  // enable receiver
  irrecv.enableIRIn(); 
  
  //reset stored codes to allow saving new ones
  digitalWrite(IRCODE_STORED_LED_PIN, LOW);
  digitalWrite(IRCODE_RECORD_LED_PIN, HIGH);
  if(DEBUG_LEVEL) Serial.println("[IRCODE RECORD] Start.  Enabling IR receiver");

  //reset all previosu codes
  startAudioCodeValue = 0;
  startAudioCodeType = -1;
  stopAudioCodeValue = 0;
  stopAudioCodeType = -1;
}

/**
* waits until IR remote sends the IR code 
* - records audio start code and saves to EEPROM and waits for recoding of the audio stop
* - records audio stop code and saves to EEPROM
* after this goes back to audio sense state
* 
* the STORED_LED will be flashing with 1HZ when only one code was recorded and will 
* be turned on permanently when all codes were recorded
*/
void on_ircode_record_loop() {

  //blink pattern for recording: twice for the first code, once for the second code
  if(startAudioCodeType == 255 && stopAudioCodeType == 255) { blink(IRCODE_RECORD_LED_PIN, 2, 500); delay(500);  }
  if(startAudioCodeType != 255 && stopAudioCodeType == 255) { blink(IRCODE_RECORD_LED_PIN, 1, 500); delay(500);  }
  
  //read and decode the ircode, ircode was received with the interrupts
  if (irrecv.decode(&results)) {
    //record start audio code
    if( startAudioCodeType == 255 ) {
      if(DEBUG_LEVEL) Serial.println("[IRCODE RECORD][1/2] - AUDIO START ircode detected");
      blink(STATUS_LED_PIN, 3, 300);
      storeCode(&results, startAudioCodeType, startAudioCodeLen, startAudioCodeValue);
      
      //write the to eeprom and report on serial
      EEPROM.update(0, startAudioCodeValue);
      EEPROM.update(1, startAudioCodeValue >> 8);
      EEPROM.update(2, startAudioCodeValue >> 16);
      EEPROM.update(3, startAudioCodeValue >> 24);
      EEPROM.update(4, startAudioCodeLen);
      EEPROM.update(5, startAudioCodeType);
      if(DEBUG_LEVEL == 2) Serial.print("[IRCODE RECORD][1/2] Saved value: ");
      if(DEBUG_LEVEL) Serial.println(startAudioCodeValue, HEX);
      if(DEBUG_LEVEL == 2) Serial.println("[IRCODE RECORD][1/2] Waiting for AUDIO STOP ircode");

      irrecv.resume(); //resume recording for the stop code
    } 
    //record stop audio code
    else if( startAudioCodeType != 255 && stopAudioCodeType == 255 ) {
      if(DEBUG_LEVEL) Serial.println("[IRCODE RECORD][2/2] AUDIO STOP ircode detected");
      blink(STATUS_LED_PIN, 3, 300);
      storeCode(&results, stopAudioCodeType, stopAudioCodeLen, stopAudioCodeValue);
  
      //write the to eeprom and report on serial
      EEPROM.update(6, stopAudioCodeValue);
      EEPROM.update(7, stopAudioCodeValue >> 8);
      EEPROM.update(8, stopAudioCodeValue >> 16);
      EEPROM.update(9, stopAudioCodeValue >> 24);
      EEPROM.update(10, stopAudioCodeLen);
      EEPROM.update(11, stopAudioCodeType);
      if(DEBUG_LEVEL == 2) Serial.print("[IRCODE RECORD][2/2] Saved value: ");
      if(DEBUG_LEVEL) Serial.println(stopAudioCodeValue, HEX);

      //trigger transition
      fsm.trigger(TRIGGER_IRCODE_RECORDED);  //trigger transition only after audio stop code was recorded
    }    
  }
}

/**
* announces exit from ircode recording state and turns off recording LED
*/
void on_ircode_record_exit() {
  if(DEBUG_LEVEL == 2) Serial.println("[IRCODE RECORD] Exit. Disabling receiver");
  digitalWrite(IRCODE_RECORD_LED_PIN, LOW);
  digitalWrite(IRCODE_STORED_LED_PIN, HIGH);
}

// AUDIO START STATE

/**
* sends recorded IR code on enter
*/
void on_audio_start_enter() {
  Serial.println("[AUDIO START] Start. Sending AUDIO START IR code and monitoring");

  //initialise trigger sense
  audioTriggerAvailable = 0;

  //send audio start IR signal
  sendCode(0, startAudioCodeType, startAudioCodeLen, startAudioCodeValue); //send recorded or saved IR code without repeat
}

/**
* waits for the external 12V trigger (typically output from an amplifier
* MK2 - if the external 12V trigger was not detected in AUDIOTRIGGER_TIMEOUT, system enables its own 12V trigger relay
* once 12V trigger was enabled or detected, move to AUDIO_ENABLED state
*/
void on_audio_start_loop() {
  uint8_t audioTriggerState = digitalRead(AUDIOTRIGGER_PIN);  //read the status of the external 12V trigger via optocoupler circuit connected to GND

  //blink 2hz to indicate waiting for audio enabled
  blink(AUDIOTRIGGER_LED_PIN, 1, 500);

  //detecting status of the 12v trigger line 
  //it stays this way, we are not trying to detect the signal, just the state
  if (audioTriggerState == LOW) {
    if(DEBUG_LEVEL) Serial.println("[AUDIO START] Audio trigger detected");
    blink(STATUS_LED_PIN, 3, 250); // blink status led that the audio is enabled
    digitalWrite(AUDIOTRIGGER_LED_PIN, HIGH); //indicate that the audio is enabled
    audioTriggerAvailable = 1; //12V audio trigger is available, no need for timeout
    fsm.trigger(TRIGGER_AUDIO_ENABLED); //initiate state transition
  }
}

/**
* announce exit from the audio start state
*/
void on_audio_start_exit() {
  if(DEBUG_LEVEL == 2) Serial.println("[AUDIO START] Exit. Audio is on");
}

/**
* timed transition to audio enabled state after AUDIO_START_TIMEOUT (in miliseconds)
*/
void on_audio_start_timed_trans_audio_enabled() {
  if(DEBUG_LEVEL) Serial.println("[AUDIO START] Timeout. Trigger not available");
  audioTriggerAvailable = 0; //12V audio trigger is not available
}

// AUDIO ENABLED STATE

/**
* announce enter audio enabled state
*/
void on_audio_enabled_enter() {
  if(DEBUG_LEVEL) Serial.println("[AUDIO ENABLED] Start. Monitoring for standby");
  
  //if audiotrigger is not available, enable interrupts to listen to audio signal
  if(audioTriggerAvailable == 0) {
    audiosense_digital_detected = 0;
    enableInterrupt(AUDIOSENSE_DIGITAL_PIN, on_audiosense_digital_irq, CHANGE);
    if(DEBUG_LEVEL == 2) { Serial.print("[AUDIO ENABLED] Trigger not available. Enabling audio signal timeout for "); Serial.print(AUDIO_STOP_TIMEOUT / 60000, DEC); Serial.println("min"); }
  }
}

/**
* listens for the 12V trigger to go LOW (AUDIOTRIGGER_PIN will go up becasue of optocoupler connected to GND and the pullup sense pin)
* when 12V LOW state is detected, it indicates that the amp is in standby. System goes back to AUDIO SENSE state
* 
* if 12V trigger is not available, system refreshes  the timer with the audio signal detection
*/
void on_audio_enabled_loop() {
  //listen for the trigger only if available
  if(audioTriggerAvailable == 1) {
    //read 12V audio trigger from the optocoupler connected to GND (inverts the signal)
    uint8_t audioTriggerState = digitalRead(AUDIOTRIGGER_PIN);  //read the status of the external 12V trigger via optocoupler circuit connected to GND 
  
    //if audiotrigger is available we await the 12V trigger to go LOW 
    //AUDIOTRIGGER_PIN goes HIGH inverted by the optocoupler
    if (audioTriggerState == HIGH) {
      if(DEBUG_LEVEL) Serial.println("[AUDIO ENABLED] Standby trigger detected");
      blink(STATUS_LED_PIN, 3, 300); // blink status led that the audio is enabled
      fsm.trigger(TRIGGER_AUDIO_DISABLED); //initiate state transition
    }
  } else if(audioTriggerAvailable == 0 && audiosense_digital_detected == 1) {
    //if signal was detected - reset timer and the signal
    if(DEBUG_LEVEL == 2) Serial.println("[AUDIO ENABLED] Audio detected. Timer reset");
    fsm.reset_timed_transition(&state_audio_sense);
    audiosense_digital_detected = 0;
  }
  
  //initiates long idle for powersaving. 
  //blink once for normal trigger, twice for no trigger (timeout driven)
  if(audioTriggerAvailable) blink(STATUS_LED_PIN, 1, 250);
  else blink(STATUS_LED_PIN, 2, 250);
  delay(1000);
}

/**
* announce leaving the audio enabled state
*/
void on_audio_enabled_exit() {
  if(DEBUG_LEVEL == 2) Serial.println("[AUDIO ENABLED] Exit. Sending AUDIO STOP ircode");

  //send AUDIO STOP IR signal
  sendCode(0, stopAudioCodeType, stopAudioCodeLen, stopAudioCodeValue); //send recorded or saved IR code without repeat
  digitalWrite(AUDIOTRIGGER_LED_PIN, LOW); //indicate that the audio is disabled

  //disable audio sense interrupt
  if(DEBUG_LEVEL == 2 && audioTriggerAvailable == 0) Serial.println("[AUDIO ENABLED] Disabling shutdown timer");
  disableInterrupt(AUDIOSENSE_DIGITAL_PIN);
}


/**
* timed transition to audio sense
*/
void on_audio_enabled_timed_trans_audio_sense() {
  if(DEBUG_LEVEL) Serial.println("[AUDIO ENABLED] Timeout. No audio signal");
}


/** 
* Stores the code for later playback
* Most of this code is just logging 
* 
* @param results - pointer to the structure with the results decoded
* @param codeType - type of the IR code (NEC, SONY etc..)
* @codeLen - length of the code in bytes
* @codeValue - 4 bytes long code value
*/
void storeCode(decode_results *results, uint8_t &codeType, uint8_t &codeLen, unsigned long &codeValue) {
  codeType = results->decode_type;
  uint8_t count = results->rawlen;
  if (codeType == UNKNOWN) {
    if(DEBUG_LEVEL) Serial.println("Received unknown code, saving as raw");
    codeLen = results->rawlen - 1;
    // To store raw codes:
    // Drop first value (gap)
    // Convert from ticks to microseconds
    // Tweak marks shorter, and spaces longer to cancel out IR receiver distortion
    for (int i = 1; i <= codeLen; i++) {
      if (i % 2) {
        // Mark
        rawCodes[i - 1] = results->rawbuf[i]*USECPERTICK - MARK_EXCESS;
        if(DEBUG_LEVEL) Serial.print(" m");
      } 
      else {
        // Space
        rawCodes[i - 1] = results->rawbuf[i]*USECPERTICK + MARK_EXCESS;
        if(DEBUG_LEVEL) Serial.print(" s");
      }
      if(DEBUG_LEVEL) Serial.print(rawCodes[i - 1], DEC);
    }
    if(DEBUG_LEVEL) Serial.println("");
  }
  else {
    if (codeType == NEC) {
      if(DEBUG_LEVEL == 2) Serial.print("Received NEC: ");
      if (results->value == REPEAT) {
        // Don't record a NEC repeat value as that's useless.
        if(DEBUG_LEVEL == 2) Serial.println("repeat; ignoring.");
        return;
      }
    } 
    else if (codeType == SONY) {
      if(DEBUG_LEVEL == 2) Serial.print("Received SONY: ");
    } 
    else {
      if(DEBUG_LEVEL) Serial.print("Unexpected");
      if(DEBUG_LEVEL) Serial.print(codeType, DEC);
      if(DEBUG_LEVEL) Serial.println("");
    }
    if(DEBUG_LEVEL == 2) Serial.println(results->value, HEX);
    codeValue = results->value;
    codeLen = results->bits;
  }
}

/**
* sends the IR code recorded in the global variables: codeType, codeValue, codeLen
* the IR code is sent using PWM pin and IR LED
* 
*/
void sendCode(uint8_t repeat, uint8_t codeType, uint8_t codeLen, unsigned long codeValue) {
  if (codeType == NEC) {
    if (repeat) {
      irsend.sendNEC(REPEAT, codeLen);
      if(DEBUG_LEVEL) Serial.println("Sent NEC repeat");
    } 
    else {
      irsend.sendNEC(codeValue, codeLen);
      if(DEBUG_LEVEL) Serial.print("Sent NEC ");
      if(DEBUG_LEVEL) Serial.println(codeValue, HEX);
    }
  } 
  else if (codeType == SONY) {
    irsend.sendSony(codeValue, codeLen);
    if(DEBUG_LEVEL) Serial.print("Sent Sony ");
    if(DEBUG_LEVEL) Serial.println(codeValue, HEX);
  } 
  else if (codeType == UNKNOWN /* i.e. raw */) {
    // Assume 38 KHz
    irsend.sendRaw(rawCodes, codeLen, 38);
    if(DEBUG_LEVEL) Serial.println("Sent raw");
  }
}

/**
 * interrupt handler for audio signal detection
*/
void on_audiosense_digital_irq() {
  audiosense_digital_detected = 1;
}

/**
* interrupt handler for ircode record button
*/
void on_ircode_record_irq() {
  ircode_record_detected = 1;  
}

/**
* blinks pin with a led #cycles number of times, each within the duration
*/
static void  blink( const byte pin, const byte cycles, const unsigned int duration){
  uint8_t initialState = digitalRead(pin);
  for(unsigned short i=0; i<cycles * 2; i++){
    digitalWrite(pin, ~(i+initialState) & 1);
    delay(duration / 2);
  }
}
