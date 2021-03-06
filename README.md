# irremote-translator
Make your TV IR remote control other devices (i.e. Audio Amplifier) equipped with the 3.5mm IR extender port. Imagine you can turn the volume up or down on your TV remote and the amplifier will have the volume adjusted accordingly. This project is an alternative to the universal remote (that some people - like me - don't like very much). You can record up to 32 codes.

## Installation and Use ##
Device translates the IR remote codes on-the-fly to the previosuly recorded codes and sends mapped IR codes over the 3.5mm jack IR repeater port. Device has buttons and LEDs to control its operation

* IR phototransistor on pin D2
* IR transmitter on pin D3
* POWER LED (no pin) - indicates that the device is powered
* STATUS LED on D4 - red, blinks when device performs some operation (i.e. recording of the IR codes)
* STORAGE LED on D5 - green, turned on when there are ir codes recorded, off when no codes recorded, blinks when cannot record codes (slots used up)
* RECORD BUTTON on D12 - when pressed initiates 2-stage recording session. When pressed during recording - cancels recording
* RESET BUTTON on D11 - when pressed resets the ir codes previously recorded and frees up all slots

To use the device you'd need to supply power with the 5v miniUSB cable and connect the 3.5mm output port using audio 3.5mm cable to the IR remote repeater input port of your audio equipment

### Initial Setup ###
When powered, system indicates if any pre-recorded IR codes were found by lighting up the STORAGE LED. When STORAGE LED is off - no codes were previously recorded.
To record the code mapping follow the instructions below:

1. turn the device on, STORAGE LED is off when POWER LED is on
1. press RECORD BUTTON once, device will enter recording mode and blink STATUS LED rapidly
1. press your TV remote button, that you'd like to control your other device with, pointing the remote at the device IR receiver. When code was captured te STATUS LED will blink much slower 
1. press your Audio Amplifier remote button to map the remote code to the tv remote code used in the previous step. STATUS LED should stop blinkng and STORAGE LED should be turned on

*Example:* to allow the volume of the audio amplifier to be changed when the TV audio volume is adjusted you'd record first the VOLUME UP tv remote code and next the VOLIME UP of the audio amplifier remote. Next you'd record the VOLUME DOWN the same way. After the codes were recorded the device will be able to translate the tv codes to amplifier ir codes immediately. 

### Typical Use ###
Once connected and when the codes were recorded - connect the device to the Audio Amplifier with the 3.5mm audio jack and put the device on top of the amplifier with the receiver end of the device pointing in your direction. Connect the device to the power source and observe that the POWER LED is on and STORAGE LED is on.

Device is ready for operation.

Now, whenever you use your TV remote and had the codes recorded (i.e. VOLUME UP and VOLUME DOWN), your amplifier will respond to the TV remote operation

## Arduino ##
The board used is the Arduino Nano and the libraries:
* State Machine was implemented with *arduino_fsm* state machine library https://github.com/jonblack/arduino-fsm. This project however uses my fork of the library to allow timed transitions reset: https://github.com/stellarshenson/arduino-fsm. Pull was already requested from the *arduino-fsm* owner, hopefully my fork will be merged soon
* IR code recording and sending was implemented with *IRRemote* library https://github.com/z3t0/Arduino-IRremote

## State Machine ##
Diagram was generated by http://asciiflow.com/

	             +----------------------+
	             | state_ircodes_record |
	             +---------+--------+---+
	                       ^        |
	TRIGGER_IRCODES_RECORD |        | TRIGGER_IRCODES_RECORDED
	                       |        v
	             +---------+--------+--+
	             | state_ircode_sense  +
	             +---------------------+
	                           

### state_ircode_sense ###
Listens for three types of events:

1. RECORD BUTTON press, when detected starts the IR code record session
1. RESET BUTTON press, when detected resets the recorded ir codes (i.e. to free all slots)
1. IR Code, when detected looks up the mapped translation code and sends using IR transmitter

### state_ircode_record ###
Listens for three types of events:

1. RECORD BUTTON press, when detected cancels the recording session and goes back to the previous state
1. RESET BUTTON press, when detected resets the recorded ir codes but stays in the recording session. Reset is possible only when recording did not start yet
1. IR Code, when detected the device looks for the previously recorded slot (to overwrite) or for a new slot (if not recorded the code yet). Device records the source IR code and listens for another IR code and maps it to the previous one and returns to the previous state

## Circuit and PCB ##
![breadboard](https://github.com/stellarshenson/irremote-translator/blob/master/irremote-translator_bb.jpg)
![schematics](https://github.com/stellarshenson/irremote-translator/blob/master/irremote-translator_sch.jpg)
![pcb all](https://raw.githubusercontent.com/stellarshenson/irremote-translator/master/irremote-translator_pcb_all.jpg)
![pcb top](https://raw.githubusercontent.com/stellarshenson/irremote-translator/master/irremote-translator_pcb_top_400dpi.jpg)
![pcb btm](https://raw.githubusercontent.com/stellarshenson/irremote-translator/master/irremote-translator_pcb_btm_400dpi.jpg)
![assembled breadboard](https://raw.githubusercontent.com/stellarshenson/irremote-translator/master/images/IMG_6503.jpg)


