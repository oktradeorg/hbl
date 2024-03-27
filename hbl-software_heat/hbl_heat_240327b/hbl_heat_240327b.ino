/*

PURPOSE - HEATBIT BASIC CODE 
2024/01/19

The Haptic Bits Lab includes a library of shapes and actuators to facilitate somaesthetic design —  
a process that allows designers to examine and improve on connections between sensation, feeling, 
emotion, subjective understanding and values.

This program enables recording, and playback, of MIDI signal from a set of potentiometers and to 
heat pads with the corresponding circuitry defined in the Haptic Bits Lab documentation.

The two potentiometers next to the pressure pad controls inputs mapped to the heatpads, with each
potentiometer mapped to one heatpad, output voltage is sent to the ports for the heatpads. 
Turning the dials clockwise increases the voltage, up to a maximum of 5V based on the board design. 

The board is also equipped with a pressure pad but it is currently not in use. However it is connected 
and ready to have new functions implemented via code. Next to the 'Arduino Nano Every' card on the 
board, a number of pins are available for connecting breakout boards, sensors, or actuators in order
to enable new interactions and functionalities.

A rotary switch on the far right moves the current active mode between recording, off, and playback. 
An RGB LED indicates the modes as red (record), off, and green (playback).

*/


// Pin definition for input rotary potentiometers and pressure pad
#define input_1_dial A0 // Rotary potentiometer 1.
#define input_2_dial A2 // Rotary potentiometer 2.

// Pin definitions for input rotary switch
#define pinREC 7 // Mode 1 (MIDI RECORD) where manual fader input is actuated and also sent to MIDI OUT.
#define pinOFF 8 // Mode 2 (MIDI OFF) where manual fader input is actuated only.
#define pinPLAY 9 // Mode 3 (MIDI PLAYBACK) where manual fader input is ignored, and MIDI IN values are actuated.

// Pin definitions for output RGB LED
#define pinBLUE 10 // Blue channel
#define pinGREEN 11 // Green channel
#define pinRED 12 // Red channel

// Pin definitions for I/O devices
#define output_1_heat 5 // Output port 1. J7 on the circuit board.
#define output_2_heat 6 // Output port 2. J2 on the circuit board.


// Variables and constants
int lastKnownMode = 0; // Storing the last mode set by rotary switch.
int rawPotentiometer1 = 0; // Readings from dial 1, 0-1023 range.
int rawPotentiometer2 = 0; // Readings from dial 2, 0-1023 range.
int midiPotentiometer1 = 0; // Readings from dial 1 or MIDI in, converted to the 0-127 MIDI range.
int midiPotentiometer2 = 0; // Readings from dial 2 or MIDI in, converted to the 0-127 MIDI range.
int lastPotentiometer1 = 0; // Storing value changes on dial 1, to prevent duplicate MIDI messaging.
int lastPotentiometer2 = 0; // Storing value changes on dial 2, to prevent duplicate MIDI messaging.
int pwmHeat1 = 0; // Readings from dial 1 or MIDI in, converted to the 0-255 PWM range.
int pwmHeat2 = 0; // Readings from dial 2 or MIDI in, converted to the 0-255 PWM range.
int receivedChannel = 0; //Used for MIDI playback.
int receivedControlNumber = 0; //Used for MIDI playback.
int receivedControlValue = 0; //Used for MIDI playback.
const int dial_1_CC = 22; // The control change number used for MIDI fader values.
const int dial_2_CC = 23; // The control change number used for MIDI fader values.


void setup() {
  //Set I/O devices as inputs and outputs
  pinMode(input_1_dial, INPUT); // Defining potentiometer 1 (Right) as input.
  pinMode(input_2_dial, INPUT); // Defining potentiometer 2 (Left) as input.
  pinMode(output_1_heat, OUTPUT); // Defining the PWM pin for port #1 as output.
  pinMode(output_2_heat, OUTPUT); // Defining the PWM pin for port #2 as output.

  // Initialize heat pads to default state
  digitalWrite(output_1_heat, LOW);
  digitalWrite(output_2_heat, LOW);

  // Configure rotary switch pins as input with internal pull-up resistors
  pinMode(pinREC, INPUT_PULLUP);
  pinMode(pinOFF, INPUT_PULLUP);
  pinMode(pinPLAY, INPUT_PULLUP);

  // Configure RGB LED pins as outputs
  pinMode(pinBLUE, OUTPUT);
  pinMode(pinGREEN, OUTPUT);
  pinMode(pinRED, OUTPUT);

  // Turn off all colors initially (common anode means HIGH turns off the LED)
  digitalWrite(pinRED, HIGH);
  digitalWrite(pinGREEN, HIGH);
  digitalWrite(pinBLUE, HIGH);

  //Initiate serial TX/RX both for monitoring and MIDI.
  Serial1.begin(31250); // Set the baud rate for MIDI on the TX. 
  Serial.begin(9600);  // Set the baud rate for serial monitoring over USB.

}


// Sends MIDI CC message via MIDI out
void sendCC(byte channel, byte CC, byte range) {

  // Convert the values from the input devices to MIDI and write to the hardware serial port.
  Serial1.write (0xB0 | (channel-1)); // Channel number (Subtraction adjusts channel for 1-16 MIDI)
  Serial1.write(CC); // CC number
  Serial1.write(range); // Range number

  /*
  // Send values to the USB serial port (only for debugging purposes)
  Serial.print("Record on Channel ");
  Serial.print(channel);
  Serial.print(", Control Number: ");
  Serial.print(CC);
  Serial.print(", Control Value: ");
  Serial.println(range);
  */

}


// Receives MIDI CC messages via MIDI in
void processMIDIByte(byte midiByte) {

  // Check if an incoming message in the hardware serial port's buffer is a Control Change message (status byte: 0xB0)
  if ((midiByte & 0xF0) == 0xB0) { 
    byte channel = midiByte & 0x0F; // Extract MIDI channel
    byte controlNumber = Serial1.read(); // Read the control number
    byte controlValue = Serial1.read(); // Read the control value

    // Process the Control Change message with 'channel', 'controlNumber', and 'controlValue'. Ignore messages out of range due to any USB bit/byte errors.
    if (channel == 0 && controlNumber == dial_1_CC && controlValue <= 127) {
      midiPotentiometer1 = controlValue;
      } else if (channel == 0 && controlNumber == dial_2_CC && controlValue <= 127) {
        midiPotentiometer2 = controlValue;
    } 

  /*
  // Send values to the USB serial port (only for debugging purposes)
  Serial.print("Playback on Channel ");
  Serial.print(channel + 1); // MIDI channels are 1-indexed
  Serial.print(", Control Number: ");
  Serial.print(controlNumber);
  Serial.print(", Control Value: ");
  Serial.println(controlValue);
  */

  } 
}


void loop() {

  // Check rotary switch values to determine current mode.
  bool mode1 = digitalRead(pinREC);
  bool mode2 = digitalRead(pinOFF);
  bool mode3 = digitalRead(pinPLAY);

  // Check if MIDI REC pin is LOW and the last known mode was not already the same.
  if(mode1 == LOW && lastKnownMode != 1) {
    lastKnownMode = 1;
    digitalWrite(pinBLUE, HIGH);
    digitalWrite(pinGREEN, HIGH);
    digitalWrite(pinRED, LOW);
  }
  // Check if MIDI OFF pin is LOW and the last known mode was not already the same.
  if(mode2 == LOW && lastKnownMode != 2) {
    lastKnownMode = 2;
    digitalWrite(pinBLUE, LOW);
    digitalWrite(pinGREEN, LOW);
    digitalWrite(pinRED, LOW);
  }
  // Check if MIDI PLAY pin is LOW and the last known mode was not already the same.
  if(mode3 == LOW && lastKnownMode != 3) {
    lastKnownMode = 3;
    digitalWrite(pinBLUE, HIGH);
    digitalWrite(pinGREEN, LOW);
    digitalWrite(pinRED, HIGH);
    //Reset inputs before listening to playback.
    midiPotentiometer1 = 0;
    midiPotentiometer2 = 0;
  }

  //LOGIC DURING MIDI REC

  if (lastKnownMode == 1) {

    rawPotentiometer1 = analogRead(input_1_dial);
    //Reading and storing the input of the left potentiometer
    rawPotentiometer2 = analogRead(input_2_dial);

    // Map raw values to 0-127 for MIDI
    midiPotentiometer1 = map(rawPotentiometer1, 0, 1023, 0, 127);
    midiPotentiometer2 = map(rawPotentiometer2, 0, 1023, 0, 127);

    // Send MIDI control change messages if the values have changed.
    byte bytePotentiometer1 = midiPotentiometer1; // Map the 10-bit value to 8-bit
    if (bytePotentiometer1 != lastPotentiometer1) {
      sendCC(1, dial_1_CC, bytePotentiometer1); // Send to channel one, CC# 21
      lastPotentiometer1 = bytePotentiometer1; // Update the last value
    }
    byte bytePotentiometer2 = midiPotentiometer2; // Map the 10-bit value to 8-bit
    if (bytePotentiometer2 != lastPotentiometer2) {
      sendCC(1, dial_2_CC, midiPotentiometer2); // Send to channel one, CC# 21
      lastPotentiometer2 = bytePotentiometer2; // Update the last value
    }

    // Map MIDI values to PWM for (potential) output
    pwmHeat1 = map(midiPotentiometer1, 0, 127, 0, 255);
    pwmHeat2 = map(midiPotentiometer2, 0, 127, 0, 255);

    //Takes the mapped inputs of the potentiometers and outputs them to the actuators, as long as the pressure pad is pressed
    analogWrite(output_1_heat, pwmHeat1);
    analogWrite(output_2_heat, pwmHeat2);


  }

  //LOGIC DURING MIDI OFF

  if (lastKnownMode == 2) {

    rawPotentiometer1 = analogRead(input_1_dial);
    //Reading and storing the input of the left potentiometer
    rawPotentiometer2 = analogRead(input_2_dial);

    // Map raw values to 0-127 for MIDI
    midiPotentiometer1 = map(rawPotentiometer1, 0, 1023, 0, 127);
    midiPotentiometer2 = map(rawPotentiometer2, 0, 1023, 0, 127);

    // Map MIDI values to PWM for (potential) output
    pwmHeat1 = map(midiPotentiometer1, 0, 127, 0, 255);
    pwmHeat2 = map(midiPotentiometer2, 0, 127, 0, 255);

    //Takes the mapped inputs of the potentiometers and outputs them to the actuators, as long as the pressure pad is pressed
    analogWrite(output_1_heat, pwmHeat1);
    analogWrite(output_2_heat, pwmHeat2);

  }


//LOGIC DURING MIDI PLAY

  if (lastKnownMode == 3) {

    // Use a for loop to check at least 3 MIDI messages. Check if there are at least 3 bytes (full message) available before processing.
    for (int i = 0; i < 3; i++) {
      if (Serial1.available() >= 3) {
        byte midiByte = Serial1.read();
        // Process the received MIDI byte
        processMIDIByte(midiByte);
      }
      // Clear the serial buffer to avoid duplicate readings.
      while (Serial.available() > 0) {
        byte trash = Serial.read(); // Read and discard any remaining bytes in the buffer
      }
    }

    // Map MIDI values to PWM for (potential) output
    pwmHeat1 = map(midiPotentiometer1, 0, 127, 0, 255);
    pwmHeat2 = map(midiPotentiometer2, 0, 127, 0, 255);

    //Takes the mapped inputs of the potentiometers and outputs them to the actuators, as long as the pressure pad is pressed
    analogWrite(output_1_heat, pwmHeat1);
    analogWrite(output_2_heat, pwmHeat2);

}
}
