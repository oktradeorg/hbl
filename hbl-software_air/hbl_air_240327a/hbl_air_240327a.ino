#include <Arduino.h>
/*

PURPOSE

The Haptic Bits Lab includes a library of shapes and actuators to facilitate somaesthetic design —  
a process that allows designers to examine and improve on connections between sensation, feeling, 
emotion, subjective understanding and values.

This program enables inflation and deflation of bags by controlling motors and valves on 2 
separate but identical output ports. Actuation is controlled by moving a slide potentiometer.
The movements can also be recorded to MIDI for later playback. There are also pressure sensors
in the system for safety. The circuitry is further defined in the Haptic Bits Lab documentation.

A rotary switch moves the current active mode between recording, off, and playback. An RGB LED 
indicates the modes as red (MIDI RECORD), blue (MIDI OFF), and green (MIDI PLAYBACK).

*/

// Pin definition for input slide potentiometer
#define input_1_fader A0 // Fader, used to control motor speed for inlation and deflation. 

// Pin definitions for input rotary switch
#define pinREC 15 // Mode 1 (MIDI RECORD) where manual fader input is actuated and also sent to MIDI OUT.
#define pinOFF 16 // Mode 2 (MIDI OFF) where manual fader input is actuated only.
#define pinPLAY 17 // Mode 3 (MIDI PLAYBACK) where manual fader input is ignored, and MIDI IN values are actuated.

// Pin definitions for output RGB LED
#define pinBLUE 7 // Blue channel
#define pinGREEN 11 // Green channel
#define pinRED 12 // Red channel

// Pin definitions for I/O devices on port J1
#define pressureSensorJ1 A6 // Sensor to stop motors when pressure is too low or high.
#define inflateMotorJ1 5 // Motor to inflate bags.
#define deflateMotorJ1 6 // Motor to deflate bags.
#define inflateSolenoidJ1 2 // Valve nearest inflation motor.
#define deflateSolenoidJ1 3 // Valve nearest deflation motor.

// Pin definitions for I/O devices on port J2
#define pressureSensorJ2 A7 // Sensor to stop motors when pressure is too low or high.
#define inflateMotorJ2 9 // Motor to inflate bags.
#define deflateMotorJ2 10 // Motor to deflate bags.
#define inflateSolenoidJ2 4 // Valve nearest inflation motor.
#define deflateSoleniodJ2 8 // Valve nearest deflation motor.

// Variables
int lastKnownMode = 0; // Storing the last mode set by rotary switch.
int fader_last = 0; // Storing value changes on the fader, to prevent duplicate MIDI messaging.
int fader_raw = 0; // Readings from fader, in the 0-1023 range.
int fader_midi = 0; // Readings from fader, converted to the 0-127 MIDI range.
int fader_pwm = 0; // Calculated PWM output to send to motors.
int pressureRawJ1 = 0; // Readings from pressure sensor on J1 port.
int pressureRawJ2 = 0; // Readings from pressuer sensor on J2 port.
int receivedChannel = 0; //Used for MIDI playback.
int receivedControlNumber = 0; //Used for MIDI playback.
int receivedControlValue = 0; //Used for MIDI playback.

// Constants
const int deadZoneStart = 56; // Start of dead zone in middle of fader, where nothing runs. 
const int deadZoneEnd = 70; // End of dead zone.
const int motorStartPWM = 200; // Used to set lowest start speed of motors.
const int limitInflate = 173; // Upper pressure limit where inflation is blocked.
const int limitDeflate = 140; // Lower pressure limit where deflation is blocked.
const int fader_cc = 22; // The control change number used for MIDI fader values.



void setup() {
  // Set the motor and solenoid pins as outputs
  pinMode(inflateMotorJ1, OUTPUT);
  pinMode(inflateMotorJ2, OUTPUT);
  pinMode(deflateMotorJ1, OUTPUT);
  pinMode(deflateMotorJ2, OUTPUT);
  pinMode(inflateSolenoidJ1, OUTPUT);
  pinMode(deflateSolenoidJ1, OUTPUT);
  pinMode(inflateSolenoidJ2, OUTPUT);
  pinMode(deflateSoleniodJ2, OUTPUT);

  // Set the fader and pressure sensors asn inputs
  pinMode(input_1_fader, INPUT);
  pinMode(pressureSensorJ1, INPUT);
  pinMode(pressureSensorJ2, INPUT);

  // Initialize motors and solenoids to default state
  digitalWrite(inflateMotorJ1, LOW);
  digitalWrite(inflateMotorJ2, LOW);
  digitalWrite(deflateMotorJ1, LOW);
  digitalWrite(deflateMotorJ2, LOW);
  digitalWrite(inflateSolenoidJ1, LOW);
  digitalWrite(deflateSolenoidJ1, LOW);
  digitalWrite(inflateSolenoidJ2, LOW);
  digitalWrite(deflateSoleniodJ2, LOW);

  // Configure rotary switch pins as input with internal pull-up resistors
  pinMode(pinREC, INPUT_PULLUP);
  pinMode(pinOFF, INPUT_PULLUP);
  pinMode(pinPLAY, INPUT_PULLUP);

  // Configure RGB LED pins as outputs
  pinMode(pinRED, OUTPUT);
  pinMode(pinGREEN, OUTPUT);
  pinMode(pinBLUE, OUTPUT);

  // Turn off all colors initially (common anode means HIGH turns off the LED)
  digitalWrite(pinRED, HIGH);
  digitalWrite(pinGREEN, HIGH);
  digitalWrite(pinBLUE, HIGH);

  //Initiate serial TX/RX both for monitoring and MIDI.
  Serial1.begin(31250); // Set the baud rate
  Serial.begin(9600);
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
    if (channel == 0 && controlNumber == fader_cc && controlValue <= 127) {
      fader_midi = controlValue;
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

  // Read the pressure sensors
  pressureRawJ1 = analogRead(pressureSensorJ1);
  pressureRawJ2 = analogRead(pressureSensorJ2);  

  // Check rotary switch values to determine current mode.
  bool mode1 = digitalRead(pinREC);
  bool mode2 = digitalRead(pinOFF);
  bool mode3 = digitalRead(pinPLAY);
  
  // Check if MIDI REC pin is LOW and the last known mode was not already the same.
  if(mode1 == LOW && lastKnownMode != 1) {
    lastKnownMode = 1;
    fader_last = 0;
    fader_raw = 0;
    fader_midi = 0;
    fader_pwm = 0;
    digitalWrite(pinBLUE, HIGH);
    digitalWrite(pinGREEN, HIGH);
    digitalWrite(pinRED, LOW);
  }
  // Check if MIDI OFF pin is LOW and the last known mode was not already the same.
  if(mode2 == LOW && lastKnownMode != 2) {
    lastKnownMode = 2;
    fader_last = 0;
    fader_raw = 0;
    fader_midi = 0;
    fader_pwm = 0;
    digitalWrite(pinBLUE, LOW);
    digitalWrite(pinGREEN, HIGH);
    digitalWrite(pinRED, HIGH);
  }
  // Check if MIDI PLAY pin is LOW and the last known mode was not already the same.
  if(mode3 == LOW && lastKnownMode != 3) {
    lastKnownMode = 3;
    fader_last = 0;
    fader_raw = 0;
    fader_midi = 0;
    fader_pwm = 0;
    digitalWrite(pinBLUE, HIGH);
    digitalWrite(pinGREEN, LOW);
    digitalWrite(pinRED, HIGH);
  }


  //LOGIC DURING MIDI REC

  if (lastKnownMode == 1) {

    // Fader input
    fader_raw = analogRead(input_1_fader); // Read the fader position
    fader_midi = map(fader_raw, 0, 1023, 0, 127); // Map fader_raw to 0-127 for MIDI
    byte fader_byte = fader_midi; // Map the 10-bit value to 8-bit
    if (fader_byte != fader_last) {
      sendCC(1, fader_cc, fader_midi); // Send to channel one, CC# 21
      fader_last = fader_byte; // Update the last value
    }

    // Process fader_midi cases
    if (fader_midi <= deadZoneStart) {
      // Case 1, Deflation: Below the dead zone start. Turn on LED if previously turned off.
      fader_pwm = map(fader_midi, 0, deadZoneStart, 255, motorStartPWM);
      analogWrite(deflateMotorJ1, pressureRawJ1 < limitDeflate ? 0 : fader_pwm);
      analogWrite(deflateMotorJ2, pressureRawJ2 < limitDeflate ? 0 : fader_pwm);
      digitalWrite(inflateMotorJ1, LOW);
      digitalWrite(inflateMotorJ2, LOW);
      digitalWrite(inflateSolenoidJ1, LOW);
      digitalWrite(inflateSolenoidJ2, LOW);
      digitalWrite(deflateSolenoidJ1, LOW);
      digitalWrite(deflateSoleniodJ2, LOW);
      digitalWrite(pinBLUE, HIGH);
      digitalWrite(pinGREEN, HIGH);
      digitalWrite(pinRED, LOW);
    } else if (fader_midi > deadZoneStart && fader_midi < deadZoneEnd) {
      // Case 2, Idle: Within the dead zone. Turn on LED if previously turned off.
      digitalWrite(inflateMotorJ1, LOW);
      digitalWrite(inflateMotorJ2, LOW);
      digitalWrite(deflateMotorJ1, LOW);
      digitalWrite(deflateMotorJ2, LOW);
      digitalWrite(inflateSolenoidJ1, LOW);
      digitalWrite(inflateSolenoidJ2, LOW);
      digitalWrite(deflateSolenoidJ1, HIGH);
      digitalWrite(deflateSoleniodJ2, HIGH);
      digitalWrite(pinBLUE, HIGH);
      digitalWrite(pinGREEN, HIGH);
      digitalWrite(pinRED, LOW);
    } else if (fader_midi >= deadZoneEnd) {
      // Case 3, Inflation: Above the dead zone end. Turn on LED if previously turned off.
      fader_pwm = map(fader_midi, deadZoneEnd, 127, motorStartPWM, 255);
      digitalWrite(deflateMotorJ1, LOW);
      digitalWrite(deflateMotorJ2, LOW);
      analogWrite(inflateMotorJ1, pressureRawJ1 > limitInflate ? 0 : fader_pwm);
      analogWrite(inflateMotorJ2, pressureRawJ2 > limitInflate ? 0 : fader_pwm);
      digitalWrite(inflateSolenoidJ1, HIGH);
      digitalWrite(inflateSolenoidJ2, HIGH);
      digitalWrite(deflateSolenoidJ1, HIGH); 
      digitalWrite(deflateSoleniodJ2, HIGH);
      digitalWrite(pinBLUE, HIGH);
      digitalWrite(pinGREEN, HIGH);
      digitalWrite(pinRED, LOW);
    }

    /*
    // Monitor pressure values on USB serial. Used for debugging only.
    Serial.print("J1 (A6) Value: ");
    Serial.print(pressureRawJ1);
    Serial.print(", J2 (A7) Value: ");
    Serial.println(pressureRawJ2);
   */

  }

  //LOGIC DURING MIDI OFF

  if (lastKnownMode == 2) {

    // Fader input
    fader_raw = analogRead(input_1_fader); // Read the fader position
    fader_midi = map(fader_raw, 0, 1023, 0, 127); // Map fader_raw to 0-127 for MIDI

    // Process fader_midi cases
    if (fader_midi <= deadZoneStart) {
      // Case 1, Deflation: Below the dead zone start. Turn on LED if previously turned off.
      fader_pwm = map(fader_midi, 0, deadZoneStart, 255, motorStartPWM);
      analogWrite(deflateMotorJ1, pressureRawJ1 < limitDeflate ? 0 : fader_pwm);
      analogWrite(deflateMotorJ2, pressureRawJ2 < limitDeflate ? 0 : fader_pwm);
      digitalWrite(inflateMotorJ1, LOW);
      digitalWrite(inflateMotorJ2, LOW);
      digitalWrite(inflateSolenoidJ1, LOW);
      digitalWrite(inflateSolenoidJ2, LOW);
      digitalWrite(deflateSolenoidJ1, LOW);
      digitalWrite(deflateSoleniodJ2, LOW);
      digitalWrite(pinBLUE, LOW);
      digitalWrite(pinGREEN, LOW);
      digitalWrite(pinRED, LOW);
    } else if (fader_midi > deadZoneStart && fader_midi < deadZoneEnd) {
      // Case 2, Idle: Within the dead zone. Turn on LED if previously turned off.
      digitalWrite(inflateMotorJ1, LOW);
      digitalWrite(inflateMotorJ2, LOW);
      digitalWrite(deflateMotorJ1, LOW);
      digitalWrite(deflateMotorJ2, LOW);
      digitalWrite(inflateSolenoidJ1, LOW);
      digitalWrite(inflateSolenoidJ2, LOW);
      digitalWrite(deflateSolenoidJ1, HIGH);
      digitalWrite(deflateSoleniodJ2, HIGH);
      digitalWrite(pinBLUE, LOW);
      digitalWrite(pinGREEN, LOW);
      digitalWrite(pinRED, LOW);
    } else if (fader_midi >= deadZoneEnd) {
      // Case 3, Inflation: Above the dead zone end. Turn on LED if previously turned off.
      fader_pwm = map(fader_midi, deadZoneEnd, 127, motorStartPWM, 255);
      digitalWrite(deflateMotorJ1, LOW);
      digitalWrite(deflateMotorJ2, LOW);
      analogWrite(inflateMotorJ1, pressureRawJ1 > limitInflate ? 0 : fader_pwm);
      analogWrite(inflateMotorJ2, pressureRawJ2 > limitInflate ? 0 : fader_pwm);
      digitalWrite(inflateSolenoidJ1, HIGH);
      digitalWrite(inflateSolenoidJ2, HIGH);
      digitalWrite(deflateSolenoidJ1, HIGH); 
      digitalWrite(deflateSoleniodJ2, HIGH);
      digitalWrite(pinBLUE, LOW);
      digitalWrite(pinGREEN, LOW);
      digitalWrite(pinRED, LOW);
    }

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

    // Process fader_midi cases
    if (fader_midi <= deadZoneStart) {
      // Case 1, Deflation: Below the dead zone start. Turn on LED if previously turned off.
      fader_pwm = map(fader_midi, 0, deadZoneStart, 255, motorStartPWM);
      analogWrite(deflateMotorJ1, pressureRawJ1 < limitDeflate ? 0 : fader_pwm);
      analogWrite(deflateMotorJ2, pressureRawJ2 < limitDeflate ? 0 : fader_pwm);
      digitalWrite(inflateMotorJ1, LOW);
      digitalWrite(inflateMotorJ2, LOW);
      digitalWrite(inflateSolenoidJ1, LOW);
      digitalWrite(inflateSolenoidJ2, LOW);
      digitalWrite(deflateSolenoidJ1, LOW);
      digitalWrite(deflateSoleniodJ2, LOW);
      digitalWrite(pinBLUE, HIGH);
      digitalWrite(pinGREEN, LOW);
      digitalWrite(pinRED, HIGH);
    } else if (fader_midi > deadZoneStart && fader_midi < deadZoneEnd) {
      // Case 2, Idle: Within the dead zone. Turn on LED if previously turned off.
      digitalWrite(inflateMotorJ1, LOW);
      digitalWrite(inflateMotorJ2, LOW);
      digitalWrite(deflateMotorJ1, LOW);
      digitalWrite(deflateMotorJ2, LOW);
      digitalWrite(inflateSolenoidJ1, LOW);
      digitalWrite(inflateSolenoidJ2, LOW);
      digitalWrite(deflateSolenoidJ1, HIGH);
      digitalWrite(deflateSoleniodJ2, HIGH);
      digitalWrite(pinBLUE, HIGH);
      digitalWrite(pinGREEN, LOW);
      digitalWrite(pinRED, HIGH);
    } else if (fader_midi >= deadZoneEnd) {
      // Case 3, Inflation: Above the dead zone end. Turn on LED if previously turned off.
      fader_pwm = map(fader_midi, deadZoneEnd, 127, motorStartPWM, 255);
      digitalWrite(deflateMotorJ1, LOW);
      digitalWrite(deflateMotorJ2, LOW);
      analogWrite(inflateMotorJ1, pressureRawJ1 > limitInflate ? 0 : fader_pwm);
      analogWrite(inflateMotorJ2, pressureRawJ2 > limitInflate ? 0 : fader_pwm);
      digitalWrite(inflateSolenoidJ1, HIGH);
      digitalWrite(inflateSolenoidJ2, HIGH);
      digitalWrite(deflateSolenoidJ1, HIGH); 
      digitalWrite(deflateSoleniodJ2, HIGH);
      digitalWrite(pinBLUE, HIGH);
      digitalWrite(pinGREEN, LOW);
      digitalWrite(pinRED, HIGH);
    }

  }


  // Ensure that motors are off if inflation pressure limit is exceed on either port. Blink the LED while supressing.
  if (pressureRawJ1 > limitInflate) {
    digitalWrite(inflateMotorJ1, LOW);
    digitalWrite(pinBLUE, HIGH);
    digitalWrite(pinGREEN, HIGH);
    digitalWrite(pinRED, HIGH);
    
  }
  if (pressureRawJ2 > limitInflate) {
    digitalWrite(inflateMotorJ2, LOW);
    digitalWrite(pinBLUE, HIGH);
    digitalWrite(pinGREEN, HIGH);
    digitalWrite(pinRED, HIGH);
    
  }

  // Ensure that motors are off if deflation pressure limit is exceed on either port. Blink the LED while supressing. To be finished in next revision to take scenarios of only one motor house connected into consideration. 
  /* if (pressureRawJ1 < limitDeflate) {
    digitalWrite(deflateMotorJ1, LOW);
    digitalWrite(pinBLUE, HIGH);
    digitalWrite(pinGREEN, HIGH);
    digitalWrite(pinRED, HIGH);
   
  } 
  if (pressureRawJ2 < limitDeflate) {
    digitalWrite(deflateMotorJ2, LOW);
    digitalWrite(pinBLUE, HIGH);
    digitalWrite(pinGREEN, HIGH);
    digitalWrite(pinRED, HIGH);
    
  } */

}
