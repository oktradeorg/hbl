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
#define slidePotentiometer A0 // Fader, used to control motor speed for inlation and deflation. 

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


// Constants
const int deadZoneStart = 56; // Start of dead zone in middle of fader, where nothing runs. 
const int deadZoneEnd = 70; // End of dead zone.
const int motorStartPWM = 200; // Used to set lowest start speed of motors.
const int limitInflate = 400; // Upper pressure limit where inflation is blocked.
const int limitDeflate = 50; // Lower pressure limit where deflation is blocked.
int lastMappedValue = 0; // Storing value changes on the fader, to prevent duplicate MIDI messaging.
int faderRaw = 0; // Readings from fader, in the 0-1023 range.
int faderMIDI = 0; // Readings from fader, converted to the 0-127 MIDI range.
int pressureRawJ1 = 0; // Readings from pressure sensor on J1 port.
int pressureRawJ2 = 0; // Readings from pressuer sensor on J2 port.
int faderPWM = 0; // Calculated PWM output to send to motors.
int lastKnownMode = 0; // Storing the last mode set by rotary switch.
int receivedChannel = 0; //Used for MIDI playback.
int receivedControlNumber = 0; //Used for MIDI playback.
int receivedControlValue = 0; //Used for MIDI playback.
int faderCC = 22; // The control change number used for MIDI fader values.

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
  pinMode(pinBLUE, OUTPUT);
  pinMode(pinGREEN, OUTPUT);
  pinMode(pinBLUE, OUTPUT);

  // Turn off all colors initially (common anode means HIGH turns off the LED)
  digitalWrite(pinBLUE, HIGH);
  digitalWrite(pinGREEN, HIGH);
  digitalWrite(pinBLUE, HIGH);

  //Serial1.begin(31250, SERIAL_8N1, RXD1, TXD1); // Set pin D1 as serial 1 (TX1) output
  Serial1.begin(31250); // Set the baud rate
  Serial.begin(9600);
}


//Sends MIDI CC messages
void sendCC(byte channel, byte CC, byte range)
{
//Send command via MIDI Out
Serial1.write (0xB0 | (channel-1)); // Channel number (Subtraction adjusts channel for 1-16 MIDI)
Serial1.write(CC); // CC number
Serial1.write(range); // Range number

//Serial monitor debug
Serial.print("Command: Note On\n"); Serial.print("Channel: "); Serial.println(channel);
Serial.print("CC#: "); Serial.println(CC);
Serial.print("Range: ");Serial.println(range); Serial.print("\n");
}

//Receives MIDI CC messages
void processMIDIByte(byte midiByte) {
// Check if it's a Control Change message (status byte: 0xB0)
if ((midiByte & 0xF0) == 0xB0) {
byte channel = midiByte & 0x0F; // Extract MIDI channel
byte controlNumber = Serial1.read(); // Read the control number
byte controlValue = Serial1.read(); // Read the control value

// Process the Control Change message with 'channel', 'controlNumber', and 'controlValue'
if (channel == 0 && controlNumber == faderCC && controlValue <= 127) {
  faderMIDI = controlValue;
} 

//Serial monitor debug
Serial.print("Control Change on Channel ");
Serial.print(channel + 1); // MIDI channels are 1-indexed
Serial.print(", Control Number: ");
Serial.print(controlNumber);
Serial.print(", Control Value: ");
Serial.println(controlValue);
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
    digitalWrite(pinBLUE, LOW);
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
  }


  //LOGIC DURING MIDI REC

  if (lastKnownMode == 1) {

    // Read the fader position
    faderRaw = analogRead(slidePotentiometer);
    // Map faderRaw to 0-127 for MIDI
    faderMIDI = map(faderRaw, 0, 1023, 0, 127);
    // Send MIDI control change message if the value has changed.
    byte mappedValue = faderMIDI; // Map the 10-bit value to 8-bit
    if (mappedValue != lastMappedValue) {
    sendCC(1, faderCC, faderMIDI); // Send to channel one, CC# 21
    lastMappedValue = mappedValue; // Update the last value
    }

    // Read the pressure sensors
    pressureRawJ1 = analogRead(pressureSensorJ1);
    pressureRawJ2 = analogRead(pressureSensorJ2);

    // Ensure that motors are off if pressure limits are exceeded
    if (pressureRawJ1 > limitInflate) {
      digitalWrite(inflateMotorJ1, LOW);
    }
    if (pressureRawJ2 > limitInflate) {
      digitalWrite(inflateMotorJ2, LOW);
    }
    if (pressureRawJ1 < limitDeflate) {
      digitalWrite(deflateMotorJ1, LOW);
    }
    if (pressureRawJ2 < limitDeflate) {
      digitalWrite(deflateMotorJ2, LOW);
    }

    // Process faderMIDI cases
    if (faderMIDI <= deadZoneStart) {
      // Case 1, Deflation: Below the dead zone start
      faderPWM = map(faderMIDI, 0, deadZoneStart, 255, motorStartPWM);
      analogWrite(deflateMotorJ1, pressureRawJ1 < limitDeflate ? 0 : faderPWM);
      analogWrite(deflateMotorJ2, pressureRawJ2 < limitDeflate ? 0 : faderPWM);
      digitalWrite(inflateMotorJ1, LOW);
      digitalWrite(inflateMotorJ2, LOW);
      digitalWrite(inflateSolenoidJ1, LOW);
      digitalWrite(inflateSolenoidJ2, LOW);
      digitalWrite(deflateSolenoidJ1, HIGH);
      digitalWrite(deflateSoleniodJ2, HIGH);
    } else if (faderMIDI > deadZoneStart && faderMIDI < deadZoneEnd) {
      // Case 2, Idle: Within the dead zone
      digitalWrite(inflateMotorJ1, LOW);
      digitalWrite(inflateMotorJ2, LOW);
      digitalWrite(deflateMotorJ1, LOW);
      digitalWrite(deflateMotorJ2, LOW);
      digitalWrite(inflateSolenoidJ1, LOW);
      digitalWrite(inflateSolenoidJ2, LOW);
      digitalWrite(deflateSolenoidJ1, HIGH);
      digitalWrite(deflateSoleniodJ2, HIGH);
    } else if (faderMIDI >= deadZoneEnd) {
      // Case 3, Inflation: Above the dead zone end
      faderPWM = map(faderMIDI, deadZoneEnd, 127, motorStartPWM, 255);
      digitalWrite(deflateMotorJ1, LOW);
      digitalWrite(deflateMotorJ2, LOW);
      analogWrite(inflateMotorJ1, pressureRawJ1 > limitInflate ? 0 : faderPWM);
      analogWrite(inflateMotorJ2, pressureRawJ2 > limitInflate ? 0 : faderPWM);
      digitalWrite(inflateSolenoidJ1, HIGH);
      digitalWrite(inflateSolenoidJ2, HIGH);
      digitalWrite(deflateSolenoidJ1, LOW);
      digitalWrite(deflateSoleniodJ2, LOW);
  }

  }

  //LOGIC DURING MIDI OFF

  if (lastKnownMode == 2) {

    // Read the fader position
    faderRaw = analogRead(slidePotentiometer);
    // Map faderRaw to 0-127 for MIDI
    faderMIDI = map(faderRaw, 0, 1023, 0, 127);

    // Read the pressure sensors
    pressureRawJ1 = analogRead(pressureSensorJ1);
    pressureRawJ2 = analogRead(pressureSensorJ2);

    // Ensure that motors are off if pressure limits are exceeded
    if (pressureRawJ1 > limitInflate) {
      digitalWrite(inflateMotorJ1, LOW);
    }
    if (pressureRawJ2 > limitInflate) {
      digitalWrite(inflateMotorJ2, LOW);
    }
    if (pressureRawJ1 < limitDeflate) {
      digitalWrite(deflateMotorJ1, LOW);
    }
    if (pressureRawJ2 < limitDeflate) {
      digitalWrite(deflateMotorJ2, LOW);
    }

    // Process faderMIDI cases
    if (faderMIDI <= deadZoneStart) {
      // Case 1, Deflation: Below the dead zone start
      faderPWM = map(faderMIDI, 0, deadZoneStart, 255, motorStartPWM);
      analogWrite(deflateMotorJ1, pressureRawJ1 < limitDeflate ? 0 : faderPWM);
      analogWrite(deflateMotorJ2, pressureRawJ2 < limitDeflate ? 0 : faderPWM);
      digitalWrite(inflateMotorJ1, LOW);
      digitalWrite(inflateMotorJ2, LOW);
      digitalWrite(inflateSolenoidJ1, LOW);
      digitalWrite(inflateSolenoidJ2, LOW);
      digitalWrite(deflateSolenoidJ1, HIGH);
      digitalWrite(deflateSoleniodJ2, HIGH);
    } else if (faderMIDI > deadZoneStart && faderMIDI < deadZoneEnd) {
      // Case 2, Idle: Within the dead zone
      digitalWrite(inflateMotorJ1, LOW);
      digitalWrite(inflateMotorJ2, LOW);
      digitalWrite(deflateMotorJ1, LOW);
      digitalWrite(deflateMotorJ2, LOW);
      digitalWrite(inflateSolenoidJ1, LOW);
      digitalWrite(inflateSolenoidJ2, LOW);
      digitalWrite(deflateSolenoidJ1, HIGH);
      digitalWrite(deflateSoleniodJ2, HIGH);
    } else if (faderMIDI >= deadZoneEnd) {
      // Case 3, Inflation: Above the dead zone end
      faderPWM = map(faderMIDI, deadZoneEnd, 127, motorStartPWM, 255);
      digitalWrite(deflateMotorJ1, LOW);
      digitalWrite(deflateMotorJ2, LOW);
      analogWrite(inflateMotorJ1, pressureRawJ1 > limitInflate ? 0 : faderPWM);
      analogWrite(inflateMotorJ2, pressureRawJ2 > limitInflate ? 0 : faderPWM);
      digitalWrite(inflateSolenoidJ1, HIGH);
      digitalWrite(inflateSolenoidJ2, HIGH);
      digitalWrite(deflateSolenoidJ1, LOW);
      digitalWrite(deflateSoleniodJ2, LOW);
  }

  }


//LOGIC DURING MIDI PLAY

  if (lastKnownMode == 3) {

    //Check for new MIDI message
    if (Serial1.available() > 0) {
      byte midiByte = Serial1.read();
      // Process the received MIDI byte
      processMIDIByte(midiByte);
    }

    // Read the pressure sensors
    pressureRawJ1 = analogRead(pressureSensorJ1);
    pressureRawJ2 = analogRead(pressureSensorJ2);

    // Ensure that motors are off if pressure limits are exceeded
    if (pressureRawJ1 > limitInflate) {
      digitalWrite(inflateMotorJ1, LOW);
    }
    if (pressureRawJ2 > limitInflate) {
      digitalWrite(inflateMotorJ2, LOW);
    }
    if (pressureRawJ1 < limitDeflate) {
      digitalWrite(deflateMotorJ1, LOW);
    }
    if (pressureRawJ2 < limitDeflate) {
      digitalWrite(deflateMotorJ2, LOW);
    }

    // Process faderMIDI cases
    if (faderMIDI <= deadZoneStart) {
      // Case 1, Deflation: Below the dead zone start
      faderPWM = map(faderMIDI, 0, deadZoneStart, 255, motorStartPWM);
      analogWrite(deflateMotorJ1, pressureRawJ1 < limitDeflate ? 0 : faderPWM);
      analogWrite(deflateMotorJ2, pressureRawJ2 < limitDeflate ? 0 : faderPWM);
      digitalWrite(inflateMotorJ1, LOW);
      digitalWrite(inflateMotorJ2, LOW);
      digitalWrite(inflateSolenoidJ1, LOW);
      digitalWrite(inflateSolenoidJ2, LOW);
      digitalWrite(deflateSolenoidJ1, HIGH);
      digitalWrite(deflateSoleniodJ2, HIGH);
    } else if (faderMIDI > deadZoneStart && faderMIDI < deadZoneEnd) {
      // Case 2, Idle: Within the dead zone
      digitalWrite(inflateMotorJ1, LOW);
      digitalWrite(inflateMotorJ2, LOW);
      digitalWrite(deflateMotorJ1, LOW);
      digitalWrite(deflateMotorJ2, LOW);
      digitalWrite(inflateSolenoidJ1, LOW);
      digitalWrite(inflateSolenoidJ2, LOW);
      digitalWrite(deflateSolenoidJ1, HIGH);
      digitalWrite(deflateSoleniodJ2, HIGH);
    } else if (faderMIDI >= deadZoneEnd && faderMIDI <= 127) {
      // Case 3, Inflation: Above the dead zone end
      faderPWM = map(faderMIDI, deadZoneEnd, 127, motorStartPWM, 255);
      digitalWrite(deflateMotorJ1, LOW);
      digitalWrite(deflateMotorJ2, LOW);
      analogWrite(inflateMotorJ1, pressureRawJ1 > limitInflate ? 0 : faderPWM);
      analogWrite(inflateMotorJ2, pressureRawJ2 > limitInflate ? 0 : faderPWM);
      digitalWrite(inflateSolenoidJ1, HIGH);
      digitalWrite(inflateSolenoidJ2, HIGH);
      digitalWrite(deflateSolenoidJ1, LOW);
      digitalWrite(deflateSoleniodJ2, LOW);
  }

  }

}