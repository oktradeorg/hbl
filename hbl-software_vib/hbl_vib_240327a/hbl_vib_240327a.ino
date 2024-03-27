/*

PURPOSE

The Haptic Bits Lab includes a library of shapes and actuators to facilitate somaesthetic design —  
a process that allows designers to examine and improve on connections between sensation, feeling, 
emotion, subjective understanding and values.

This program enables recording, and playback, of MIDI signal from a set of potentiometers and to 
Haptic™ Hybrid Tough Type (HTT) reactors, model AFT14A903A with the corresponding circuitry defined
in the Haptic Bits Lab documentation.

The HTT's are designed to add feel to controls through quick response to an applied control signal, 
generating vibrational forces with two responsance points at 320 and 160 Hz for vertical and 
horizontal movement respectively. In layman terms, each high signal generates a movement. Applying 
quick bursts of high signals will generate movement which can be represented as a waveform. 
Changes in frequency of the high vis-a-vis low signal timings (while remaining dependant on the 
current positioning and direction of the HTT when signal is applied) will generate different 
vibrational patterns.

A pressure pad allows for rhythm creation by touch. While the pressure pad is registering touch, 
A rotary potentiometer sets the frequency for the high signal in a range between the two resonance 
frequencies. A second rotary potentiometer does the same for the freuqnecy of the low time between 
each high signal. Thus, setting the two simultaneously in their highest or lowest settings should put
the HTT's close to one of their two resonance frequencies.

A rotary switch moves the current active mode between recording, off, and playback. An RGB LED 
indicates the modes as red (record), off, and green (playback).

*/



// Pin definitions for I/O devices
#define input_1_dial A0 // Rotary potentiometer. Used for LOW state timing. Knob closest to pressure pad.      
#define input_2_dial A2 // Rotary potentiometer. Used for HIGH state timing. Knob closest to mode switch. 
#define input_3_pad A1 // Pressure pad potentiometer. Used for rhythm. 
#define output_1_reactor 5 // Output port 1. J7 on the circuit board.
#define output_2_reactor 6 // Output port 2. J2 on the circuit board.

// Pin definitions for input rotary switch
#define pinREC 7 // Mode 1 (MIDI RECORD) where manual fader input is actuated and also sent to MIDI OUT.
#define pinOFF 9 // Mode 2 (MIDI OFF) where manual fader input is actuated only.
#define pinPLAY 8 // Mode 3 (MIDI PLAYBACK) where manual fader input is ignored, and MIDI IN values are actuated.

// Pin definitions for RGB LED
#define pinBLUE 10
#define pinGREEN 11
#define pinRED 12


// Variables
int lastKnownMode = 0;
int lastPotentiometer1 = 0;
int lastPotentiometer2 = 0;
int lastPressurePad = 0;
int pulseCounter = 0; // Used to track number of pulses while pulsating.
int playbackInit = 0; // Used to check if it's the first iteration of the playback code, so input values can be read once.
unsigned long previousMicros = 0; // Used to create time stamps while pulsating.
unsigned long durationSum = 0; // Used to measure total time spent on HIGH and LOW states while pulsating.
int rawPotentiometer1 = 0; // Readings from dial 1, 0-1023 range.
int rawPotentiometer2 = 0; // Readings from dial 2, 0-1023 range.
int rawPressurePad = 0; // Readings from pad, 0-1023 range.
int midiPotentiometer1 = 0; // Readings from dial 1 or MIDI in, converted to the 0-127 MIDI range.
int midiPotentiometer2 = 0; // Readings from dial 2 or MIDI in, converted to the 0-127 MIDI range.
int midiPressurePad = 0; // Readings from pad or MIDI in, converted to the 0-127 MIDI range.

//Constants
unsigned long cycleDuration = 50000; // Total time of one pulsation cycle, including 3 HIGH states, 3 LOW states, and a safety break to prevent over-oscillation.
const int cycleRepeats = 3; //Number of HIGH and LOW states each to process in one pulsating cycle.
const int pressureSensitivity = 6; // Pressure pad sensitivity threshold. May differ from board to board.
const int resonanceHorizontal = 1563; // The HTT horizontal movement resonance frequency. 
const int resonanceVertical = 3125; // The HTT vertical movement resonance frequency. 
const int deadZoneStart = 61; // Used to control output in the middle range of potentiometers.
const int deadZoneEnd = 65; // Used to control output in the middle range of potentiometers.
const int dial_1_CC = 22; // The control change number used for MIDI fader values.
const int dial_2_CC = 23; // The control change number used for MIDI fader values.
const int pad_CC = 24; // The control change number used for MIDI fader values.



// Pulsation states for vibration
enum State {
  HIGH_STATE,
  LOW_STATE,
  WAIT_STATE
};
State currentState = HIGH_STATE; // Ensures the state switcher can track states.
State previousState = WAIT_STATE; // Set to a different state to ensure proper initialization.



void setup() {
  pinMode(input_1_dial, INPUT); // Defining the potentiometer as input.
  pinMode(input_2_dial, INPUT); // Defining the potentiometer as input.
  pinMode(input_3_pad, INPUT); // Defining the pressure pad as input.
  pinMode(output_1_reactor, OUTPUT); // Defining the PWM pin for port #1 as output.
  pinMode(output_2_reactor, OUTPUT); // Defining the PWM pin for port #2 as output.

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

  // Initiate serial TX/RX both for monitoring and MIDI.
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
    if (channel == 0 && controlNumber == dial_1_CC && controlValue <= 127) {
      midiPotentiometer1 = controlValue;
      } else if (channel == 0 && controlNumber == dial_2_CC && controlValue <= 127) {
        midiPotentiometer2 = controlValue;
      } else if (channel == 0 && controlNumber == pad_CC && controlValue <= 127) {
        midiPressurePad = controlValue;
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



// Generate pulses (when applicable) to the vibration motors.
void pulseFunction(int midi_pot_pulses, int midi_pot_channels) {
  unsigned long currentMicros = micros();
    static unsigned long pulseDuration = 0;
    static unsigned long restDuration = 0;

    // Check for state change and handle the entry actions
    if (currentState != previousState) {
        previousMicros = currentMicros; // Reset the timer at each state entry
        if (currentState == HIGH_STATE) {
            pulseDuration = map(midi_pot_pulses, 0, 127, resonanceHorizontal, resonanceVertical);
          if (midi_pot_channels <= deadZoneStart) {
            digitalWrite(output_1_reactor, HIGH);
            digitalWrite(output_2_reactor, LOW);
          } else if (midi_pot_channels > deadZoneStart && midi_pot_channels < deadZoneEnd) {
            digitalWrite(output_1_reactor, HIGH);
            digitalWrite(output_2_reactor, HIGH);
          } else if (midi_pot_channels >= deadZoneEnd) {
            digitalWrite(output_1_reactor, LOW);
            digitalWrite(output_2_reactor, HIGH);
          }

        } else if (currentState == LOW_STATE) {
            restDuration = map(midi_pot_pulses, 0, 127, resonanceHorizontal, resonanceVertical);
            digitalWrite(output_1_reactor, LOW);
            digitalWrite(output_2_reactor, LOW);
        }
        previousState = currentState; // Update previousState for the next iteration
    }

    // Handle state-specific timing and transitions
    switch (currentState) {
        case HIGH_STATE:
            if (currentMicros - previousMicros >= pulseDuration) {
                durationSum += currentMicros - previousMicros; // Accumulate the HIGH state duration
                currentState = LOW_STATE; // Transition to LOW_STATE
            }
            break;

        case LOW_STATE:
            if (currentMicros - previousMicros >= restDuration) {
                durationSum += currentMicros - previousMicros; // Accumulate the LOW state duration
                pulseCounter++; // Increment the number of completed HIGH/LOW cycles
                if (pulseCounter >= cycleRepeats) {
                    currentState = WAIT_STATE; // Transition to WAIT_STATE after 3 cycles
                } else {
                    currentState = HIGH_STATE; // Otherwise, go back to HIGH_STATE
                }
            }
            break;

        case WAIT_STATE:
            if (currentMicros - previousMicros >= (cycleDuration - durationSum)) {
                // Do whatever is needed when the wait is over, possibly reset for the next cycle
                pulseCounter = 0; // Reset the pulse counter for the next cycle
                durationSum = 0; // Reset the duration sum for the next cycle
                currentState = HIGH_STATE; // Start the cycle over again
            }
            break;
    }
}



// Resets all states for consistent pulses after mode switching.
void resetStateMachine() {
    currentState = HIGH_STATE;
    previousState = WAIT_STATE; // Set to a different state to ensure proper initialization
    previousMicros = 0; // Reset the time reference
    durationSum = 0; // Reset the sum of durations
    pulseCounter = 0; // Reset the pulse counter
    // Reset any outputs or other variables if needed
    digitalWrite(output_1_reactor, LOW);
    digitalWrite(output_2_reactor, LOW);
}





void loop() {

  // Check rotary switch values to determine current mode.
  bool mode1 = digitalRead(pinREC);
  bool mode2 = digitalRead(pinPLAY);
  bool mode3 = digitalRead(pinOFF);
 
  // Check if MIDI REC pin is LOW and the last known mode was not already the same.
  if(mode1 == LOW && lastKnownMode != 1) {
    lastKnownMode = 1;
    playbackInit = 0;
    midiPressurePad = 0;
    rawPotentiometer1 = 0;
    midiPotentiometer2 = 0;
    digitalWrite(pinBLUE, HIGH);
    digitalWrite(pinGREEN, HIGH);
    digitalWrite(pinRED, LOW);
  }
  // Check if MIDI OFF pin is LOW and the last known mode was not already the same.
  if(mode2 == LOW && lastKnownMode != 2) {
    lastKnownMode = 2;
    playbackInit = 0;
    midiPressurePad = 0;
    rawPotentiometer1 = 0;
    midiPotentiometer2 = 0;
    digitalWrite(pinBLUE, LOW);
    digitalWrite(pinGREEN, LOW);
    digitalWrite(pinRED, LOW);
  }
  // Check if MIDI PLAY pin is LOW and the last known mode was not already the same.
  if(mode3 == LOW && lastKnownMode != 3) {
    lastKnownMode = 3;
    playbackInit = 1;
    midiPressurePad = 0;
    rawPotentiometer1 = 0;
    midiPotentiometer2 = 0;
    digitalWrite(pinBLUE, HIGH);
    digitalWrite(pinGREEN, LOW);
    digitalWrite(pinRED, HIGH);
  }

  
  // LOGIC DURING MIDI REC

  if (lastKnownMode == 1) {

    // Pressure pad input
    rawPressurePad = analogRead(input_3_pad); // Read the pressure pad
    midiPressurePad = map(rawPressurePad, 0, 1023, 0, 127); // Map to 0-127 for MIDI
    byte bytePressurePad = midiPressurePad; // Map the 10-bit value to 8-bit
    if (bytePressurePad != lastPressurePad) {
      sendCC(1, pad_CC, bytePressurePad); // Send to channel and CC#
      lastPressurePad = bytePressurePad; // Update the last value
    }

    // Potentiometer 1 input
    rawPotentiometer1 = analogRead(input_1_dial); // Read potentiometer
    midiPotentiometer1 = map(rawPotentiometer1, 0, 1023, 127, 0); // Map to 0-127 (inverted) for MIDI
    byte bytePotentiometer1 = midiPotentiometer1; // Map the 10-bit value to 8-bit
    if (bytePotentiometer1 != lastPotentiometer1) {
      sendCC(1, dial_1_CC, bytePotentiometer1); // Send to channel and CC#
      lastPotentiometer1 = bytePotentiometer1; // Update the last value
    }

    // Potentiometer 2 input
    rawPotentiometer2 = analogRead(input_2_dial); // Read potentiometer
    midiPotentiometer2 = map(rawPotentiometer2, 0, 1023, 0, 127);
    byte bytePotentiometer2 = midiPotentiometer2; // Map the 10-bit value to 8-bit
    if (bytePotentiometer2 != lastPotentiometer2) {
      sendCC(1, dial_2_CC, bytePotentiometer2); // Send to channel and CC#
      lastPotentiometer2 = bytePotentiometer2; // Update the last value
    }

    // Run if pressure above the tolerance level is being applied to the pressure pad
    if (midiPressurePad >= pressureSensitivity) {
      pulseFunction(midiPotentiometer1, midiPotentiometer2);
    } else resetStateMachine();
  }


  //LOGIC DURING MIDI OFF

  if (lastKnownMode == 2) {

    // Pressure pad input
    rawPressurePad = analogRead(input_3_pad); // Read the pressure pad
    midiPressurePad = map(rawPressurePad, 0, 1023, 0, 127); // Map to 0-127 for MIDI

    // Potentiometer 1 input
    rawPotentiometer1 = analogRead(input_1_dial); // Read potentiometer
    midiPotentiometer1 = map(rawPotentiometer1, 0, 1023, 127, 0); // Map to 0-127 (inverted) for MIDI

    // Potentiometer 2 input
    rawPotentiometer2 = analogRead(input_2_dial); // Read potentiometer
    midiPotentiometer2 = map(rawPotentiometer2, 0, 1023, 0, 127);

    // Run if pressure above the tolerance level is being applied to the pressure pad
    if (midiPressurePad >= pressureSensitivity) {
      pulseFunction(midiPotentiometer1, midiPotentiometer2);
    } else resetStateMachine();

  }


  //LOGIC DURING MIDI PLAY

  if (lastKnownMode == 3) {

    // Recording will only pick up on changes to control values. Check if this is the first iteration of playback mode, and if so, check input sensor current values.
    if (playbackInit == 1) {

      // Potentiometer 1 input
      rawPotentiometer1 = analogRead(input_1_dial); // Read potentiometer
      midiPotentiometer1 = map(rawPotentiometer1, 0, 1023, 127, 0); // Map to 0-127 (inverted) for MIDI

      // Potentiometer 2 input
      rawPotentiometer2 = analogRead(input_2_dial); // Read potentiometer
      midiPotentiometer2 = map(rawPotentiometer2, 0, 1023, 0, 127);
      
      /*
      // Send values to the USB serial port (only for debugging purposes)
      Serial.print("Playback on Channel ");
      Serial.print(1); // MIDI channels are 1-indexed
      Serial.print(", Control Number: ");
      Serial.print(dial_1_CC);
      Serial.print(", Control Value: ");
      Serial.println(midiPotentiometer1);
      Serial.print("Playback on Channel ");
      Serial.print(1); // MIDI channels are 1-indexed
      Serial.print(", Control Number: ");
      Serial.print(dial_2_CC);
      Serial.print(", Control Value: ");
      Serial.println(midiPotentiometer2);
      */   

      // Reset variable so this reading is only done once at initialization of playback mode.
      playbackInit = 0;
    
    }


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


    // Run if pressure above the tolerance level is being applied to the pressure pad
    if (midiPressurePad >= pressureSensitivity) {
      pulseFunction(midiPotentiometer1, midiPotentiometer2);
    } else resetStateMachine();

  }

}

