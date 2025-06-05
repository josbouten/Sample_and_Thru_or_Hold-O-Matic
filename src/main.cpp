#include <Arduino.h>


// Original code: Modular in a Week, 2024, 2025

/* JS Bouten 2025-03-05
   De functionaliteit waar ik op ben uitgekomen wijkt af van die die Kristian had geimplementeerd.
   Ik heb dus de gesuggereerde functies niet toegevoegd.

   Met random als ingangssignaal, dus schakelaar in bovenste stand,
   kun je met de knoppen de hoogte van het signaal kiezen. Dit levert dus een
   offset op met een bepaalde amplitude. In de slide mode, regelt de MAX knop
   hoe gladjes de transitities zijn. Van stapjes naar glijden, in andere woorden.
   De MIN knop regelt dat voor het dalen van de transisties.

   Met een extern bipolair signaal in normal mode kun je met de MIN en MAX draaiknop de amplitude en offset vh signaal instellen.

   Met een extern bipolair signaal in slide mode bepaalt de MAX knop de daalsnelheid tussen de stappen.
   De MIN-knop bepaalt de stijgsnelheid tussen de stappen

   N.b. er wordt geen info op de tty-uitgang gezet. Maar omdat bij de
   realisatie van de R2R DAC ook D0 en D1 zijn gebruikt, lijkt er wel output te zijn,
   maar dat zijn de bits 0 en 1 van het gegenereerde signaal.

   2025-06-04 - alle constanten in kapitalen gezet, het gebruik ervan via parameterlijsten geregeld om zij-effecten te voorkomen
                en de code beter leesbaar te maken.
*/

#define USE_EEPROM

#ifdef USE_EEPROM
    #include <EEPROM.h> // Include EEPROM library for saving selected function
    const byte eepromAddress = 0;
    // The place in the eeprom to save the selectedFunction, if changing too often (100000+ times)
    // then change this adress because that address might have been worned out ;)
    // variables to read selected from start from EEPROM!
    byte modeFromEEPROM;    //to make sure we dont write to eeprom too often.
#endif

//constants edit for functionality:
const int cyclesBetweenReadPots = 50; // How often the pots max and min should be checked, and intorext switch.

// Timing constants (in milliseconds)
const unsigned long holdTime1 = 3000; // Time in milli seconds. This is the duration needed to press down the hold button to alternate between modes.


// Define the pin numbers
const int TRIG_IN = A3;      // Pin connected to the button and the trig in jack
const int TRIG_LED = A5;     // Pin connected to the LED.
const int SIGNAL_INPUT = A6; // Input signal to sample or track.
const int SAMPLE_OR_TRACK_PIN = A0; //switch between 5v and GND to decide sample or track.
const int INT_OR_EXT_PIN = A4;

#define NORMAL_MODE 0
#define SLIDE_MODE 1
#define INITIAL_MODE NORMAL_MODE

byte mode = INITIAL_MODE;

// Variables for sample and slide.
unsigned long lastPulseTime = 0; // Stores the time of the last trigger pulse.
unsigned long pulseInterval = 1000; // Default interval between pulses (ms).
int currentValue = 0;  // Current output value that gradually changes.
int targetValue = 0;   // Target value that the output slides towards.
int lastTrigState = 0; //checking if the trig is active or not.
unsigned long currentTime = millis();
int lastValue; //last value for the slide to slide FROM.
float slideFactor = 1.0;        // Factor controlling slide duration (0.0 to 1.0)

// Simple function to blink an LED a specified number of times
void blinkLED(int blinkCount) {
    for (int i = 0; i < 2 * blinkCount; i++) {
        digitalWrite(TRIG_LED, HIGH); // Turn the LED on.
        delay(1000);                  // Wait 250ms.
        digitalWrite(TRIG_LED, LOW);  // Turn the LED off.
        delay(1000);                  // Wait 250ms.
    }
}

byte getMode(byte currentMode) {
    static unsigned long startTime = 0;  // Use static variable to retain state.
    static bool buttonPressed = false;   // Use static variable to track button state.

    // The level of the trigIn is the inverse of the input hold signal.
    if (digitalRead(TRIG_IN) == LOW) {
        if (!buttonPressed) {  // Button press detected (LOW to HIGH transition)
            // Reset the start time, whenever the button is pressed the "first" time
            // while the input is low.
            startTime = millis();
            buttonPressed = true;
        }

        if (millis() - startTime > holdTime1) {
            // Button held for more than 3 seconds.
            // will alternate between modes.
            switch(currentMode) {
                case NORMAL_MODE:
                    currentMode = SLIDE_MODE;
                break;
                case SLIDE_MODE:
                    currentMode = NORMAL_MODE;
                break;
            }
            digitalWrite(TRIG_LED, LOW);
            buttonPressed = false;
            //startTime = millis();
        }
    } else {
      buttonPressed = false;  // Button released (HIGH to LOW transition).
      digitalWrite(TRIG_LED, LOW);
      #ifdef USE_EEPROM
        if (currentMode != modeFromEEPROM) {
            EEPROM.write(eepromAddress, currentMode);
            modeFromEEPROM = currentMode;
        }
      #endif
    }
    return(currentMode);
}

byte doNormalMode(unsigned int sampleValue, byte currentMode) {
    // The level of the trigIn is the inverse of the input hold signal.
    if (digitalRead(TRIG_IN) == LOW) {
        // We have a trig! Lets lock the sample value and send it.
        // light up the trigLed! We have a trig!
        digitalWrite(TRIG_LED, HIGH);

        // Send the sample to the DAC
        PORTD = lowByte(sampleValue);
        PORTB = highByte(sampleValue);

        while (digitalRead(TRIG_IN) == LOW) {
            currentMode = getMode(currentMode);
        }
    } else { // if trigIn
        //================= NO TRIG ================
        // There is no trig...
        digitalWrite(TRIG_LED, LOW); // No more trig...
        if (digitalRead(SAMPLE_OR_TRACK_PIN) == LOW) {
            //============= TRACKMODE ==============
            // Set the 10 pins that together output the input signal directly.
            // This could be an external signal or a sample from the noise source.
            // I.e. send the sample to the DAC.
            PORTD = lowByte(sampleValue);
            PORTB = highByte(sampleValue);
        }
    }
    return(currentMode);
}

// Smoothly slide from lastValue to targetValue
void slideToTarget(unsigned long currentTime, unsigned long pulseInterval, int &currentValue,
                   int targetValue, int lastValue, float slideFactor, unsigned long lastPulseTime) {
  if (pulseInterval == 0) return; // Avoid division by zero.

  // Calculate the progress as a fraction of pulseInterval, adjusted by slideFactor.
  unsigned long elapsedTime = currentTime - lastPulseTime;
  float progress = float(elapsedTime) / (pulseInterval * slideFactor);

  if (progress >= 1.0 || slideFactor == 0.0) {
    currentValue = targetValue; // Instantly jump to target if slideFactor is 0.
  } else {
    currentValue = lastValue + (targetValue - lastValue) * progress;
  }
}

// Slide from sampled level to sampled level. High pot determines smoothness or transitions.
byte doSlideMode(int highCut, int lowCut, unsigned int sampleValue, byte currentMode, unsigned long &pulseInterval,
                 int &lastTrigState, int &currentValue, int &targetValue, int &lastValue, float slideFactor,
                 unsigned long &lastPulseTime) {
    // Active TRIG IN, read the time and values for slide.
    unsigned long currentTime = millis(); // Ensure currentTime is declared and updated at the start.

    // The level of the trigIn is the inverse of the input hold signal.
    if (digitalRead(TRIG_IN) == LOW) {
        // We have a trig! Lock the sample value and send it.
        digitalWrite(TRIG_LED, HIGH); // Light up the trig LED

        if (lastTrigState == 0) { // Check the previous state
            lastValue = currentValue;
            //currentValue = sample;                // Lock the new sample value.
            targetValue = sampleValue;
            pulseInterval = currentTime - lastPulseTime; // Update pulse interval.
            lastPulseTime = currentTime;          // Update last pulse time.
        }
        lastTrigState = 1; // Set the current trigger state.
    } else { // TrigIn is HIGH (not active).
        digitalWrite(TRIG_LED, LOW);
        lastTrigState = 0; // Reset the trigger state.
    }

    if (lastValue < targetValue) {
        slideFactor = float(lowCut) / 1023.0; // Map potentiometer value to 0.0 - 1.0
    } else {
        slideFactor = float(highCut) / 1023.0; // Map potentiometer value to 0.0 - 1.0
    }


    slideToTarget(currentTime, pulseInterval, currentValue,
                  targetValue, lastValue, slideFactor, lastPulseTime); // Sets currentValue. Smoothly transition to the next value

    // Output the current value as a PWM signal on the analog output pin.
    // N.b. er wordt geen info op de tty-uitgang gezet. Maar omdat bij de
    // realisatie van de R2R DAC ook D0 en D1 zijn gebruikt, lijkt er wel output te zijn,
    // maar dat zijn de bits 0 en 1 van het gegenereerde signaal.
    PORTD = lowByte(currentValue);
    PORTB = highByte(currentValue);

    // Check to see if we go into an alternate function.
   return(getMode(currentMode));
}

#define DELAY_TIME 100

void ledTest() {
    for (byte i = 0; i < 8; i++) {
        digitalWrite(TRIG_LED, HIGH);
        delay(DELAY_TIME);
        digitalWrite(TRIG_LED, LOW);
        delay(DELAY_TIME);
    }
}

void setup() {
    // Set the appropriate pins as output for port manipulation.
    DDRD = 0b11111111;
    DDRB = 0b11111111;
    pinMode(SAMPLE_OR_TRACK_PIN, INPUT_PULLUP);
    pinMode(INT_OR_EXT_PIN, INPUT_PULLUP);
    pinMode(TRIG_IN, INPUT_PULLUP);

    ledTest();

    #ifdef USE_EEPROM
        mode = EEPROM.read(0);
    #else
        mode = INITIAL_MODE;
    #endif
    #ifdef USE_EEPROM
        modeFromEEPROM = mode; // To make sure we dont write to eeprom without needing to.
        if (mode < NORMAL_MODE || mode > SLIDE_MODE) {
            mode = NORMAL_MODE;
        }
    #endif
}

void loop() {
    static int highCut = 1023;
    static int lowCut = 0;
    static int intOrExt = INITIAL_MODE; // If we are reading from input or using internal noise source 0=internal, 1=external
    unsigned int sample;
    unsigned int mappedSample;
    static int timeToReadPots = 0; // Only reads the low and high pots once every 50 cycles.

    // Check the pots for value range
    if (timeToReadPots >= cyclesBetweenReadPots) {
        lowCut   = analogRead(A1);
        highCut  = analogRead(A2);
        intOrExt = digitalRead(INT_OR_EXT_PIN);
        timeToReadPots = 0;
    } else {
        timeToReadPots++;
    }

    // Getting a new sample and preprocessing it.
    // The sample upper and lower limits are set using the 2 potentiometer values.
    // We do this in normal mode and in slide mode.
    if (intOrExt == HIGH) {
        // Using external signal.
        // map(value, fromLow, fromHigh, toLow, toHigh)
        sample = (unsigned int) analogRead(SIGNAL_INPUT);
        switch(mode) {
            case NORMAL_MODE:
                mappedSample = (unsigned int) map(sample, 0, 1023, lowCut, highCut);
                mode = doNormalMode(mappedSample, mode);
                break;
            case SLIDE_MODE:
                // We do not scale or offset. This can be done with OFFSET-O-Matic.
                // The MAX pot sets the slide time, the MIN plot does nothing.
                mode = doSlideMode(highCut, lowCut, sample, mode, pulseInterval, lastTrigState,
                                   currentValue, targetValue, lastValue, slideFactor, lastPulseTime);
                break;
        }
    } else {
        // Using internal noise source.
        // In slide mode the amount of sliding is always the same.
        switch(mode) {
            case NORMAL_MODE:
                // We scale the signal according to the pots.
                sample = (unsigned int) random(lowCut, highCut);
                mode = doNormalMode(sample, mode);
            break;
            case SLIDE_MODE:
                sample = (unsigned int) random(0, 1023);
                // We do not scale the signal but use the pots to set the slide speed.
                slideFactor = float(highCut) / 1023.0; // Map potentiometer value to 0.0 - 1.0
                mode = doSlideMode(highCut, lowCut, sample, mode, pulseInterval, lastTrigState,
                                   currentValue, targetValue, lastValue, slideFactor, lastPulseTime);
            break;
        }
    }
}