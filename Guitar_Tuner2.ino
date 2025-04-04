#include <Arduino.h>
#include "arduinoFFT.h"
#include "ESP32Servo.h"

#define MIC_PIN A0               
#define SERVO_PIN 13 
#define SETTUNER A2            
#define SETDOWNBUTTON_PIN A3
#define SETUPBUTTON_PIN A4
#define POWERBUTTON_PIN 15 
#define NUM_STRINGS 6
#define TUNE_STRINGS 6
#define CHANNEL 34


Servo tuningMotor;
const uint16_t SAMPLES = 128; //This value MUST ALWAYS be a power of 2
const float SAMPLING_FREQUENCY = 1500; //Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us;
unsigned long microseconds;

float vReal[SAMPLES];
float vImag[SAMPLES];

const double targetOctaves[NUM_STRINGS] = {
    82.41, //Low E
    105.00,// A
    152.00,// D
    199.00,// G
    246.94,// B
    329.63,// High E
};

// const char targetTune[TUNE_STRINGS] = {
//     0,// Low E
//     1,// A
//     2,// D
//     3,// G
//     4,// B
//     5,// High E
// };

int currentString = 0;
bool buttonPressed = false;
// bool PowerOn = false;

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY, true);

void stringName(int stringIndex) {
    switch (stringIndex)
    {
    case 0: Serial.println("Low E"); break; //DEBUG
    case 1: Serial.println("A"); break; //DEBUG
    case 2: Serial.println("D"); break; //DEBUG
    case 3: Serial.println("G"); break; //DEBUG
    case 4: Serial.println("B"); break; //DEBUG
    case 5: Serial.println("High E"); break; //DEBUG
    }
  }


  
// Motor Functions
void rotateClockwise() {
    tuningMotor.writeMicroseconds(1600);  // Rotate CW (tune up)
  }
  
void rotateCounterClockwise() {
    tuningMotor.writeMicroseconds(1400);  // Rotate CCW (tune down)
  }
void stopMotor() {
    tuningMotor.writeMicroseconds(1500);  // Stop motor (neutral position)
  }

void setup() {
    Serial.begin(115200);
    tuningMotor.attach(SERVO_PIN);
    pinMode(SETUPBUTTON_PIN, INPUT_PULLUP); 
    pinMode(SETDOWNBUTTON_PIN, INPUT_PULLUP); 
    pinMode(SETTUNER, INPUT_PULLUP);
    pinMode(MIC_PIN, INPUT);
    stopMotor();
    delay(500);
    Serial.println("ESP32 Guitar Tuner Starting...");//Troubleshoot
}

// void checkPowerButton(){
//     static bool lastState = HIGH;  // Store last button state
//     bool currentState = digitalRead(POWERBUTTON_PIN);
  
//     if (currentState == LOW && lastState == HIGH) { // Detect button press
//       PowerOn = !PowerOn;  // Toggle power state
//       Serial.print("Power: ");
//       Serial.println(PowerOn ? "ON" : "OFF");//Troubleshoot
  
//       if (!PowerOn) {
//         stopMotor();  // Stop motor when turning OFF
//       }
  
//       delay(200);  // Debounce delay
//     }
  
//     lastState = currentState;  // Update last button state

// }

// void checkSet(){
//   static unsigned long lastPressTime=0;
//   const unsigned long debounceDelay=200;

//   if (millis() - lastPressTime> debounceDelay){
//       if(digitalRead(SETTUNER) == LOW) {

//       }
//   }

// }


void checkTune() {
    static unsigned long lastPressTime = 0;
    const unsigned long debounceDelay = 200; // 200ms debounce time

    if (millis() - lastPressTime > debounceDelay) {
        if (digitalRead(SETUPBUTTON_PIN) == LOW) {  // Button is pressed
            lastPressTime = millis();  // Update last press time
            currentString = (currentString + 1) % NUM_STRINGS;  // Cycle forward
            Serial.print("Tuning Up -> Current String: ");
            Serial.println(currentString);
        }

        if (digitalRead(SETDOWNBUTTON_PIN) == LOW) {  // If down button is pressed
            lastPressTime = millis();
            currentString = (currentString - 1 + NUM_STRINGS) % NUM_STRINGS;  // Safe decrement
            Serial.print("Tuning Down -> Current String: ");
            Serial.println(currentString);
        }
    //     if (digitalRead(SETTUNER) == LOW) {  
    //       if (millis() - lastPressTime > debounceDelay) {  
    //         lastSetPressTime = millis();
    //         Serial.println("SET Button Pressed! Confirming selection...");
    //         // Add any action you want when the SET button is pressed
    //     }
    // }
}



void adjustTuning(double freq, double targetFreq) {
    double tolerance = 3.0; 
  
    if (freq < targetFreq - tolerance) {
      Serial.println("Tuning Up..."); //debug
      rotateClockwise();  // Increase tension
    } 
    else if (freq > targetFreq + tolerance) {
      Serial.println("Tuning Down..."); //debug
      rotateCounterClockwise();  // Decrease tension
    } 
    else {
      Serial.println("String In Tune!"); //debug
      stopMotor();
      delay(10000);
    }
}
  
double getPeakFrequency() {
    double peakValue = 0;
    int peakIndex = 0;

    // Ignore the first few bins (DC offset & noise)
    for (int i = 2; i < (SAMPLES / 2); i++) {
        if (vReal[i] > peakValue) {
            peakValue = vReal[i];
            peakIndex = i;
        }
    }

    double frequency = (peakIndex * (SAMPLING_FREQUENCY / SAMPLES));
    // * 659 / 703
    return frequency;

}



void loop() {

    checkTune();
    // 1. Sample audio data
    for (int i = 0; i < SAMPLES; i++) {
        microseconds = micros();
        vReal[i] = analogRead(MIC_PIN);  // Read mic input
        vImag[i] = 0;  // Set imaginary part to zero
        while (micros() - microseconds < (1000000.0 / SAMPLING_FREQUENCY));  // Maintain sample rate
    }

    // 2. Apply FFT
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Apply window function to reduce spectral leakage
    FFT.compute(FFT_FORWARD);  // Perform FFT
    FFT.complexToMagnitude();  // Convert complex numbers to magnitudes

    // 3. Find dominant frequency
    double freq = getPeakFrequency();

    // 4. Process tuning
    if (freq > 0) {  // Only adjust if a valid frequency is detected
        Serial.print("Detected Frequency: ");
        Serial.println(freq);  // Debugging output
        adjustTuning(freq, targetOctaves[currentString]);
        Serial.println(currentString);
    } else {
        Serial.println("No valid frequency detected.");//debug
    }

  delay(500);
}




