#include <Arduino.h>
#include "ESP32Servo.h"

#define MIC_PIN A0               
#define SERVO_PIN 13 
#define SETTUNER_PIN A2            
#define SETDOWNBUTTON_PIN A3
#define SETUPBUTTON_PIN A4
#define POWERBUTTON_PIN 15 
#define NUM_STRINGS 6
#define SAMPLES 4096  // Reduced for autocorrelation performance
#define SAMPLING_FREQUENCY 10000

Servo tuningMotor;
unsigned int sampling_period_us;
unsigned long microseconds;

bool tunerGo = false;
float samples[SAMPLES];
float autocorrelation[SAMPLES/2];

const float targetOctaves[NUM_STRINGS] = {
    82.41,   // Low E (E2)
    110.00,  // A (A2)
    146.83,  // D (D3)
    198.00,  // G (G3)
    246.94,  // B (B3)
    333.00   // High E (E4)
};

int currentString = 0;

void stringName(int stringIndex) {
    switch (stringIndex) {
        case 0: Serial.println("Low E (E2)"); break;
        case 1: Serial.println("A (A2)"); break;
        case 2: Serial.println("D (D3)"); break;
        case 3: Serial.println("G (G3)"); break;
        case 4: Serial.println("B (B3)"); break;
        case 5: Serial.println("High E (E4)"); break;
    }
}

// Motor Functions
void rotateClockwise() {
    tuningMotor.writeMicroseconds(1600);
}
  
void rotateCounterClockwise() {
    tuningMotor.writeMicroseconds(1400);
}

void stopMotor() {
    tuningMotor.writeMicroseconds(1500);
}

void setup() {
    Serial.begin(115200);
    tuningMotor.attach(SERVO_PIN);
    pinMode(SETUPBUTTON_PIN, INPUT_PULLUP); 
    pinMode(SETDOWNBUTTON_PIN, INPUT_PULLUP); 
    pinMode(SETTUNER_PIN, INPUT_PULLUP);
    pinMode(MIC_PIN, INPUT);
    stopMotor();
    delay(500);
    Serial.println("ESP32 Guitar Tuner with Autocorrelation");
    sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
}

bool isSignalStrongEnough() {
    float sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += abs(samples[i] - (SAMPLES / 2)); // Account for DC offset
    }
    return (sum / 100) > 20;
}

float autocorrelationPitchDetection() {
    // Remove DC offset
    float mean = 0;
    for (int i = 0; i < SAMPLES; i++) {
        mean += samples[i];
    }
    mean /= SAMPLES;
    for (int i = 0; i < SAMPLES; i++) {
        samples[i] -= mean;
    }
    // Autocorrelation
    for (int lag = 0; lag < SAMPLES/2; lag++) {
        autocorrelation[lag] = 0;
        for (int i = 0; i < SAMPLES - lag; i++) {
            autocorrelation[lag] += samples[i] * samples[i + lag];
        }
    }
    
    // Find the first major peak after the zero-lag peak
    int peakLag = 10; // Skip the very short lags
    for (int lag = 10; lag < SAMPLES/2; lag++) {
        if (autocorrelation[lag] > autocorrelation[peakLag]) {
            peakLag = lag;
        }
    }
    
    if (peakLag == 0) {
        return 0;
    }
    float detectedFreq = SAMPLING_FREQUENCY / peakLag;

    float ratio = detectedFreq / targetOctaves[currentString];
    Serial.print("Ratio: ");
    Serial.println(ratio);
    Serial.print("targetOctave: ");
    Serial.println(targetOctaves[currentString]);

    if (ratio > 1.9 && ratio < 2.1){
        detectedFreq /= ratio;
        Serial.print("propFreq: ");
        Serial.println(detectedFreq);
    }
    else if (ratio > 0.45 && ratio < 0.55) {
        detectedFreq /= ratio;
    }
    
    return detectedFreq;
}

void checkGo() {
  static unsigned long lastPressTime = 0;
  const unsigned long debounceDelay = 300;

  if (millis() - lastPressTime > debounceDelay){
    lastPressTime = millis();
    
    if (digitalRead(SETTUNER_PIN) == LOW) {
      lastPressTime = millis();
      tunerGo = true;
      Serial.print("Tuner Go!");
    }
  }
}

void checkTune() {
    static unsigned long lastPressTime = 0;
    const unsigned long debounceDelay = 300;

    if (millis() - lastPressTime > debounceDelay) {
        if (digitalRead(SETUPBUTTON_PIN) == LOW) {
            lastPressTime = millis();
            currentString = (currentString + 1) % NUM_STRINGS;
            Serial.print("Selected string: ");
            stringName(currentString);
        }

        if (digitalRead(SETDOWNBUTTON_PIN) == LOW) {
            lastPressTime = millis();
            currentString = (currentString - 1 + NUM_STRINGS) % NUM_STRINGS;
            Serial.print("Selected string: ");
            stringName(currentString);
        }
    }
}

void adjustTuning(float freq, float targetFreq) {
    float tolerance = 1.0; // Hz tolerance
    
    // Calculate how far off we are in percent
    float percentDiff = abs(freq - targetFreq) / targetFreq * 100;
    
    if (percentDiff > 10.0) { // If way off
        if (freq < targetFreq) {
            Serial.println("Tuning Up (fast)...");
            tuningMotor.writeMicroseconds(1700);
        } else {
            Serial.println("Tuning Down (fast)...");
            tuningMotor.writeMicroseconds(1367);
        }
    } 
    else if (freq < targetFreq - tolerance) {
        Serial.println("Tuning Up...");
        rotateClockwise();
    } 
    else if (freq > targetFreq + tolerance) {
        Serial.println("Tuning Down...");
        rotateCounterClockwise();
    } 
    else {
        Serial.println("In Tune!");
        stopMotor();
        tunerGo = false;
        delay(3000);
    }
}

void loop() {

  while (tunerGo == false){
    checkTune();
    checkGo();
    delay(100);
  }
  delay(100);
  while (tunerGo == true){
  
    // Sample audio data
    for (int i = 0; i < SAMPLES; i++) {
        microseconds = micros();
        samples[i] = analogRead(MIC_PIN);
      while (micros() - microseconds < sampling_period_us);
    }
    
    if (!isSignalStrongEnough()) {
        Serial.println("No strong signal detected");
        stopMotor();
        delay(100);
        return;
    }
    
    // Get frequency using autocorrelation
    float freq = autocorrelationPitchDetection();

    if (currentString == 0){
      if (freq > 60 && freq < 110) { // Guitar string range
          Serial.print("Detected: ");
          Serial.print(freq);
          Serial.print(" Hz | Target: ");
          Serial.print(targetOctaves[currentString]);
          Serial.print(" Hz | ");
          stringName(currentString);
          
          adjustTuning(freq, targetOctaves[currentString]);
      }
    }else if (currentString == 1){
      if (freq > 80 && freq < 160) { // Guitar string range
          Serial.print("Detected: ");
          Serial.print(freq);
          Serial.print(" Hz | Target: ");
          Serial.print(targetOctaves[currentString]);
          Serial.print(" Hz | ");
          stringName(currentString);
      }     
          adjustTuning(freq, targetOctaves[currentString]);
    }else if (currentString == 2){
      if (freq > 100 && freq < 180) { // Guitar string range
          Serial.print("Detected: ");
          Serial.print(freq);
          Serial.print(" Hz | Target: ");
          Serial.print(targetOctaves[currentString]);
          Serial.print(" Hz | ");
          stringName(currentString);
      }     
          adjustTuning(freq, targetOctaves[currentString]);
    }else if (currentString == 3){
      if (freq > 120 && freq < 200) { // Guitar string range
          Serial.print("Detected: ");
          Serial.print(freq);
          Serial.print(" Hz | Target: ");
          Serial.print(targetOctaves[currentString]);
          Serial.print(" Hz | ");
          stringName(currentString);
          
          adjustTuning(freq, targetOctaves[currentString]);
      }
    }else if (currentString == 4){
      if (freq > 200 && freq < 300) { // Guitar string range
          Serial.print("Detected: ");
          Serial.print(freq);
          Serial.print(" Hz | Target: ");
          Serial.print(targetOctaves[currentString]);
          Serial.print(" Hz | ");
          stringName(currentString);
          
          adjustTuning(freq, targetOctaves[currentString]);
      }
    }else if (currentString == 5){
      if (freq > 280 && freq < 360) { // Guitar string range
          Serial.print("Detected: ");
          Serial.print(freq);
          Serial.print(" Hz | Target: ");
          Serial.print(targetOctaves[currentString]);
          Serial.print(" Hz | ");
          stringName(currentString);
          
          adjustTuning(freq, targetOctaves[currentString]);
      }           
      else {
        Serial.print("Invalid frequency detected: ");
        Serial.println(freq);
        stopMotor();
      }
  
  }
    
  }   
}