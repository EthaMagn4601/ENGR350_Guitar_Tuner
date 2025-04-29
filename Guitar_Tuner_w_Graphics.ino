// Guitar Tuner Engr 350 Project
// Group 16: Ethan Magnante, Joshua Woolcock
// 4/29/2025
// This Code implements an automatic guitar tuner using a ESP32-s3, a servo motor to turn tuning pegs, and a microphone to detect pitch.
// It includes a TFT display for user interaction and feedback.
// Autocorrelation is use to estimate the pitch of the plucked strings, and the servo motor adjusts the tension until the target frequeuncy is reached.

#include <Arduino.h>
#include "ESP32Servo.h"
#include <Adafruit_GFX.h>    
#include <Adafruit_ST7789.h> 
#include <SPI.h>

// TFT display setup
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Guitar tuner pin and sampling definitions
#define MIC_PIN A0               
#define SERVO_PIN 13 
#define SETTUNER_PIN A2            
#define SETDOWNBUTTON_PIN A3
#define SETUPBUTTON_PIN A4
#define NUM_STRINGS 6
#define SAMPLES 4096
#define SAMPLING_FREQUENCY 10000

// Global varible declaration
unsigned int sampling_period_us; // sampling_period_us & microseconds must be unsigned int as a 32 
unsigned int microseconds;       // bit memory is needed for the large value nature of these two variables

Servo tuningMotor;

bool tunerGo = false;
float samples[SAMPLES];
float autocorrelation[SAMPLES/2];

// Hz array for strings
const float targetOctaves[NUM_STRINGS] = {
    82.41,   // Low E (E2)
    110.00,  // A (A2)
    146.83,  // D (D3)
    198.00,  // G (G3)
    246.94,  // B (B3)
    333.00   // High E (E4)
};

// String name array
const char* stringNames[NUM_STRINGS] = {
    "Low E (E2)",
    "A (A2)",
    "D (D3)",
    "G (G3)",
    "B (B3)",
    "High E (E4)"
};

// Motor functions
void rotateClockwise() {
    tuningMotor.attach(SERVO_PIN);
    tuningMotor.writeMicroseconds(1367);
} 
void rotateCounterClockwise() {
    tuningMotor.attach(SERVO_PIN);
    tuningMotor.writeMicroseconds(1600);
}
void stopMotor() {
    tuningMotor.detach();
}

int currentString = 0;

void setup() {
    Serial.begin(115200);
    
    // Turn on backlite
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);
    
    // Turn on the TFT / I2C power supply
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    
    // Delay for TFT power to initialize
    delay(10);

    // Initialize TFT
    tft.init(135, 240); 
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);

    Serial.println(F("Initialized"));
    
    // Initialize tuner hardware
    tuningMotor.attach(SERVO_PIN);
    pinMode(SETUPBUTTON_PIN, INPUT_PULLUP); 
    pinMode(SETDOWNBUTTON_PIN, INPUT_PULLUP); 
    pinMode(SETTUNER_PIN, INPUT_PULLUP);
    pinMode(MIC_PIN, INPUT);
    stopMotor();
    
    // Display startup screen
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.println("Guitar Tuner");
    tft.setTextSize(1);
    tft.println("Initializing...");
    
    delay(500);
    Serial.println("ESP32 Guitar Tuner with Autocorrelation");
    sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
    
    // Clear screen and show initial string selection
    tft.fillScreen(ST77XX_BLACK);
    updateDisplay();
}

void updateDisplay() {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.print("String: ");
    tft.println(stringNames[currentString]);
    
    tft.setTextSize(1);
    tft.print("Target: ");
    tft.print(targetOctaves[currentString]);
    tft.println(" Hz");
    
    if (tunerGo) {
        tft.setTextColor(ST77XX_GREEN);
        tft.println("Status: TUNING");
    } else {
        tft.setTextColor(ST77XX_YELLOW);
        tft.println("Status: READY");
    }
}

void drawTuningIndicator(float freq) {
    float targetFreq = targetOctaves[currentString];
    float diff = freq - targetFreq;
    int centerX = tft.width() / 2;
    int centerY = tft.height() - 20;
    
    tft.drawFastHLine(0, centerY, tft.width(), ST77XX_WHITE);
    tft.fillTriangle(centerX, centerY - 5, centerX - 5, centerY + 5, centerX + 5, centerY + 5, ST77XX_RED);
    
    // Position indicator based on frequency difference
    int indicatorPos = centerX + map(constrain(diff, -10, 10), -10, 10, -centerX + 10, centerX - 10);
    tft.fillCircle(indicatorPos, centerY - 10, 5, ST77XX_GREEN);
    
    // Display frequency difference
    tft.setCursor(0, centerY + 10);
    tft.print("Diff: ");
    tft.print(diff);
    tft.println(" Hz");
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
    if (ratio > 1.9 && ratio < 2.1) {
        detectedFreq /= ratio;
    }
    else if (ratio > 0.45 && ratio < 0.55) { 
        detectedFreq /= ratio;
    }
    
    return detectedFreq;
}

void checkGo() {
    static unsigned long lastPressTime = 0;
    const unsigned long debounceDelay = 300;

    if (millis() - lastPressTime > debounceDelay) {
        lastPressTime = millis();
        
        if (digitalRead(SETTUNER_PIN) == LOW) {
            lastPressTime = millis();
            tunerGo = !tunerGo; // Toggle tuning mode
            updateDisplay();
            if (tunerGo){
                Serial.println("Tuner Go!");
            } else {
                Serial.println("Tuner Stopped");
            }
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
            Serial.println(stringNames[currentString]);
            updateDisplay();
        }

        if (digitalRead(SETDOWNBUTTON_PIN) == LOW) {
            lastPressTime = millis();
            currentString = (currentString - 1 + NUM_STRINGS) % NUM_STRINGS;
            Serial.print("Selected string: ");
            Serial.println(stringNames[currentString]);
            updateDisplay();
        }
    }
}

void adjustTuning(float freq, float targetFreq) {
    float tolerance = 1.0; // Hz tolerance
    
    // Calculate how far off we are in percent
    float percentDiff = abs(freq - targetFreq) / targetFreq * 100;
    
    if (percentDiff > 10.0) { // If way off
        if (freq < targetFreq) {
            tft.fillScreen(ST77XX_BLACK);
            tft.setTextColor(ST77XX_CYAN);
            tft.setTextSize(2);
            tft.setCursor(0, 0);
            tft.println("Tuning UP (FAST)");
            tuningMotor.writeMicroseconds(1700);
        } else {
            tft.fillScreen(ST77XX_BLACK);
            tft.setTextColor(ST77XX_CYAN);
            tft.setTextSize(2);
            tft.setCursor(0, 0);
            tft.println("Tuning DOWN (FAST)");
            tuningMotor.writeMicroseconds(1367);
        }
    } 
    else if (freq < targetFreq - tolerance) {
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextColor(ST77XX_BLUE);
        tft.setTextSize(2);
        tft.setCursor(0, 0);
        tft.println("Tuning UP...");
        rotateClockwise();
    } 
    else if (freq > targetFreq + tolerance) {
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextColor(ST77XX_BLUE);
        tft.setTextSize(2);
        tft.setCursor(0, 0);
        tft.println("Tuning DOWN...");
        rotateCounterClockwise();
    } 
    else {
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextColor(ST77XX_GREEN);
        tft.setTextSize(3);
        tft.setCursor(0, 0);
        tft.println("IN TUNE!");
        stopMotor();
        tunerGo = false;
        delay(3000);
        updateDisplay();
    }
    
    // Update frequency display
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    tft.setCursor(0, 40);
    tft.print("Detected: ");
    tft.print(freq);
    tft.println(" Hz");
    tft.print("Target: ");
    tft.print(targetFreq);
    tft.println(" Hz");
    
    drawTuningIndicator(freq);
}

void loop() {
    while (tunerGo == false) {
        checkTune();
        checkGo();
        delay(100);
    }
    
    while (tunerGo == true) {
        // Sample audio data
        for (int i = 0; i < SAMPLES; i++) {
            microseconds = micros();
            samples[i] = analogRead(MIC_PIN);
            while (micros() - microseconds < sampling_period_us);
        }
        
        if (!isSignalStrongEnough()) {
            tft.fillScreen(ST77XX_BLACK);
            tft.setTextColor(ST77XX_RED);
            tft.setTextSize(2);
            tft.setCursor(0, 0);
            tft.println("No signal!");
            stopMotor();
            delay(100);
            continue;
        }
        
        // Get frequency using autocorrelation
        float freq = autocorrelationPitchDetection();
        
        // Check if frequency is within expected range for current string
        bool validFrequency = false;
        switch (currentString) {
            case 0: validFrequency = (freq > 40 && freq < 110); break;
            case 1: validFrequency = (freq > 80 && freq < 160); break;
            case 2: validFrequency = (freq > 100 && freq < 180); break;
            case 3: validFrequency = (freq > 120 && freq < 200); break;
            case 4: validFrequency = (freq > 200 && freq < 300); break;
            case 5: validFrequency = (freq > 280 && freq < 400); break;
        }
        
        if (validFrequency) {
            adjustTuning(freq, targetOctaves[currentString]);
        } else {
            tft.fillScreen(ST77XX_BLACK);
            tft.setTextColor(ST77XX_RED);
            tft.setTextSize(2);
            tft.setCursor(0, 0);
            tft.print("Invalid freq: ");
            tft.println(freq);
            stopMotor();
            delay(500);
        }
    }
}
