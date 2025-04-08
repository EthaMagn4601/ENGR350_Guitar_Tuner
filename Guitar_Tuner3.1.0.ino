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
#define NUM_AVG 5  // Number of readings to average

double frequencyBuffer[NUM_AVG] = {0};  
int freqIndex = 0;  // Rolling index
bool bufferFilled = false;  // Track when buffer is fully populated

Servo tuningMotor;
const uint16_t SAMPLES = 2048; //This value MUST ALWAYS be a power of 2
const float SAMPLING_FREQUENCY = 5000; //Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us;
unsigned long microseconds;

float vReal[SAMPLES];
float vImag[SAMPLES];

const double targetOctaves[NUM_STRINGS] = {
    164.06, //Low E
    329.29,// A
    146.48,// D
    196.29,// G
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
    tuningMotor.write(60);  // Rotate CW (tune up)
  }
  
void rotateCounterClockwise() {
    tuningMotor.write(120);  // Rotate CCW (tune down)
  }
void stopMotor() {
    tuningMotor.write(93);  // Stop motor (neutral position)
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




void checkTune() {
    static unsigned long lastPressTime = 0;
    const unsigned long debounceDelay = 200; // 

    if (millis() - lastPressTime > debounceDelay) {
        if (digitalRead(SETUPBUTTON_PIN) == LOW) {  // Button is pressed
            lastPressTime = millis();  // Update last press time
            currentString = (currentString + 1) % NUM_STRINGS;  // Cycle forward
            Serial.print("Tuning Up -> Current String: ");
            Serial.println(currentString);
        }

        if (digitalRead(SETDOWNBUTTON_PIN) == LOW) {  
            lastPressTime = millis();
            currentString = (currentString - 1 + NUM_STRINGS) % NUM_STRINGS; 
            Serial.print("Tuning Down -> Current String: ");
            Serial.println(currentString);
        }

    }
}



void adjustTuning(double freq, double targetFreq) {
    double tolerance = 1.5; 

  
    if (freq < targetFreq - tolerance) {
      Serial.println("Tuning Up"); //debug
      rotateClockwise();  // 
      delay(100);
    } 
    else if (freq > targetFreq + tolerance) {
      Serial.println("Tuning Down"); //debug
      rotateCounterClockwise(); 
      delay(100);
    } 
    else if(freq == 0) {
      stopMotor();
    }
    else { 
      Serial.println("String In Tune!"); //debug
      stopMotor();
      delay(5000);
    }
}
  
double getPeakFrequency() {
    double peakValue = 0;
    int peakIndex = 0;

    for (int i = 2; i < (SAMPLES / 2); i++) {
        if (vReal[i] > peakValue) {
            peakValue = vReal[i];
            peakIndex = i;
        }
    }

    double frequency = (peakIndex * (SAMPLING_FREQUENCY / SAMPLES));

    if (frequency > 700){
      frequency = 0.0;
    }
    else if (frequency < 20){
      frequency = 0.0;
    }

    // Store frequency in buffer
    frequencyBuffer[freqIndex] = frequency;
    freqIndex = (freqIndex + 1) % NUM_AVG;  // Circular buffer update

    for (int i = 0; i < NUM_AVG; i++) {
      Serial.println("Frequency Buffer: ");
      Serial.println(frequencyBuffer[i]);
    }

    if (!bufferFilled && freqIndex == 0) {
        bufferFilled = true;
    }

    // Determine the mode (most prevalent value)
    int count[NUM_AVG] = {0};  // Count occurrences
    double mode = frequencyBuffer[0];
    int maxCount = 1;

    for (int i = 0; i < NUM_AVG; i++) {
        int tempCount = 1;
        for (int j = i + 1; j < NUM_AVG; j++) {
            // if (abs(frequencyBuffer[i] - frequencyBuffer[j]) < 1.0) {  // Small tolerance for floating point
            //     tempCount++;
            // }
        }
        if (tempCount > maxCount) {
            maxCount = tempCount;
            mode = frequencyBuffer[i];
        }
    }

    // If no mode (all different), return median instead
    if (maxCount == 1) {
        double sorted[NUM_AVG];
        memcpy(sorted, frequencyBuffer, sizeof(frequencyBuffer));
        std::sort(sorted, sorted + NUM_AVG);
        return sorted[NUM_AVG / 2];  // Median value
    }
    if (mode == targetOctaves[NUM_STRINGS]){
      std::fill(frequencyBuffer, frequencyBuffer + NUM_AVG, 0.0);
    }
    return mode;  // Return the most frequent value
    
}




void loop() {

    if (digitalRead(SETTUNER) == LOW) {  // If SET button is pressed
        Serial.println("Tune Set");
    } else {
        Serial.println("Tune not set");
        delay(100); 
        return;
    }
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


}
