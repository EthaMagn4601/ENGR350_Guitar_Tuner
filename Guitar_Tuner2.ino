#include <Arduino.h>
#include "arduinoFFT.h"
#include "ESP32Servo.h"

#define MIC_PIN A0               
#define SERVO_PIN 13             
#define SETDOWNBUTTON_PIN 12
#define SETUPBUTTON_PIN 13
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
    110.00,// A
    146.83,// D
    196.00,// G
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
    case 0: Serial.println("Low E"); break; //Troubleshoot
    case 1: Serial.println("A"); break; //Troubleshoot
    case 2: Serial.println("D"); break; //Troubleshoot
    case 3: Serial.println("G"); break; //Troubleshoot
    case 4: Serial.println("B"); break; //Troubleshoot
    case 5: Serial.println("High E"); break; //Troubleshoot
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
    pinMode(SETUPBUTTON_PIN, INPUT); 
    pinMode(SETDOWNBUTTON_PIN, INPUT); 
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


void checkTune(){
    static unsigned long lastPressTime = 0;
    unsigned long currentTime = millis();

    if (digitalRead(SETUPBUTTON_PIN) == LOW && !buttonPressed) {
        if (currentTime - lastPressTime > 200) {  // Debounce
            buttonPressed = true;
            currentString = (currentString + 1) % NUM_STRINGS;
            Serial.println("Tuning Up");
            stringName(currentString);
            lastPressTime = currentTime;
        }
    } else if (digitalRead(SETUPBUTTON_PIN) == HIGH) {
        buttonPressed = false;
    }

//     if (digitalRead(SETDOWNBUTTON_PIN) == LOW &&!buttonPressed) {
//     buttonPressed = true;
//     currentString = (currentString - 1)% NUM_STRINGS;
//     Serial.print("tuning down"); //Troubleshoot
//     stringName(currentString);
//     delay(300);

// }
//     if (digitalRead(SETDOWNBUTTON_PIN)== HIGH){
//     buttonPressed= false;
// }

}

void adjustTuning(double freq, double targetFreq) {
    double tolerance = 3.0; 
  
    if (freq < targetFreq - tolerance) {
      Serial.println("Tuning Up..."); //Troubleshoot
      rotateClockwise();  // Increase tension
    } 
    else if (freq > targetFreq + tolerance) {
      Serial.println("Tuning Down..."); //Troubleshoot
      rotateCounterClockwise();  // Decrease tension
    } 
    else {
      Serial.println("String In Tune!"); //Troubleshoot
      stopMotor();
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
    checkTune();  // Check if tuning buttons are pressed

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
        Serial.println("No valid frequency detected.");
    }
    if (freq == 82.03) {
        Serial.println("IM IN TUNE");
        // delay(10000);
    }  else{
        delay(500);
      }  // Short delay for stability
}




