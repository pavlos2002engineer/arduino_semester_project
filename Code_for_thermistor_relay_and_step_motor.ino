// The code for the Thermistor, relay and step-motor.

#include <Stepper.h>
#include <LiquidCrystal_I2C.h>

// Motor settings
const int motorStepsPerRevolution = 2048;
Stepper myStepper(motorStepsPerRevolution, 8, 10, 9, 11);

// Relay settings
const int pinFeedback = 13; 
const int pinFault = 7;   
const int power = 3;

// Temperature sensor settings
const int thermistorPin = A1;
const float R_pullup = 2250.0; // Pull-up resistor value in ohms (2.25k)
const float R0 = 2000.0;       // Thermistor resistance at 25°C
const float Beta = 3250.0;     // Beta coefficient
const float T0 = 298.15;       // Reference temperature in Kelvin (25°C)
const float Vcc = 5.0;
const float baseHOT = 22;
const float baseCOLD = 19;
const int numSamples = 10;     // Number of samples for averaging
int ledHOT = 6;
int ledCOLD = 5;

// Motor variables
int pwmValue = 0;
int currentStep = 0;
int motorDirection = 1;       // Motor direction
unsigned long lastMotorUpdate = 0;
const unsigned long motorInterval = 2;   // Time between motor steps (ms)
const unsigned long motorChangeDelay = 4000; // Delay when changing direction (ms)
bool motorDelayActive = false;
unsigned long motorDelayStartTime = 0;

// Timing variables
unsigned long lastRelayCheck = 0;
unsigned long lastTempCheck = 0;
const unsigned long relayInterval = 100; // Time interval to check relay (ms)
const unsigned long tempInterval = 5500; // Time interval to check temperature (ms)

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Function to read and average analog values
float readAverage() {
  float sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(thermistorPin);
    delay(10); // Small delay for stable readings
  }
  return sum / numSamples;
}

void setup() {
  // Motor setup
  pinMode(A0, INPUT);

  // Relay setup
  pinMode(pinFeedback, INPUT);
  pinMode(pinFault, OUTPUT);
  pinMode(power, INPUT);

  // Temperature LEDs setup
  pinMode(ledHOT, OUTPUT);
  pinMode(ledCOLD, OUTPUT);

  // LCD setup
  lcd.begin();
  lcd.backlight();

  Serial.begin(9600);
  Serial.println("System Initialized");

  myStepper.setSpeed(10); // Initial speed
}

void loop() {
  unsigned long currentMillis = millis();

  // --- Temperature Check ---
  if (currentMillis - lastTempCheck >= tempInterval) {
    lastTempCheck = currentMillis;

    // Read temperature sensor
    float avgAnalog = readAverage();
    float Vout = avgAnalog * (5.0 / 1023.0);
    float R_thermistor = (Vout * R_pullup) / (Vcc - Vout);
    float tempK = 1 / ((1 / T0) + (1 / Beta) * log(R_thermistor / R0));
    float tempC = tempK - 273.15; // Convert to Celsius

    // Update LEDs based on temperature
    if (tempC >= baseHOT){
      digitalWrite(ledHOT, HIGH);
    } else {
      digitalWrite(ledHOT, LOW);
    }

    if (tempC <= baseCOLD){
      digitalWrite(ledCOLD, HIGH);
    } else {
      digitalWrite(ledCOLD, LOW);
    }

    // Update LCD with temperature
    lcd.setCursor(0, 0);
    lcd.print("Temperature ");
    lcd.setCursor(0, 1);
    lcd.print(tempC);
    lcd.print(" C  ");
    }

  // --- Motor Control ---
  int sensorValue = analogRead(A0);
  pwmValue = map(sensorValue, 0, 1023, 50, 255);
  int motorSpeed = map(pwmValue, 50, 255, 5, 18);
  myStepper.setSpeed(motorSpeed);

  if (motorDelayActive == true) {
    // Handle delay for direction change without blocking
    if (currentMillis - motorDelayStartTime >= 4000) {
      motorDelayActive = false;
    }
  } else if (currentMillis - lastMotorUpdate >= motorInterval) {
    lastMotorUpdate = currentMillis;
    myStepper.step(motorDirection); // Move motor
    currentStep += motorDirection;
   

    if (currentStep >= motorStepsPerRevolution / 4 || currentStep <= -motorStepsPerRevolution / 4) {
      motorDirection = -motorDirection;  // Reverse direction
      motorDelayActive = true;
      motorDelayStartTime = currentMillis;
      Serial.println("[Motor] Direction Changed");
      
    }
  }

  // --- Relay Check ---
  if (currentMillis - lastRelayCheck >= relayInterval) {
    lastRelayCheck = currentMillis;

    if (digitalRead(power) == HIGH) {
      if (digitalRead(pinFeedback) == HIGH) {
      digitalWrite(pinFault, LOW);  // Feedback is HIGH -> No fault
      } else {
      digitalWrite(pinFault, HIGH); // Feedback is LOW -> Fault detected
      }
    }
  }
}



#include <LiquidCrystal_I2C.h>

// Define pins for the ultrasonic sensor
const int trigPin = 13;
const int echoPin = 12;

// Define pins for LEDs
const int ledPins[] = {10, 9, 8, 7, 6, 5}; // Change according to your wiring
const int numLeds = sizeof(ledPins) / sizeof(ledPins[0]);

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  lcd.begin();

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize LED pins
  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW); // Start with LEDs off
  }
}

void loop() {
  // Measure distance
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2; // Convert to cm

  // Debugging: Print distance
  Serial.println(distance);
  lcd.setCursor(0,0);
  lcd.print(distance);

  // Determine the number of LEDs to light up based on distance
  int ledsToLight = map(distance, 2, 50, numLeds, 0); // Map 5-50 cm to all LEDs
  ledsToLight = constrain(ledsToLight, 0, numLeds);

  // Turn on the corresponding number of LEDs
  for (int i = 0; i < numLeds; i++) {
    if (i < ledsToLight) {
      digitalWrite(ledPins[i], HIGH);
    } else {
      digitalWrite(ledPins[i], LOW);
    }
  }

  delay(100); // Short delay to stabilize readings
}
