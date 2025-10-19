#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h"
#include "Wire.h"
#include <PID_v1.h>

// ----------------- Motor & Sensor Setup -----------------
SCMD myMotorDriver;
const int PhotoSens = A5; // Analog photo sensor

// PID parameters
double Setpoint, Input, Output;
double Kp = 1.5, Ki = 1.0, Kd = 0.05;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Encoder variables
volatile unsigned long tickCount = 0;
unsigned long lastTickCount = 0;
unsigned long lastTime = 0;

// Motor settings
#define MOTOR 0
#define UPDATE_INTERVAL 100 // PID update interval in ms
#define MIN_OUTPUT 20       // Minimum PWM to keep motor spinning

// ----------------- Hysteresis thresholds -----------------
#define HIGH_THRESHOLD 280   // Above this = light mark
#define LOW_THRESHOLD 230    // Below this = dark background
bool sensorHigh = false;

// ----------------- Setup -----------------
void setup() {
  pinMode(PhotoSens, INPUT);
  Serial.begin(9600);

  // Initialize motor driver
  myMotorDriver.settings.commInterface = I2C_MODE;
  myMotorDriver.settings.I2CAddress = 0x5D;
  myMotorDriver.settings.chipSelectPin = 10;

  while (myMotorDriver.begin() != 0xA9) {
    Serial.println("ID mismatch, trying again...");
    delay(500);
  }
  Serial.println("Motor driver initialized.");

  while (!myMotorDriver.ready());
  Serial.println("Driver ready.");

  myMotorDriver.inversionMode(0, 1);
  myMotorDriver.enable();

  // PID setup
  Setpoint = 40.0;  // Target speed in ticks/sec
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(UPDATE_INTERVAL);
}

// ----------------- Loop -----------------
void loop() {
  // --- Read sensor value ---
  int LightValue = analogRead(PhotoSens);

  // --- Encoder pulse detection with hysteresis ---
  if (!sensorHigh && LightValue > HIGH_THRESHOLD) {
    tickCount++;       // Rising edge detected
    sensorHigh = true;
  } 
  else if (sensorHigh && LightValue < LOW_THRESHOLD) {
    sensorHigh = false; // Wait for next rising edge
  }

  // --- PID update every UPDATE_INTERVAL ---
  unsigned long now = millis();
  if (now - lastTime >= UPDATE_INTERVAL) {
    unsigned long ticks = tickCount - lastTickCount;
    lastTickCount = tickCount;
    lastTime = now;

    // Convert to ticks/sec
    Input = (double)ticks * (1000.0 / UPDATE_INTERVAL);

    // Compute PID
    myPID.Compute();

    // Apply PWM to motor
    int motorSpeed = constrain((int)Output, MIN_OUTPUT, 150);
    myMotorDriver.setDrive(MOTOR, 0, motorSpeed);

    // Serial output: display speed, PWM, and sensor value
    Serial.print("Measured Speed (ticks/sec): ");
    Serial.print(Input);
    Serial.print(" | Target: ");
    Serial.print(Setpoint);
    Serial.print(" | Output (PWM): ");
    Serial.print(motorSpeed);
    Serial.print(" | Sensor Value: ");
    Serial.println(LightValue);
  }
  // --- Check for Serial input to change setpoint ---
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim();

    // Expect input like: S50  (meaning setpoint = 50)
    if (inputString.startsWith("S")) {
      double newSetpoint = inputString.substring(1).toDouble();
      if (newSetpoint >= 0 && newSetpoint <= 200) { // adjust range as needed
        Setpoint = newSetpoint;
        Serial.print("✅ New setpoint: ");
        Serial.println(Setpoint);
      } else {
        Serial.println("⚠️ Invalid setpoint range (0-200).");
      }
    }
  }
}




