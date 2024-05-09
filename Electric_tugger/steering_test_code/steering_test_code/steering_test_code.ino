#include <Wire.h>
#include "AS5600.h"
#include <PID_v1.h>

AS5600 as5600;

// Define PID constants
double Kp = 0.2;//0.2
double Ki = 0.01;//0.01
double Kd = 0.001;//0.001

// Define variables for PID
double setpoint = 0.0;
double input, output;

// Define sensor offset
int maxMotorSpeed = 200; 
int sensorOffset = 1988;
int min = 1074;
int max = 2731;
// Define PID object
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Motor control pins
const int motorDirectionPin = 55; // Motor direction pin
#define motorSpeedPin  5    // Motor speed control pin

// RC receiver input pin
const int receiverPin = 2; // Example pin, change as per your setup

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
  Wire.begin();

  // Set motor control pins as outputs
  pinMode(motorDirectionPin, OUTPUT);
  pinMode(motorSpeedPin, OUTPUT);

  // Initialize PID
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255); // Adjust output limits based on your motor driver
  pid.SetSampleTime(20);          // Set PID sample time (ms)
}

void loop() {
  // Read RC receiver input
  int receiverValue = pulseIn(receiverPin, HIGH, 20000); // Change timeout as needed

  // Map receiver input to setpoint (example mapping)
  // setpoint = map(receiverValue, 1000, 2000, 0, 360); // Map receiver input to angle range (0-360 degrees)
  setpoint = 0;

  // Read encoder value (AS5600)
  int encoderValue = as5600.readAngle();

  // Apply sensor offset
  encoderValue -= sensorOffset;

  // Convert encoder value to degrees (assuming AS5600 output is in 0-4095 range)
  double angle = map(encoderValue, 0, 4095, 0, 360);

  // Compute PID
  input = angle;
  pid.Compute();

  int scaledOutput = map(abs(output), 0, 200, 0, maxMotorSpeed);

  // Control motor speed based on PID output
  if (angle < setpoint) {
     digitalWrite(motorDirectionPin, HIGH); // Set motor direction forward
    digitalWrite(motorSpeedPin, abs(scaledOutput)); // Set motor speed
  } else {
     digitalWrite(motorDirectionPin, LOW); // Set motor direction reverse
    digitalWrite(motorSpeedPin, abs(scaledOutput)); // Set motor speed
  }

  // Print debug information
  Serial.print("Output: ");
Serial.print(output);
Serial.print("Scaled Output: ");
Serial.println(scaledOutput);
  // Serial.print("Angle: ");
  // Serial.print(angle);
  // Serial.print(" degrees, Setpoint: ");
  // Serial.print(setpoint);
  // Serial.print(" degrees, Output: ");
  // Serial.println(output);
//   analogWrite(motorSpeedPin, 0);
// analogWrite(motorDirectionPin, 0);
// delay(2000);
// analogWrite(motorDirectionPin, 255);
//   delay(2000); // Adjust delay based on PID sample time
}
