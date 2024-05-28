// Include iBus Library
#include <IBusBM.h>
#include <AS5600.h>
#include <PID_v1.h>
#include <Wire.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro
// Create iBus Object
IBusBM ibus;

#define PWM 5
#define DIR A1

int rcCH1 = 0;
int throttle_val = 0; 
int maxthrottlefrontspeed = 65;
int maxthrottlebackspeed = 95;
int rcCH3 = 0;
int steer = 0; 
int throttlespeedch = 0;
int rcCH6 = 0;
int key = 0;
int IGN = 0;
int SW = 0;
bool rcCH10 = 0;
int throttle;
int throttle1;
// Define pin numbers for motor driver inputs
const int brakepwmPin = 6;    // PWM pin for speed control
const int brakedirPin = 54;    // Direction pin
const int limitSwitch1 = 3;  // Limit switch 1 (fully extended)
const int limitSwitch2 = 4;  // Limit switch 2 (fully retracted)
// Define variables for motor speed and direction
int brakespeed = 255;  // Adjust speed as needed (0-255 for PWM)
bool brakedirection = HIGH;  // HIGH for extending, LOW for retracting

AS5600 as5600;   //  use default Wire

// Define PID constants for steering
double Kp = 2.6;//2.6
double Ki = 0.000001;//0.000001
double Kd = 0.0000001;//0.0000001

// Define variables for PID
double setpoint = 0.0;
double input, output;

// Define sensor offset
int maxMotorSpeed = 100; 
int sensorOffset = 1988;
int min = 1074;
int max = 2731;
// Define PID object
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Motor control pins
const int motorDirectionPin = 55; // Motor direction pin
#define motorSpeedPin  5    // Motor speed control pin

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void setup() {
  // put your setup code here, to run once:
Wire.begin();
as5600.begin();  // Initialize AS5600 sensor

pinMode(brakepwmPin, OUTPUT);
pinMode(brakedirPin, OUTPUT);
  // Initialize limit switch pins
pinMode(limitSwitch1, INPUT_PULLUP);
pinMode(limitSwitch2, INPUT_PULLUP);

pinMode(PWM, OUTPUT);
pinMode(DIR, OUTPUT);

pinMode(23, OUTPUT);
pinMode(2, OUTPUT);
pinMode(12, OUTPUT);
pinMode(8, OUTPUT);
pinMode(11, OUTPUT);
pinMode(7, OUTPUT);
pinMode(motorSpeedPin, OUTPUT);
pinMode(motorDirectionPin, OUTPUT);
digitalWrite(23, LOW);
Serial.begin(115200);
ibus.begin(Serial1);
// Initialize PID
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255); // Adjust output limits based on your motor driver
  pid.SetSampleTime(10);          // Set PID sample time (ms)-20
}

void loop() {
  // Get RC channel values
  rcCH1 = readChannel(0, -100, 100, 0);
  throttle_val = readChannel(1, -100, 100, 0);
  rcCH3 = readChannel(2, 0, 155, 0);
  steer = readChannel(3, -80, 80, 0);
  throttlespeedch = readChannel(4, 1000, 2000, 0);
  rcCH6 = readChannel(5, -100, 100, 0);
  key = readSwitch(6, true);
  IGN = readSwitch(7, true);
  SW = readChannel(8, -100, 100, 0);
  rcCH10 = readSwitch(9, true);

    // Print values to serial monitor for debugging
  Serial.print("Ch1 = ");
  Serial.print(rcCH1);

  // Serial.print(" Ch2 = ");
  // Serial.print(throttle_val);

  Serial.print(" Ch3 = ");
  Serial.print(rcCH3);

  Serial.print(" throttle_val = ");
  Serial.print(throttle_val);

  Serial.print(" throttlespeedch = ");
  Serial.print(throttlespeedch);

  Serial.print(" sw = ");
  Serial.print(SW);

///////////////////////  KEY  ///////////////
if(key ==0){
  digitalWrite(23,HIGH);
}
else if(key == 1){
  digitalWrite(23,LOW);
}
/////////////////// IGN. ///////////////////////
if(IGN == 1){
  digitalWrite(12,LOW);
}
else if(IGN == 0){
  digitalWrite(12,HIGH);
}
//////////////////// Throttle/reverse ////////////////
//low speed
digitalWrite(8,LOW);
digitalWrite(11,HIGH);
// //mid speed
// digitalWrite(8,HIGH);
// digitalWrite(11,HIGH);
// //high speed
// digitalWrite(8,HIGH);
// digitalWrite(11,LOW);

//int throttlespeed = map(throttlespeedch,1000,2000,0,180);
int throttlefrontspeed = maxthrottlefrontspeed;
int throttlebackspeed = maxthrottlebackspeed;
if (throttle_val == 0 || rcCH10 == LOW){
analogWrite(2,0);
}

else if (throttle_val > 0 && key ==1){
digitalWrite(7,HIGH);
throttle = map (throttle_val,0,100,49,throttlefrontspeed);
analogWrite(2,throttle);
}
else if (throttle_val < 0 && key ==1){
digitalWrite(7,LOW);
throttle1 = map (throttle_val,0,-100,49,throttlebackspeed);
analogWrite(2,throttle1);
}
//////////////////// Speed select /////////////
/*if(SW == 0){
  digitalWrite(8,HIGH);
  digitalWrite(11,HIGH);
}
else if(SW < -20){
  digitalWrite(8,LOW);
  digitalWrite(11,HIGH);
}
else if(SW > 20){
  digitalWrite(8,HIGH);
  digitalWrite(11,LOW);
}*/
////////////////// Steering //////////////////////////////////

setpoint = steer;

 int encoderValue = as5600.readAngle();

  // Apply sensor offset
  encoderValue -= sensorOffset;

  // Convert encoder value to degrees (assuming AS5600 output is in 0-4095 range)
  double angle = map(encoderValue, 0, 4095, 0, 360);

  // Compute PID
  input = angle;
  pid.Compute();

  int scaledOutput = map(abs(output), 0, 255, 0, maxMotorSpeed);
  // float scaledOutput = fabs(output);
  // if (scaledOutput > 255) {
  //   scaledOutput = 80;
  // }
  // Control motor speed based on PID output
  if (angle < setpoint) {
     digitalWrite(motorDirectionPin, HIGH); // Set motor direction forward
    digitalWrite(motorSpeedPin, scaledOutput); // Set motor speed
  } else {
     digitalWrite(motorDirectionPin, LOW); // Set motor direction reverse
    digitalWrite(motorSpeedPin, scaledOutput); // Set motor speed
  }
/////////////////////////////////// Brake ////////////////////////////////////////////////
bool canExtend = !digitalRead(limitSwitch1);
  bool canRetract = !digitalRead(limitSwitch2);

  // Move the brake actuator based on RC receiver switch state and limit switch conditions
  if (rcCH10 == HIGH && canExtend) {
    moveActuator(brakespeed, HIGH);  // Extend
  } else if (rcCH10 == LOW && canRetract) {
    moveActuator(brakespeed, LOW);  // Retract
  } else {
    stopActuator();  // Stop if limit reached or switch state invalid
  }

  // Serial.print(" Ch7 = ");
  // Serial.print(output);

  // Serial.print(" Ch8 = ");
  // Serial.print(IGN);

  // Serial.print(" Ch9 = ");
  // Serial.print(SW);

  Serial.print(" Ch10 = ");
  Serial.println(throttle1);
}
// Function to move the brake actuator with specified speed and direction
void moveActuator(int speedValue, bool dirValue) {
  analogWrite(brakepwmPin, speedValue);  // Set PWM for speed control
  digitalWrite(brakedirPin, dirValue);   // Set direction
}

// Function to stop the brake actuator
void stopActuator() {
  analogWrite(brakepwmPin, 0);  // Stop PWM
}
