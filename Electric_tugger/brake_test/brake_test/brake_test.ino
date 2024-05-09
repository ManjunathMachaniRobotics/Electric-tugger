#include <IBusBM.h>
// Define pin numbers for motor driver inputs
const int pwmPin = 6;    // PWM pin for speed control
const int dirPin = 54;    // Direction pin
const int limitSwitch1 = 3;  // Limit switch 1 (fully extended)
const int limitSwitch2 = 4;  // Limit switch 2 (fully retracted)
IBusBM ibus;
// Define variables for motor speed and direction
int speed = 255;  // Adjust speed as needed (0-255 for PWM)
bool direction = HIGH;  // HIGH for extending, LOW for retracting
bool rcCH10 = 0;

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
  // Initialize motor control pins
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  // Initialize limit switch pins
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
Serial.begin(115200);
ibus.begin(Serial1);

}

void loop() {
  // Read the state of the RC receiver switch

  rcCH10 = readSwitch(9, false);
  // Check the state of limit switches to determine the allowed movement direction
  bool canExtend = !digitalRead(limitSwitch1);
  bool canRetract = !digitalRead(limitSwitch2);

  // Move the actuator based on RC receiver switch state and limit switch conditions
  if (rcCH10 == HIGH && canExtend) {
    moveActuator(speed, HIGH);  // Extend
  } else if (rcCH10 == LOW && canRetract) {
    moveActuator(speed, LOW);  // Retract
  } else {
    stopActuator();  // Stop if limit reached or switch state invalid
  }

  // Add any additional control logic or input handling here
}

// Function to move the actuator with specified speed and direction
void moveActuator(int speedValue, bool dirValue) {
  analogWrite(pwmPin, speedValue);  // Set PWM for speed control
  digitalWrite(dirPin, dirValue);   // Set direction
}

// Function to stop the actuator
void stopActuator() {
  analogWrite(pwmPin, 0);  // Stop PWM
}
