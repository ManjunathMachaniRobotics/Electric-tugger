/*
  RC Remote Car
  fsi6x-RC-car-spin.ino
  Uses Flysky FS-I6X RC receiver & FS-I6X 6-ch Controller
  Uses TB6612FNG H-Bridge Motor Controller
  Drives two DC Motors

  Two Drive Modes - Normal and Spin
  Map channel 6 on controller to switch SWA for mode control

  Right stick controls direction in Normal mode (CH1 & CH2)
  VRA controls speed and direction in Spin mode (CH5)
  Left stick is acceleration in both modes (CH3)

  Channel functions by Ricardo Paiva - https://gist.github.com/werneckpaiva/

  DroneBot Workshop 2021
  https://dronebotworkshop.com
*/

// Include iBus Library
#include <IBusBM.h>

// Create iBus Object
IBusBM ibus;

// Channel Values

int rcCH1 = 0; // Left - Right
int rcCH2 = 0; // Forward - Reverse
int rcCH3 = 0; // Acceleration
int rcCH4 = 0; // Spin Control
int rcCH5 = 0;
int rcCH6 = 0;
int rcCH7 = 0;
int rcCH8 = 0;
int rcCH9 = 0;
bool rcCH10 = 0; // Mode Control


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

void setup()

{
  // Start serial monitor for debugging
  Serial.begin(115200);

  // Attach iBus object to serial port
  ibus.begin(Serial1);

}

void loop() {

  // Get RC channel values
  rcCH1 = readChannel(0, -100, 100, 0);
  rcCH2 = readChannel(1, -100, 100, 0);
  rcCH3 = readChannel(2, 0, 155, 0);
  rcCH4 = readChannel(3, -100, 100, 0);
  rcCH5 = readChannel(4, -100, 100, 0);
  rcCH6 = readChannel(5, -100, 100, 0);
  rcCH7 = readChannel(6, -100, 100, 0);
  rcCH8 = readChannel(7, -100, 100, 0);
  rcCH9 = readChannel(8, -100, 100, 0);
  rcCH10 = readSwitch(9, false);

  // Print values to serial monitor for debugging
  Serial.print("Ch1 = ");
  Serial.print(rcCH1);

  Serial.print(" Ch2 = ");
  Serial.print(rcCH2);

  Serial.print(" Ch3 = ");
  Serial.print(rcCH3);

  Serial.print(" Ch4 = ");
  Serial.print(rcCH4);

  Serial.print(" Ch5 = ");
  Serial.print(rcCH5);

  Serial.print(" Ch6 = ");
  Serial.print(rcCH6);

  Serial.print(" Ch7 = ");
  Serial.print(rcCH7);

  Serial.print(" Ch8 = ");
  Serial.print(rcCH8);

  Serial.print(" Ch9 = ");
  Serial.print(rcCH9);

  Serial.print(" Ch10 = ");
  Serial.println(rcCH10);

}