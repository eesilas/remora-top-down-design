https://learn.adafruit.com/adafruit-neopixel-uberguide/downloads

A neopixel is the status indicator light of the sub carrier
color code: red, orange, yellow, green, blue, purple, white 
The code run in nvidia orin nano and the neopixel (signal, power Vcc, Gnd) connected to it

sample code
/*
 * @file        RGB ring.ino
 * @brief       The colors of the 12 lamp beads are displayed in gradients. Adjust the brightness of the RGB LED ring using the analog rotation sensor.     
 */

#include <DFRobot_NeoPixel.h>

// Dynamic variable
volatile float brightness;
// Create an Object
DFRobot_NeoPixel neoPixel_2;


// Main program start
void setup() {
  neoPixel_2.begin(2, 24);
  brightness = 0;
}
void loop() {
  neoPixel_2.setBrightness((map(brightness, 0, 749, 0, 255)));
  neoPixel_2.showRainbow(0, 23, 1, 360);
  brightness = analogRead(A0);
}
