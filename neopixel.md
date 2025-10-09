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


--------------
Here’s how to add NeoPixel status indication to your existing Python program for the NVIDIA Orin Nano. I’ll use the rpi_ws281x library, which is commonly used for NeoPixel control in Python on Linux (including Jetson/Orin boards). You’ll see colored indication for different robot statuses (startup, approaching wall, cleaning, complete, error/stop).

NeoPixel Wiring:

Data pin: Connect to a GPIO (e.g., GPIO18, physical pin 12)
Vcc: 5V
GND: Ground
NeoPixel Color Codes (RGB):

Red: (255, 0, 0)
Orange: (255, 128, 0)
Yellow: (255, 255, 0)
Green: (0, 255, 0)
Blue: (0, 0, 255)
Purple: (128, 0, 128)
White: (255, 255, 255)
Install the library:

bash
pip install rpi_ws281x
Or on Jetson, you may need to build from source (see Adafruit NeoPixel guide).


sub-carrier-main.py
#!/usr/bin/env python3
import time
import spidev
import RPi.GPIO as GPIO  # (if using GPIO for SPI CS)

# NeoPixel imports and setup
NeoPixel Status Mapping:

White: Startup
Orange: Approaching wall
Yellow: Ready to clean
Blue: Cleaning in progress
Green: Clean complete
Purple: All done
Red: Error/stop
