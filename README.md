# remora-top-down-design

Design Remora workforce robot in top-down approach methods

write a main control program for running in nvidia orin nano connected by SPI bus with arduino mega 2560 pro, where arduino mega 2560 pro has codes to get sensor measure distance from the wall, and arduino mega 2560 pro also contain codes to control thruster motor rotation direction and rotation speed in its control program, build the software structure of nvidia orin nano program that automatically run when boot up the power of orin nano, it continuouly get data from ultrasound sensors and thruster motor approach the sub-carrier to the wall, there are 4 brushes on the sub-carrier, so it is a cleaning robot, control the movement of the sub-carrier to clean a 3 meter by 3 meter wall (the wall size may change from time to time when Remora do it's work, e.g. 4 meter by 5 meter).

Here's a structured software architecture for an **NVIDIA Orin Nano**-based cleaning robot controller that communicates with an **Arduino Mega 2560 Pro** over **SPI** for sensor data and motor control.  

Bill of Materials (BOM) of sub carrier 
- Water resisted sub carrier body 
- Underwater Ultrasonic Obstacle Avoidance Sensor (hereafter "UUOAS") -3m Arduino WiKi - DFRobot SKU:SEN0598 x 7
- Brushless underwater motor for driving brush (hereafter "thruster motor") x 8
- Connection wire between NVIDIA Orin Nano and Arduino Mega 2560 Pro
- Left and Right sub carrier body limit switch x 4


### **System Overview**
1. **NVIDIA Orin Nano** (Main Controller)  
   - Runs a **Python-based control program** (or C++ if preferred)  
   - Communicates with Arduino via **SPI**  
   - Processes **UUOAS data** for wall distance from sub carrier to the wall
   - Controls **4 brush motors** and **thruster motors**  
   - Implements **autonomous wall-cleaning algorithm**  

2. **Arduino Mega 2560 Pro** (Peripheral Controller)  
   - Reads **ultrasonic sensors** (UUOAS or similar)  
   - Controls **thruster motors** (ESC-based)  
   - Reports sensor data to Orin Nano  

### **Software Architecture (Orin Nano)**
#### **1. Boot Automation (systemd Service)**
To run the program automatically on boot:  

```bash
sudo nano /etc/systemd/system/cleaning_robot.service
```
**File Contents:**
```ini
[Unit]
Description=Cleaning Robot Controller
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/orin/robot_control/main.py
Restart=always
User=orin

[Install]
WantedBy=multi-user.target
```
**Enable the service:**
```bash
sudo systemctl enable cleaning_robot.service
sudo systemctl start cleaning_robot.service
```
#### **2. Main Control Program (`main.py`)**
```python
#!/usr/bin/env python3
import time
import spidev
import RPi.GPIO as GPIO  # (if using GPIO for SPI CS)

# Constants
WALL_WIDTH = 3.0  # meters
WALL_HEIGHT = 3.0  # meters
SWIPE_DISTANCE = 1.4  # meters (horizontal movement per swipe)
STEP_DOWN = 0.2  # meters (vertical step after each swipe)
MAX_SWIPES = int(WALL_HEIGHT / STEP_DOWN)

# SPI Setup
spi = spidev.SpiDev()
spi.open(0, 0)  # (bus 0, device 0)
spi.max_speed_hz = 1000000  # 1 MHz SPI speed

def read_sensor_data():
    """Read ultrasonic sensor data from Arduino via SPI"""
    response = spi.xfer2([0x01])  # Request sensor data (0x01 = command)
    distance = int.from_bytes(response[1:3], byteorder='big') / 100.0  # cm to meters
    return distance

def send_motor_command(direction, speed):
    """Send motor control command to Arduino via SPI"""
    cmd = [0x02, direction, int(speed * 100)]  # 0x02 = motor command
    spi.xfer2(cmd)

def approach_wall(target_distance=0.1):
    """Use thrusters to approach the wall at a safe distance"""
    while True:
        current_dist = read_sensor_data()
        if current_dist <= target_distance:
            break
        # Move forward slowly
        send_motor_command(1, 0.2)  # (1 = forward, 0.2 = 20% speed)
        time.sleep(0.1)
    send_motor_command(0, 0)  # Stop

def clean_wall():
    """Main cleaning algorithm: Swipe left-right, step down"""
    for swipe in range(MAX_SWIPES):
        # Move right
        send_motor_command(3, 0.5)  # (3 = right, 0.5 = 50% speed)
        time.sleep(SWIPE_DISTANCE / 0.5)  # Time = distance / speed
        
        # Step down
        send_motor_command(2, 0.3)  # (2 = down, 0.3 = 30% speed)
        time.sleep(STEP_DOWN / 0.3)
        
        # Move left
        send_motor_command(4, 0.5)  # (4 = left)
        time.sleep(SWIPE_DISTANCE / 0.5)
        
        # Step down
        send_motor_command(2, 0.3)
        time.sleep(STEP_DOWN / 0.3)

def main():
    try:
        print("Starting Cleaning Robot...")
        approach_wall()  # Get close to the wall
        clean_wall()     # Perform cleaning pattern
        print("Cleaning Complete!")
    except KeyboardInterrupt:
        print("Stopping Robot...")
    finally:
        spi.close()

if __name__ == "__main__":
    main()
```

---

### **3. Arduino Mega 2560 Pro Code (SPI Slave)**
```cpp
#include <SPI.h>

// Motor & Sensor Pins
#define THRUSTER_PIN 9   // ESC control
#define TRIG_PIN 10      // Ultrasonic sensor
#define ECHO_PIN 11      // Ultrasonic sensor

volatile uint8_t command = 0;
volatile int motorSpeed = 0;
volatile uint8_t motorDir = 0;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  SPCR |= _BV(SPE);      // Enable SPI Slave
  SPI.attachInterrupt();  // Enable SPI interrupt

  pinMode(THRUSTER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

ISR (SPI_STC_vect) {
  command = SPDR;  // Read SPI command
  if (command == 0x01) {
    // Send sensor data
    float distance = readUltrasonic();
    uint16_t dist_cm = distance * 100;
    SPDR = highByte(dist_cm);  // Send high byte first
  } else if (command == 0x02) {
    // Motor control command
    motorDir = SPDR;  // Next byte = direction
    motorSpeed = SPDR; // Next byte = speed
    controlMotor(motorDir, motorSpeed);
  }
}

float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.000343 / 2;  // Convert to meters
}

void controlMotor(uint8_t dir, uint8_t speed) {
  // Implement motor control logic (ESC, brush motors, etc.)
  analogWrite(THRUSTER_PIN, speed);
}

void loop() {
  // Main loop is minimal since SPI is interrupt-driven
}
```

---

### **4. Key Features**
✅ **Autonomous Wall Cleaning**  
- Swipe left-right pattern with step-down movement  
- Adjustable speed and distance  

✅ **SPI Communication**  
- Orin Nano (Master) sends motor commands  
- Arduino (Slave) sends sensor data
- add multi-threading lock during communicating between master and slave to avoid concurrent actions

✅ **Safe Wall Approach**  
- Ultrasonic sensors act as positioning to sub carrier itself to prevent collisions
- On/Off switch at the side of sub carrier to detect touch of wall on both left and right, the switch is SPST switch to trigger close when touch the wall, and normally open circuit
- Neopixel LED light ring indicates status of the sub carrier

✅ **Boot Automation**  
- Runs automatically on startup  

---

### **Next Steps**
0. Side touch switch
   - There are left and right side touch switch at both side, left and right, at the middle of sub carrier, it is an on/off switch, which is each switch connected to a GPIO pin of arduino mega 2560 pro to ground, and when the sub carrier phycally touch the left side wall, left switch circuit closeand trigger the corresponding GPIO connected to ground of the arduino mega 2560 pro circuit, the same, hen the sub carrier phycally touch the right side wall, right switch circuit close and trigger the corresponding GPIO connected to ground of the arduino mega 2560 pro circuit.
   - 
2. **Test SPI Communication**  
   - Verify data exchange between Orin Nano and Arduino.  
3. **Calibrate Motor Speeds**  
   - Adjust PWM values for thrusters and brush motors.  
4. **Add Emergency Stop**  
   - Implement a kill switch (GPIO button).


Would you like any modifications (e.g., ROS integration, PID control for smoother movement)? 🚀
