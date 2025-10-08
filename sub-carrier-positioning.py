#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Motor & Sensor Pins
#define THRUSTER_PIN 9   // ESC control
#define TRIG_PIN 10      // Ultrasonic sensor (unused in this version)
#define ECHO_PIN 11      // Ultrasonic sensor (unused in this version)

// Define sensor data structures
struct SensorData {
  unsigned char buffer[4] = {0};
  uint8_t CS;
  int Distance = -1;
  int lastStableDistance = -1;
  unsigned long stableStartTime = 0;
};

#define COM 0x55
#define NUM_SENSORS 3

// For the SSD1306 OLED display Module
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Array to hold sensor data
SensorData sensors[NUM_SENSORS];

// SPI variables
volatile uint8_t command = 0;
volatile int motorSpeed = 0;
volatile uint8_t motorDir = 0;

void setup() {
  // Initialize serial communications
  Serial.begin(115200);  // Serial0 for debugging via USB
  Serial1.begin(115200); // TX1 (Pin 18), RX1 (Pin 19)
  Serial2.begin(115200); // TX2 (Pin 16), RX2 (Pin 17)
  Serial3.begin(115200); // TX3 (Pin 14), RX3 (Pin 15)

  // Initialize SPI
  SPI.begin();
  SPCR |= _BV(SPE);      // Enable SPI Slave
  SPI.attachInterrupt();  // Enable SPI interrupt

  // Initialize motor control pin
  pinMode(THRUSTER_PIN, OUTPUT);

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }

  delay(2000);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
}

ISR (SPI_STC_vect) {
  command = SPDR;  // Read SPI command
  if (command == 0x01) {
    // Send sensor data - using the most stable sensor reading
    uint16_t dist_mm = sensors[0].lastStableDistance; // Default to first sensor
    SPDR = highByte(dist_mm);  // Send high byte first
  } else if (command == 0x02) {
    // Motor control command
    motorDir = SPDR;  // Next byte = direction
    motorSpeed = SPDR; // Next byte = speed
    controlMotor(motorDir, motorSpeed);
  }
}

void loop() {
  // Send command to all underwater ultrasound sensors
  Serial1.write(COM);
  Serial2.write(COM);
  Serial3.write(COM);
  delay(100);  // Delay between commands

  // Process each sensor
  processSensor(Serial1, sensors[0], "Sensor 1");
  processSensor(Serial2, sensors[1], "Sensor 2");
  processSensor(Serial3, sensors[2], "Sensor 3");

  // Update display with all three sensor values
  updateDisplay();
}

void processSensor(HardwareSerial &serial, SensorData &sensor, const char* sensorName) {
  if (serial.available() > 0) {
    delay(4);
    if (serial.read() == 0xff) {
      sensor.buffer[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        sensor.buffer[i] = serial.read();
      }
      sensor.CS = sensor.buffer[0] + sensor.buffer[1] + sensor.buffer[2];

      if (sensor.buffer[3] == sensor.CS) {
        int newDistance = (sensor.buffer[1] << 8) + sensor.buffer[2];

        if (sensor.Distance == -1) {  
          // First valid reading, set immediately
          sensor.Distance = newDistance;
          sensor.lastStableDistance = newDistance;
          sensor.stableStartTime = millis();
        } else if (newDistance == sensor.Distance) {
          // Check if stable for 500ms
          if (millis() - sensor.stableStartTime >= 500 && newDistance != sensor.lastStableDistance) {
            sensor.lastStableDistance = newDistance;
          }
        } else {
          // Reset stability timer on change
          sensor.Distance = newDistance;
          sensor.stableStartTime = millis();
        }

        Serial.print(sensorName);
        Serial.print(" Distance: ");
        Serial.print(sensor.lastStableDistance);
        Serial.println("mm");
      }
    }
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setCursor(10, 5);
  display.println("Distances:");
  
  // Display each sensor's stable distance
  display.setCursor(10, 25);
  display.print("1: ");
  display.print(sensors[0].lastStableDistance);
  display.print("mm");
  
  display.setCursor(10, 45);
  display.print("2: ");
  display.print(sensors[1].lastStableDistance);
  display.print("mm");
  
  display.setCursor(10, 65);
  display.print("3: ");
  display.print(sensors[2].lastStableDistance);
  display.print("mm");
  
  display.display();
}

void controlMotor(uint8_t dir, uint8_t speed) {
  // Implement motor control logic (ESC, brush motors, etc.)
  // You might want to add direction control here if needed
  analogWrite(THRUSTER_PIN, speed);
}
