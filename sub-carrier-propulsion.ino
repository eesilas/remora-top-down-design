#include <Servo.h>
#include <SPI.h>

// ESC signal pins for all 8 thrusters
const uint8_t ESC_PINS[8] = {2, 3, 4, 5, 6, 7, 8, 9};
const char THRUSTER_NAMES[8][5] = {"TR1", "UR2", "LR3", "BR4", "TL5", "UL6", "LL7", "BL8"};
const char VECTORS[8][12] = {"Up/Down", "Forth/Back", "Forth/Back", "Up/Down", "Up/Down", "Forth/Back", "Forth/Back", "Up/Down"};
const uint8_t NUM_THRUSTERS = 8;
const uint16_t PULSE_STOP = 1500;
const uint16_t PULSE_CW = 1600;
const uint16_t PULSE_CCW = 1400;

enum Movement : uint8_t {
  STOP = 0, UP, DOWN, LEFT, RIGHT, FRONT, BACK, SWIPE, ROTATE_CW
};

struct Thruster {
  Servo esc;
  uint8_t pin;
  uint16_t targetPulse;
  uint16_t currentPulse;
  unsigned long lastUpdate;
};

Thruster thrusters[NUM_THRUSTERS];
volatile Movement currentMovement = STOP;
volatile bool brushOn = false;

// SPI protocol
#define SPI_CMD_SIZE 4
#define SPI_RESP_SIZE 8
volatile uint8_t spiRxBuffer[SPI_CMD_SIZE];
volatile uint8_t spiTxBuffer[SPI_RESP_SIZE];

void setupThrusters() {
  for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
    thrusters[i].pin = ESC_PINS[i];
    thrusters[i].esc.attach(thrusters[i].pin, 1000, 2000);
    thrusters[i].targetPulse = PULSE_STOP;
    thrusters[i].currentPulse = PULSE_STOP;
    thrusters[i].lastUpdate = millis();
    thrusters[i].esc.writeMicroseconds(PULSE_STOP);
  }
}

void setThrusterPulse(uint8_t idx, uint16_t pulse) {
  if (idx < NUM_THRUSTERS) thrusters[idx].targetPulse = pulse;
}

void executeMovement(Movement mvmt) {
  currentMovement = mvmt;
  switch (mvmt) {
    case STOP:
      for (uint8_t i = 0; i < NUM_THRUSTERS; i++)
        setThrusterPulse(i, PULSE_STOP);
      break;
    case UP:
      setThrusterPulse(0, PULSE_CCW); setThrusterPulse(3, PULSE_CCW);
      setThrusterPulse(4, PULSE_CCW); setThrusterPulse(7, PULSE_CCW);
      setThrusterPulse(1, PULSE_STOP); setThrusterPulse(2, PULSE_STOP);
      setThrusterPulse(5, PULSE_STOP); setThrusterPulse(6, PULSE_STOP);
      break;
    case DOWN:
      setThrusterPulse(0, PULSE_CW); setThrusterPulse(3, PULSE_CW);
      setThrusterPulse(4, PULSE_CW); setThrusterPulse(7, PULSE_CW);
      setThrusterPulse(1, PULSE_STOP); setThrusterPulse(2, PULSE_STOP);
      setThrusterPulse(5, PULSE_STOP); setThrusterPulse(6, PULSE_STOP);
      break;
    case LEFT:
      setThrusterPulse(0, PULSE_CCW); setThrusterPulse(3, PULSE_CW);
      setThrusterPulse(4, PULSE_CW); setThrusterPulse(7, PULSE_CCW);
      setThrusterPulse(1, PULSE_STOP); setThrusterPulse(2, PULSE_STOP);
      setThrusterPulse(5, PULSE_STOP); setThrusterPulse(6, PULSE_STOP);
      break;
    case RIGHT:
      setThrusterPulse(0, PULSE_CW); setThrusterPulse(3, PULSE_CCW);
      setThrusterPulse(4, PULSE_CCW); setThrusterPulse(7, PULSE_CW);
      setThrusterPulse(1, PULSE_STOP); setThrusterPulse(2, PULSE_STOP);
      setThrusterPulse(5, PULSE_STOP); setThrusterPulse(6, PULSE_STOP);
      break;
    case FRONT:
      setThrusterPulse(1, PULSE_CW); setThrusterPulse(2, PULSE_CCW);
      setThrusterPulse(5, PULSE_CW); setThrusterPulse(6, PULSE_CCW);
      setThrusterPulse(0, PULSE_STOP); setThrusterPulse(3, PULSE_STOP);
      setThrusterPulse(4, PULSE_STOP); setThrusterPulse(7, PULSE_STOP);
      break;
    case BACK:
      setThrusterPulse(1, PULSE_CCW); setThrusterPulse(2, PULSE_CW);
      setThrusterPulse(5, PULSE_CCW); setThrusterPulse(6, PULSE_CW);
      setThrusterPulse(0, PULSE_STOP); setThrusterPulse(3, PULSE_STOP);
      setThrusterPulse(4, PULSE_STOP); setThrusterPulse(7, PULSE_STOP);
      break;
    // Add SWIPE, ROTATE_CW as needed
    default: break;
  }
}

void transferThrusterPulses() {
  for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
    if (thrusters[i].currentPulse != thrusters[i].targetPulse) {
      thrusters[i].currentPulse = thrusters[i].targetPulse;
      thrusters[i].esc.writeMicroseconds(thrusters[i].currentPulse);
    }
  }
}

void setup() {
  setupThrusters();
  // SPI Slave setup
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE); // SPI enable
  SPI.attachInterrupt();
}

ISR(SPI_STC_vect) {
  static uint8_t idx = 0;
  uint8_t rx = SPDR;
  spiRxBuffer[idx++] = rx;
  if (idx >= SPI_CMD_SIZE) {
    // Parse SPI command (move, brush, etc.)
    Movement mvmt = (Movement)spiRxBuffer[0];
    bool brushCmd = spiRxBuffer[1];
    executeMovement(mvmt);
    brushOn = brushCmd;
    // Prepare response: status, pulses
    spiTxBuffer[0] = currentMovement;
    spiTxBuffer[1] = brushOn;
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++)
      spiTxBuffer[2+i] = (uint8_t)((thrusters[i].currentPulse-1000)/2); // scale down for 8-bit
    idx = 0;
  }
  SPDR = spiTxBuffer[0]; // send first byte of response
}

void loop() {
  transferThrusterPulses();
  // Other periodic tasks as needed (brush control, safety, etc.)
}