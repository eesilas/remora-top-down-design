#!/usr/bin/env python3
import time
import spidev
import threading
try:
    import Jetson.GPIO as GPIO  # NVIDIA Orin Nano
except ImportError:
    import RPi.GPIO as GPIO  # Fallback for Raspberry Pi

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
spi_lock = threading.Lock()  # Thread-safe SPI communication

# Precomputed timing constants for optimization
SWIPE_TIME = SWIPE_DISTANCE / 0.5  # Time for horizontal swipe
STEP_TIME = STEP_DOWN / 0.3  # Time for vertical step

def read_sensor_data():
    """Read ultrasonic sensor data from Arduino via SPI"""
    with spi_lock:
        response = spi.xfer2([0x01])  # Request sensor data (0x01 = command)
        distance = int.from_bytes(response[1:3], byteorder='big') / 100.0  # cm to meters
    return distance

def send_motor_command(direction, speed):
    """Send motor control command to Arduino via SPI"""
    with spi_lock:
        cmd = [0x02, direction, int(speed * 100)]  # 0x02 = motor command
        spi.xfer2(cmd)

def approach_wall(target_distance=0.1, max_attempts=100):
    """Use thrusters to approach the wall at a safe distance"""
    attempts = 0
    while attempts < max_attempts:
        current_dist = read_sensor_data()
        if current_dist <= target_distance:
            break
        # Move forward slowly
        send_motor_command(1, 0.2)  # (1 = forward, 0.2 = 20% speed)
        time.sleep(0.1)
        attempts += 1
    send_motor_command(0, 0)  # Stop

def clean_wall():
    """Main cleaning algorithm: Swipe left-right, step down"""
    for swipe in range(MAX_SWIPES):
        # Move right
        send_motor_command(3, 0.5)  # (3 = right, 0.5 = 50% speed)
        time.sleep(SWIPE_TIME)  # Use precomputed time
        
        # Step down
        send_motor_command(2, 0.3)  # (2 = down, 0.3 = 30% speed)
        time.sleep(STEP_TIME)  # Use precomputed time
        
        # Move left
        send_motor_command(4, 0.5)  # (4 = left)
        time.sleep(SWIPE_TIME)  # Use precomputed time
        
        # Step down
        send_motor_command(2, 0.3)
        time.sleep(STEP_TIME)  # Use precomputed time

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
