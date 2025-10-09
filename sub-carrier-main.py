#!/usr/bin/env python3
import time
import spidev
import Jetson.GPIO as GPIO  # Use Jetson.GPIO for NVIDIA Orin Nano
import threading

# Constants
DEFAULT_WALL_WIDTH = 3.0  # meters
DEFAULT_WALL_HEIGHT = 3.0  # meters
SUB_CARRIER_LENGTH = 0.4  # meters
SWIPE_DISTANCE = 1.4  # meters (horizontal movement per swipe)
STEP_PERCENTAGE = 0.25  # Step down 25% of the sub-carrier length
TARGET_DISTANCE = 0.1  # meters (safe distance from wall)

# Precomputed values
STEP_DOWN = SUB_CARRIER_LENGTH * STEP_PERCENTAGE

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define GPIO pins for SPST switches
LEFT_SWITCH_PIN = 17  # GPIO pin for left switch
RIGHT_SWITCH_PIN = 27  # GPIO pin for right switch

# Set up GPIO pins as inputs with pull-up resistors
GPIO.setup(LEFT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RIGHT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# SPI Setup
spi = spidev.SpiDev()
spi.open(0, 0)  # (bus 0, device 0)
spi.max_speed_hz = 1000000  # 1 MHz SPI speed

# Thread-safe lock for SPI communication
spi_lock = threading.Lock()

def read_sensor_data():
    """Read ultrasonic sensor data from Arduino via SPI"""
    with spi_lock:  # Ensure thread-safe SPI communication
        response = spi.xfer2([0x01])  # Request sensor data (0x01 = command)
    distance = int.from_bytes(response[1:3], byteorder='big') / 100.0  # cm to meters
    return distance

def send_motor_command(direction, speed, duration=None):
    """Send motor control command to Arduino via SPI and optionally wait for completion."""
    cmd = [0x02, direction, int(speed * 100)]  # 0x02 = motor command
    with spi_lock:  # Ensure thread-safe SPI communication
        spi.xfer2(cmd)
    if duration:
        time.sleep(duration)
        stop_motors()

def stop_motors():
    """Stop all motors."""
    with spi_lock:  # Ensure thread-safe SPI communication
        spi.xfer2([0x02, 0, 0])  # Send stop command

def self_diagnostic():
    """Perform self-diagnostic check on sensors and motors"""
    print("Performing self-diagnostic...")
    try:
        # Check sensor readings
        sensor_data = read_sensor_data()
        if sensor_data is None:
            raise Exception("Ultrasonic sensor not responding!")

        # Check motor commands
        send_motor_command(1, 0.2, 0.5)  # Forward at 20% speed for 0.5 seconds
        print("Self-diagnostic passed!")
    except Exception as e:
        print(f"Self-diagnostic failed: {e}")
        raise e

def approach_wall(target_distance=TARGET_DISTANCE):
    """Use thrusters to approach the wall at a safe distance"""
    while True:
        current_dist = read_sensor_data()
        if current_dist <= target_distance:
            break
        # Move forward slowly
        send_motor_command(1, 0.2, 0.05)  # Reduced sleep for quicker response
    stop_motors()

def clean_wall(wall_width, wall_height):
    """Main cleaning algorithm: Swipe left-right, step down"""
    max_swipes = int(wall_height / STEP_DOWN)
    swipe_time = wall_width / 0.5  # Time = distance / speed
    step_time = STEP_DOWN / 0.3

    for swipe in range(max_swipes):
        # Combine horizontal movement (right) and step-down
        send_motor_command(3, 0.5, swipe_time)  # Move right
        send_motor_command(2, 0.3, step_time)  # Step down

        # Combine horizontal movement (left) and step-down
        send_motor_command(4, 0.5, swipe_time)  # Move left
        send_motor_command(2, 0.3, step_time)  # Step down

def clean_floor(floor_length, floor_width):
    """Clean the floor after walls"""
    print("Starting floor cleaning...")
    swipes = int(floor_width / SWIPE_DISTANCE)
    swipe_time = SWIPE_DISTANCE / 0.5
    floor_time = floor_length / 0.5

    for _ in range(swipes):
        # Combine forward and backward motion with right movement
        send_motor_command(1, 0.5, floor_time)  # Move forward
        send_motor_command(3, 0.5, swipe_time)  # Move right
        send_motor_command(5, 0.5, floor_time)  # Move backward
        send_motor_command(3, 0.5, swipe_time)  # Move right

def main():
    try:
        # Input tank dimensions
        wall_width = float(input("Enter wall width (meters): ") or DEFAULT_WALL_WIDTH)
        wall_height = float(input("Enter wall height (meters): ") or DEFAULT_WALL_HEIGHT)
        floor_length = wall_width  # Assuming floor length equals wall width
        floor_width = wall_height  # Assuming floor width equals wall height

        print("Starting Cleaning Robot...")

        # Perform self-diagnostic before starting
        self_diagnostic()

        for i in range(4):  # Clean 4 walls
            print(f"Cleaning wall {i + 1}...")
            approach_wall()  # Get close to the wall
            clean_wall(wall_width, wall_height)  # Perform cleaning pattern

            # Monitor SPST switches
            if GPIO.input(LEFT_SWITCH_PIN) == GPIO.LOW:
                print("Left wall touched")
            if GPIO.input(RIGHT_SWITCH_PIN) == GPIO.LOW:
                print("Right wall touched")

        # Clean the floor
        clean_floor(floor_length, floor_width)
        print("Cleaning Complete!")
    except KeyboardInterrupt:
        print("Stopping Robot...")
    finally:
        spi.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()