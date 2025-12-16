# main.py - Goal celebration motor control for Robo ESP32
# Motor connected to pins 12 and 13

from machine import Pin, PWM, UART
import sys
import time

# Motor pins
MOTOR_PIN_1 = 12
MOTOR_PIN_2 = 13

# Setup motor pins
motor1 = PWM(Pin(MOTOR_PIN_1), freq=1000)
motor2 = PWM(Pin(MOTOR_PIN_2), freq=1000)

def stop_motor():
    """Stop the motor"""
    motor1.duty(0)
    motor2.duty(0)

def spin_motor_forward(speed=1023):
    """Spin motor forward at given speed (0-1023)"""
    motor1.duty(speed)
    motor2.duty(0)

def spin_motor_backward(speed=1023):
    """Spin motor backward at given speed (0-1023)"""
    motor1.duty(0)
    motor2.duty(speed)

def celebration_dance():
    """Fun celebration routine - spin motor back and forth"""
    print("CELEBRATION DANCE!")
    
    # Spin forward
    spin_motor_forward(800)
    time.sleep(0.3)
    
    # Quick reverse
    spin_motor_backward(800)
    time.sleep(0.3)
    
    # Forward again
    spin_motor_forward(1023)
    time.sleep(0.5)
    
    # Stop
    stop_motor()
    print("Celebration complete!")

# Make sure motor is stopped at start
stop_motor()
print("Goal detection system ready!")
print("Waiting for GOAL signal from computer...")

# Main loop - listen for goal signals using UART
uart = UART(0, baudrate=115200)

while True:
    if uart.any():
        try:
            data = uart.read().decode('utf-8').strip()
            if "GOAL" in data:
                print("GOAL DETECTED!")
                celebration_dance()
        except:
            pass  # Ignore decoding errors
    
    time.sleep(0.01)
