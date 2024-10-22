# commands.py
"""
This module contains command constants for the RoboClaw motor controller.
"""

class Cmd:
    """
    Cmd is a collection of command constants used with the RoboClaw motor controller.
    Each constant represents a command that can be sent to the RoboClaw to perform specific actions,
    such as moving a motor, reading sensor data, or setting configurations.
    """

    M1_FORWARD = 0       # Drive Motor 1 forward
    M1_BACKWARD = 1      # Drive Motor 1 backward
    SET_MINIMUM_MAIN_VOLTAGE = 2  # Set minimum main battery voltage
    SET_MAXIMUM_MAIN_VOLTAGE = 3  # Set maximum main battery voltage
    M2_FORWARD = 4       # Drive Motor 2 forward
    M2_BACKWARD = 5      # Drive Motor 2 backward
    M1_DRIVE = 6         # Drive Motor 1 with speed and direction
    M2_DRIVE = 7         # Drive Motor 2 with speed and direction
    MIXED_DRIVE = 8      # Mixed drive mode for M1 and M2
    MIXED_RIGHT = 9      # Mixed right mode
    MIXED_LEFT = 10      # Mixed left mode
    MIXED_FORWARD = 11   # Mixed forward mode
    MIXED_BACKWARD = 12  # Mixed backward mode
    MIXED_RIGHT_FORWARD = 13  # Mixed right forward mode
    MIXED_LEFT_FORWARD = 14   # Mixed left forward mode
    MIXED_RIGHT_BACKWARD = 15 # Mixed right backward mode
    MIXED_LEFT_BACKWARD = 16  # Mixed left backward mode
    READ_M1_ENCODER = 16  # Read Motor 1 encoder value
    READ_M2_ENCODER = 17  # Read Motor 2 encoder value
    SET_M1_ENCODER = 18   # Set Motor 1 encoder value
    SET_M2_ENCODER = 19   # Set Motor 2 encoder value
    READ_M1_SPEED = 20    # Read Motor 1 speed
    READ_M2_SPEED = 21    # Read Motor 2 speed
    READ_FIRMWARE_VERSION = 21  # Read firmware version
    READ_MAIN_BATTERY_VOLTAGE = 24  # Read main battery voltage
    READ_LOGIC_BATTERY_VOLTAGE = 25  # Read logic battery voltage
    SET_MINIMUM_LOGIC_VOLTAGE = 26  # Set minimum logic battery voltage
    SET_MAXIMUM_LOGIC_VOLTAGE = 27  # Set maximum logic battery voltage
    SET_M1_PID = 28       # Set Motor 1 PID constants
    SET_M2_PID = 29       # Set Motor 2 PID constants
    DRIVE_M1_WITH_SIGNED_DUTY = 32  # Drive Motor 1 with signed duty cycle
    DRIVE_M2_WITH_SIGNED_DUTY = 33  # Drive Motor 2 with signed duty cycle
    DRIVE_M1_WITH_SIGNED_SPEED = 35  # Drive Motor 1 with signed speed
    DRIVE_M2_WITH_SIGNED_SPEED = 36  # Drive Motor 2 with signed speed
    DRIVE_M1_WITH_SIGNED_SPEED_ACCEL = 38  # Drive Motor 1 with signed speed and acceleration
    DRIVE_M2_WITH_SIGNED_SPEED_ACCEL = 39  # Drive Motor 2 with signed speed and acceleration
    READ_M1_MAX_MIN_SPEED = 42  # Read Motor 1 max and min speeds
    READ_M2_MAX_MIN_SPEED = 43  # Read Motor 2 max and min speeds
    READ_M1_DUTY_CYCLE = 48   # Read Motor 1 duty cycle
    READ_M2_DUTY_CYCLE = 49   # Read Motor 2 duty cycle
    READ_MOTOR_CURRENTS = 49  # Read motor currents for M1 and M2
    READ_M1_PID = 55  # Read Motor 1 PID constants
    READ_M2_PID = 56  # Read Motor 2 PID constants
    READ_TEMPERATURE = 82  # Read board temperature
    READ_STATUS = 90       # Read the status byte
    READ_ERROR_STATE = 93  # Read error state
    EMERGENCY_STOP = 224  # Emergency stop (in decimal)
    ENTER_SAFE_MODE = 240  # Enter safe mode (in decimal)
    EXIT_SAFE_MODE = 241   # Exit safe mode (in decimal)
