#!/usr/bin/env python
import sys
import os
import pygame
import time

sys.path.append("..")
from scservo_sdk import *

# Servo Configuration
GRIPPER_ID = 6
BASE_SERVO_ID = 1
BAUDRATE = 115200
DEVICENAME = '/dev/ttyUSB0'

# Gripper servo limits
GRIPPER_MIN_POSITION = 200
GRIPPER_MAX_POSITION = 875
GRIPPER_MOVING_SPEED = 1500
GRIPPER_MOVING_ACC = 50

# Base servo limits
BASE_MIN_POSITION = 370
BASE_MAX_POSITION = 2610
BASE_MOVING_SPEED = 1000
BASE_MOVING_ACC = 50

# Joystick axes
AXIS_1 = 1  # Left stick Y-axis - base servo control
AXIS_5 = 5  # RT trigger - gripper control

def clip_axis_to_0_1(value):
    """Clip joystick axis from range [-1, 1] to [0, 1]"""
    return max(0.0, min(1.0, (value + 1.0) / 2.0))

def map_to_servo_position(normalized_value, min_pos, max_pos):
    """Map 0-1 range to servo position range"""
    return int(min_pos + normalized_value * (max_pos - min_pos))

# Initialize Pygame and Joystick
pygame.init()
pygame.joystick.init()
clock = pygame.time.Clock()

# Check for joystick
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joystick found!")
    quit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Joystick Connected: {joystick.get_name()}")
print(f"Number of Axes: {joystick.get_numaxes()}")

# Verify axes exist
if joystick.get_numaxes() < max(AXIS_1, AXIS_5) + 1:
    print(f"Warning: Joystick doesn't have required axes")
    print("Available axes:", joystick.get_numaxes())

# Initialize Servo Communication
portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)

# Open port
if not portHandler.openPort():
    print(f"Failed to open port {DEVICENAME}")
    pygame.quit()
    quit()
print("Servo port opened successfully")

# Set baudrate
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to set baudrate")
    portHandler.closePort()
    pygame.quit()
    quit()

# Test servo connections
print(f"Testing connection to servos...")
scs_model, scs_comm_result, scs_error = packetHandler.ping(GRIPPER_ID)
if scs_comm_result != COMM_SUCCESS:
    print(f"Cannot find gripper servo ID {GRIPPER_ID}!")
    portHandler.closePort()
    pygame.quit()
    quit()
print(f"Gripper servo (ID {GRIPPER_ID}) connected! Model: {scs_model}")

scs_model, scs_comm_result, scs_error = packetHandler.ping(BASE_SERVO_ID)
if scs_comm_result != COMM_SUCCESS:
    print(f"Cannot find base servo ID {BASE_SERVO_ID}!")
    portHandler.closePort()
    pygame.quit()
    quit()
print(f"Base servo (ID {BASE_SERVO_ID}) connected! Model: {scs_model}\n")

print("=" * 60)
print("ROBOT ARM CONTROL ACTIVE")
print("=" * 60)
print(f"Axis {AXIS_1} (Left Stick Y) - Base Servo Rotation (ID {BASE_SERVO_ID})")
print("  Push forward (negative) = Rotate one direction")
print("  Pull back (positive) = Rotate opposite direction")
print("  Center = Hold current position")
print(f"\nAxis {AXIS_5} (RT Trigger) - Gripper Control (ID {GRIPPER_ID})")
print("  Release trigger = Gripper CLOSED")
print("  Press trigger = Gripper OPEN")
print("\nPress ESC or close window to exit")
print("=" * 60)

running = True
last_gripper_position = -1
base_current_position = 2048  # Start at center position
last_base_command_time = time.time()
update_interval = 0.05  # Update servo every 50ms
base_deadzone = 0.1  # Deadzone for axis 1 to prevent drift

try:
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        # Read axis values
        axis_1_value = joystick.get_axis(AXIS_1) if joystick.get_numaxes() > AXIS_1 else 0.0
        axis_5_value = joystick.get_axis(AXIS_5) if joystick.get_numaxes() > AXIS_5 else -1.0

        # === BASE SERVO CONTROL (Axis 1) ===
        # Apply deadzone to prevent drift
        if abs(axis_1_value) < base_deadzone:
            axis_1_value = 0.0
        
        # Calculate incremental movement based on axis value
        # Negative axis = rotate one direction, Positive = rotate opposite
        movement_speed = 30  # Position units per update
        position_delta = int(axis_1_value * movement_speed)
        
        if position_delta != 0:
            # Update base position incrementally
            base_current_position += position_delta
            
            # Clamp to servo limits
            base_current_position = max(BASE_MIN_POSITION, min(BASE_MAX_POSITION, base_current_position))
            
            # Send command to base servo
            scs_comm_result, scs_error = packetHandler.WritePosEx(
                BASE_SERVO_ID,
                base_current_position,
                BASE_MOVING_SPEED,
                BASE_MOVING_ACC
            )
            
            if scs_comm_result == COMM_SUCCESS and scs_error == 0:
                direction = "CW" if position_delta > 0 else "CCW"
                print(f"[Base] Axis: {axis_1_value:+.3f} | Position: {base_current_position:4d} | Direction: {direction}")
            else:
                print(f"[Base] Communication error with servo ID {BASE_SERVO_ID}")

        # === GRIPPER CONTROL (Axis 5) ===
        # Clip axis to 0-1 range
        axis_5_clipped = clip_axis_to_0_1(axis_5_value)
        gripper_control = axis_5_clipped

        # Map to servo position
        target_gripper_position = map_to_servo_position(gripper_control, GRIPPER_MIN_POSITION, GRIPPER_MAX_POSITION)

        # Only send command if position changed significantly (reduce communication overhead)
        if abs(target_gripper_position - last_gripper_position) > 10:
            # Send position command
            scs_comm_result, scs_error = packetHandler.WritePosEx(
                GRIPPER_ID,
                target_gripper_position,
                GRIPPER_MOVING_SPEED,
                GRIPPER_MOVING_ACC
            )

            if scs_comm_result == COMM_SUCCESS and scs_error == 0:
                last_gripper_position = target_gripper_position
                
                # Display status
                status = 'CLOSED' if gripper_control < 0.1 else 'OPEN' if gripper_control > 0.9 else 'PARTIAL'
                print(f"[Gripper] RT: {axis_5_value:+.3f} → Position: {target_gripper_position:4d} ({status})")
            else:
                print(f"[Gripper] Communication error with servo ID {GRIPPER_ID}")

        clock.tick(20)  # Run at 20 Hz
        time.sleep(update_interval)

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    print("\nClosing connections...")
    portHandler.closePort()
    pygame.quit()
    print("Shutdown complete")