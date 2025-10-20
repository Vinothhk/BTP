#!/usr/bin/env python
import sys
import os
import pygame
import time

sys.path.append("..")
from scservo_sdk import *

# ==============================================================================
# 1. SERVO AND COMMUNICATION CONFIGURATION
# ==============================================================================
GRIPPER_ID = 6
BASE_SERVO_ID = 1
SERVO_2_ID = 2
BAUDRATE = 115200
DEVICENAME = '/dev/ttyUSB0'

# Gripper servo limits
GRIPPER_MIN_POSITION = 200
GRIPPER_MAX_POSITION = 875
GRIPPER_MOVING_SPEED = 1500
GRIPPER_MOVING_ACC = 50

# Base servo limits (ID 1)
BASE_MIN_POSITION = 370
BASE_MAX_POSITION = 610
BASE_MOVING_SPEED = 1000
BASE_MOVING_ACC = 50

# Servo 2 limits
SERVO_2_MIN_POSITION = 1050
SERVO_2_MAX_POSITION = 3090
SERVO_2_MOVING_SPEED = 1000
SERVO_2_MOVING_ACC = 50

# ==============================================================================
# 2. JOYSTICK AND CONTROL CONFIGURATION (Highly Responsive Settings)
# ==============================================================================
# Pygame is reading your axes as:
# AXIS_0 (0) = Left Stick Y (Vertical) - Should control Servo 2
# AXIS_1 (1) = Left Stick X (Horizontal) - Should control Base Servo 1

AXIS_0 = 0
AXIS_1 = 1
AXIS_5 = 5  # RT trigger - Gripper control

# Deadzones (Reduced from 0.1 for more sensitivity)
BASE_DEADZONE = 0.05
SERVO_2_DEADZONE = 0.05

# Incremental movement speeds (Increased from 30 for faster motion)
INCREMENTAL_SPEED_BASE = 50
INCREMENTAL_SPEED_SERVO_2 = 50

# Gripper position change threshold
GRIPPER_POS_THRESHOLD = 5

# ==============================================================================
# 3. UTILITY FUNCTIONS
# ==============================================================================
def clip_axis_to_0_1(value):
    """Clip joystick axis from range [-1, 1] to [0, 1]"""
    return max(0.0, min(1.0, (value + 1.0) / 2.0))

def map_to_servo_position(normalized_value, min_pos, max_pos):
    """Map 0-1 range to servo position range"""
    return int(min_pos + normalized_value * (max_pos - min_pos))

# ==============================================================================
# 4. INITIALIZATION
# ==============================================================================
pygame.init()
pygame.joystick.init()
clock = pygame.time.Clock()
LOOP_FREQUENCY = 20  # Run at 20 Hz (50ms per loop)

# Joystick Setup
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joystick found! Please connect a controller.")
    quit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Joystick Connected: {joystick.get_name()}")
if joystick.get_numaxes() < max(AXIS_0, AXIS_1, AXIS_5) + 1:
    print(f"Warning: Joystick doesn't have required axes (Needed: {max(AXIS_0, AXIS_1, AXIS_5) + 1}, Available: {joystick.get_numaxes()})")

# Servo Communication Setup
portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)

if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print(f"Failed to open port {DEVICENAME} or set baudrate {BAUDRATE}")
    pygame.quit()
    quit()
print("Servo port opened successfully.")

# Ping Test
for servo_id in [GRIPPER_ID, BASE_SERVO_ID, SERVO_2_ID]:
    scs_model, scs_comm_result, scs_error = packetHandler.ping(servo_id)
    if scs_comm_result != COMM_SUCCESS:
        print(f"FATAL: Cannot find servo ID {servo_id}!")
        portHandler.closePort()
        pygame.quit()
        quit()
    print(f"Servo ID {servo_id} connected! Model: {scs_model}")

# Initial positions
base_current_position = 2048
servo_2_current_position = (SERVO_2_MIN_POSITION + SERVO_2_MAX_POSITION) // 2
last_gripper_position = -1

print("=" * 60)
print("ROBOT ARM CONTROL ACTIVE")
print("=" * 60)
# Updated print statements to reflect the actual axis-to-servo mapping
print(f"Axis {AXIS_0} (Left Stick Y) - Servo 2 (ID {SERVO_2_ID}) - Incremental Control")
print(f"Axis {AXIS_1} (Left Stick X) - Base Servo (ID {BASE_SERVO_ID}) - Incremental Control")
print(f"Axis {AXIS_5} (RT Trigger) - Gripper (ID {GRIPPER_ID}) - Direct Mapped Control")
print("=" * 60)

# ==============================================================================
# 5. MAIN CONTROL LOOP
# ==============================================================================
running = True
try:
    while running:
        # --- Pygame Event Handling ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False

        # --- Read Axis Values ---
        # axis_0_value holds Left Stick Y (Vertical)
        # axis_1_value holds Left Stick X (Horizontal)
        axis_0_value = joystick.get_axis(AXIS_0) if joystick.get_numaxes() > AXIS_0 else 0.0
        axis_1_value = joystick.get_axis(AXIS_1) if joystick.get_numaxes() > AXIS_1 else 0.0
        axis_5_value = joystick.get_axis(AXIS_5) if joystick.get_numaxes() > AXIS_5 else -1.0
        
        # NOTE: The order of the control blocks has been swapped to match the intended logic
        # Axis 0 (Y-axis) controls Servo 2
        # Axis 1 (X-axis) controls Base Servo 1

        # --- SERVO 2 CONTROL (ID 2, controlled by AXIS 0 - Left Stick Y) ---
        if abs(axis_0_value) >= SERVO_2_DEADZONE:
            position_delta_2 = int(axis_0_value * INCREMENTAL_SPEED_SERVO_2)
            servo_2_current_position += position_delta_2
            servo_2_current_position = max(SERVO_2_MIN_POSITION, min(SERVO_2_MAX_POSITION, servo_2_current_position))

            scs_comm_result, scs_error = packetHandler.WritePosEx(
                SERVO_2_ID,
                servo_2_current_position,
                SERVO_2_MOVING_SPEED,
                SERVO_2_MOVING_ACC
            )

            if scs_comm_result == COMM_SUCCESS and scs_error == 0:
                direction = "DOWN" if position_delta_2 > 0 else "UP"
                print(f"[Servo2] Axis 0 (Y): {axis_0_value:+.3f} | Position: {servo_2_current_position:4d} | Direction: {direction}")
            else:
                print(f"[Servo2] Communication error with servo ID {SERVO_2_ID}")


        # --- BASE SERVO CONTROL (ID 1, controlled by AXIS 1 - Left Stick X) ---
        if abs(axis_1_value) >= BASE_DEADZONE:
            position_delta = int(axis_1_value * INCREMENTAL_SPEED_BASE)
            base_current_position += position_delta
            base_current_position = max(BASE_MIN_POSITION, min(BASE_MAX_POSITION, base_current_position))

            scs_comm_result, scs_error = packetHandler.WritePosEx(
                BASE_SERVO_ID,
                base_current_position,
                BASE_MOVING_SPEED,
                BASE_MOVING_ACC
            )

            if scs_comm_result == COMM_SUCCESS and scs_error == 0:
                # Direction is based on the movement (positive delta is CW/Right, negative is CCW/Left)
                direction = "RIGHT" if position_delta > 0 else "LEFT"
                print(f"[Base] Axis 1 (X): {axis_1_value:+.3f} | Position: {base_current_position:4d} | Direction: {direction}")
            else:
                print(f"[Base] Communication error with servo ID {BASE_SERVO_ID}")


        # --- GRIPPER CONTROL (ID 6, Axis 5) ---
        gripper_control = clip_axis_to_0_1(axis_5_value)
        target_gripper_position = map_to_servo_position(gripper_control, GRIPPER_MIN_POSITION, GRIPPER_MAX_POSITION)

        # Send command if position changed significantly
        if abs(target_gripper_position - last_gripper_position) >= GRIPPER_POS_THRESHOLD:
            scs_comm_result, scs_error = packetHandler.WritePosEx(
                GRIPPER_ID,
                target_gripper_position,
                GRIPPER_MOVING_SPEED,
                GRIPPER_MOVING_ACC
            )

            if scs_comm_result == COMM_SUCCESS and scs_error == 0:
                last_gripper_position = target_gripper_position
                status = 'CLOSED' if gripper_control < 0.1 else 'OPEN' if gripper_control > 0.9 else 'PARTIAL'
                print(f"[Gripper] RT: {axis_5_value:+.3f} → Position: {target_gripper_position:4d} ({status})")
            else:
                print(f"[Gripper] Communication error with servo ID {GRIPPER_ID}")

        # --- Loop Timing ---
        clock.tick(LOOP_FREQUENCY) # Maintains a stable 20 Hz loop rate (50ms)

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    print("\nClosing connections...")
    portHandler.closePort()
    pygame.quit()
    print("Shutdown complete")