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
SERVO_3_ID = 3  # <-- NEW SERVO ID
SERVO_4_ID = 4  # <-- NEW SERVO ID
BAUDRATE = 115200

DEVICENAME = '/dev/ttyUSB0'

# Gripper servo limits
GRIPPER_MIN_POSITION = 200
GRIPPER_MAX_POSITION = 875
GRIPPER_MOVING_SPEED = 1500
GRIPPER_MOVING_ACC = 50

# Base servo limits (ID 1)
BASE_MIN_POSITION = 200
BASE_MAX_POSITION = 1430
BASE_MOVING_SPEED = 200
BASE_MOVING_ACC = 50

# Servo 2 limits (ID 2)
SERVO_2_MIN_POSITION = 1050
SERVO_2_MAX_POSITION = 3090
SERVO_2_MOVING_SPEED = 200
SERVO_2_MOVING_ACC = 50

# Servo 3 limits (ID 3) <-- NEW CONFIGURATION
SERVO_3_MIN_POSITION = 970
SERVO_3_MAX_POSITION = 2900
SERVO_3_MOVING_SPEED = 200
SERVO_3_MOVING_ACC = 50

# Servo 4 limits (ID 4) <-- NEW CONFIGURATION
SERVO_4_MIN_POSITION = 695
SERVO_4_MAX_POSITION = 2550
SERVO_4_MOVING_SPEED = 200
SERVO_4_MOVING_ACC = 50

# ==============================================================================
# 2. JOYSTICK AND CONTROL CONFIGURATION (Highly Responsive & Correct Mapping)
# ==============================================================================
AXIS_0 = 0  # Left Stick Y (Vertical) - Controls Servo 2
AXIS_1 = 1  # Left Stick X (Horizontal) - Controls Base Servo 1
AXIS_3 = 3  # <-- NEW AXIS - Controls Servo 4
AXIS_4 = 4  # <-- NEW AXIS - Controls Servo 3
AXIS_5 = 5  # RT trigger - Gripper control

# Deadzones (Reduced from 0.1 for more sensitivity)
DEADZONE = 0.05

# Incremental movement speeds (Used for all four axis-controlled servos)
INCREMENTAL_SPEED = 50

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
required_axes = max(AXIS_0, AXIS_1, AXIS_3, AXIS_4, AXIS_5) + 1
if joystick.get_numaxes() < required_axes:
    print(f"Warning: Joystick may not have all required axes (Needed: {required_axes}, Available: {joystick.get_numaxes()})")

# Servo Communication Setup
portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)

if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print(f"Failed to open port {DEVICENAME} or set baudrate {BAUDRATE}")
    pygame.quit()
    quit()
print("Servo port opened successfully.")

# Ping Test (Now includes Servo 3 and Servo 4)
for servo_id in [GRIPPER_ID, BASE_SERVO_ID, SERVO_2_ID, SERVO_3_ID, SERVO_4_ID]:
    scs_model, scs_comm_result, scs_error = packetHandler.ping(servo_id)
    if scs_comm_result != COMM_SUCCESS:
        print(f"FATAL: Cannot find servo ID {servo_id}!")
        portHandler.closePort()
        pygame.quit()
        quit()
    print(f"Servo ID {servo_id} connected! Model: {scs_model}")

# Initial positions (Start new servos near the center of their range)
base_current_position = 2048
servo_2_current_position = (SERVO_2_MIN_POSITION + SERVO_2_MAX_POSITION) // 2
servo_3_current_position = (SERVO_3_MIN_POSITION + SERVO_3_MAX_POSITION) // 2 # New Servo 3 initial position
servo_4_current_position = (SERVO_4_MIN_POSITION + SERVO_4_MAX_POSITION) // 2 # New Servo 4 initial position
last_gripper_position = -1

print("=" * 60)
print("ROBOT ARM CONTROL ACTIVE")
print("=" * 60)
print(f"Axis {AXIS_0} (Left Stick Y) - Servo 2 (ID {SERVO_2_ID})")
print(f"Axis {AXIS_1} (Left Stick X) - Base Servo (ID {BASE_SERVO_ID})")
print(f"Axis {AXIS_4} (New Axis) - Servo 3 (ID {SERVO_3_ID})")
print(f"Axis {AXIS_3} (New Axis) - Servo 4 (ID {SERVO_4_ID})")
print(f"Axis {AXIS_5} (RT Trigger) - Gripper (ID {GRIPPER_ID})")
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
        axis_0_value = joystick.get_axis(AXIS_0) if joystick.get_numaxes() > AXIS_0 else 0.0
        axis_1_value = joystick.get_axis(AXIS_1) if joystick.get_numaxes() > AXIS_1 else 0.0
        axis_3_value = joystick.get_axis(AXIS_3) if joystick.get_numaxes() > AXIS_3 else 0.0 # NEW AXIS READ
        axis_4_value = joystick.get_axis(AXIS_4) if joystick.get_numaxes() > AXIS_4 else 0.0 # NEW AXIS READ
        axis_5_value = joystick.get_axis(AXIS_5) if joystick.get_numaxes() > AXIS_5 else -1.0
        
        
        # --- SERVO 2 CONTROL (ID 2, controlled by AXIS 0) ---
        if abs(axis_0_value) < DEADZONE:
            axis_0_value = 0.0 
        position_delta_2 = int(axis_0_value * INCREMENTAL_SPEED)
        
        if position_delta_2 != 0:
            servo_2_current_position += position_delta_2
            servo_2_current_position = max(SERVO_2_MIN_POSITION, min(SERVO_2_MAX_POSITION, servo_2_current_position))

            scs_comm_result, scs_error = packetHandler.WritePosEx(
                SERVO_2_ID, servo_2_current_position, SERVO_2_MOVING_SPEED, SERVO_2_MOVING_ACC
            )
            if scs_comm_result == COMM_SUCCESS and scs_error == 0:
                direction = "DOWN" if position_delta_2 > 0 else "UP"
                print(f"[Servo2] Axis 0 (Y): {axis_0_value:+.3f} | Position: {servo_2_current_position:4d} | Direction: {direction}")
            else:
                print(f"[Servo2] Communication error with servo ID {SERVO_2_ID}")


        # --- BASE SERVO CONTROL (ID 1, controlled by AXIS 1) ---
        if abs(axis_1_value) < DEADZONE:
            axis_1_value = 0.0 
        position_delta = int(axis_1_value * INCREMENTAL_SPEED)
        
        if position_delta != 0:
            base_current_position += position_delta
            base_current_position = max(BASE_MIN_POSITION, min(BASE_MAX_POSITION, base_current_position))

            scs_comm_result, scs_error = packetHandler.WritePosEx(
                BASE_SERVO_ID, base_current_position, BASE_MOVING_SPEED, BASE_MOVING_ACC
            )
            if scs_comm_result == COMM_SUCCESS and scs_error == 0:
                direction = "RIGHT" if position_delta > 0 else "LEFT"
                print(f"[Base] Axis 1 (X): {axis_1_value:+.3f} | Position: {base_current_position:4d} | Direction: {direction}")
            else:
                print(f"[Base] Communication error with servo ID {BASE_SERVO_ID}")
                
        
        # --- SERVO 3 CONTROL (ID 3, controlled by AXIS 4) <-- NEW LOGIC ---
        if abs(axis_4_value) < DEADZONE:
            axis_4_value = 0.0 
        position_delta_3 = int(axis_4_value * INCREMENTAL_SPEED)
        
        if position_delta_3 != 0:
            servo_3_current_position += position_delta_3
            servo_3_current_position = max(SERVO_3_MIN_POSITION, min(SERVO_3_MAX_POSITION, servo_3_current_position))

            scs_comm_result, scs_error = packetHandler.WritePosEx(
                SERVO_3_ID, servo_3_current_position, SERVO_3_MOVING_SPEED, SERVO_3_MOVING_ACC
            )
            if scs_comm_result == COMM_SUCCESS and scs_error == 0:
                direction = "POS" if position_delta_3 > 0 else "NEG"
                print(f"[Servo3] Axis 4: {axis_4_value:+.3f} | Position: {servo_3_current_position:4d} | Direction: {direction}")
            else:
                print(f"[Servo3] Communication error with servo ID {SERVO_3_ID}")


        # --- SERVO 4 CONTROL (ID 4, controlled by AXIS 3) <-- NEW LOGIC ---
        if abs(axis_3_value) < DEADZONE:
            axis_3_value = 0.0 
        position_delta_4 = int(axis_3_value * INCREMENTAL_SPEED)
        
        if position_delta_4 != 0:
            servo_4_current_position += position_delta_4
            servo_4_current_position = max(SERVO_4_MIN_POSITION, min(SERVO_4_MAX_POSITION, servo_4_current_position))

            scs_comm_result, scs_error = packetHandler.WritePosEx(
                SERVO_4_ID, servo_4_current_position, SERVO_4_MOVING_SPEED, SERVO_4_MOVING_ACC
            )
            if scs_comm_result == COMM_SUCCESS and scs_error == 0:
                direction = "POS" if position_delta_4 > 0 else "NEG"
                print(f"[Servo4] Axis 3: {axis_3_value:+.3f} | Position: {servo_4_current_position:4d} | Direction: {direction}")
            else:
                print(f"[Servo4] Communication error with servo ID {SERVO_4_ID}")


        # --- GRIPPER CONTROL (ID 6, Axis 5) ---
        gripper_control = clip_axis_to_0_1(axis_5_value)
        target_gripper_position = map_to_servo_position(gripper_control, GRIPPER_MIN_POSITION, GRIPPER_MAX_POSITION)

        # Send command if position changed significantly
        if abs(target_gripper_position - last_gripper_position) >= GRIPPER_POS_THRESHOLD:
            scs_comm_result, scs_error = packetHandler.WritePosEx(
                GRIPPER_ID, target_gripper_position, GRIPPER_MOVING_SPEED, GRIPPER_MOVING_ACC
            )

            if scs_comm_result == COMM_SUCCESS and scs_error == 0:
                last_gripper_position = target_gripper_position
                status = 'CLOSED' if gripper_control < 0.1 else 'OPEN' if gripper_control > 0.9 else 'PARTIAL'
                print(f"[Gripper] RT: {axis_5_value:+.3f} → Position: {target_gripper_position:4d} ({status})")
            else:
                print(f"[Gripper] Communication error with servo ID {GRIPPER_ID}")

        # --- Loop Timing ---
        clock.tick(LOOP_FREQUENCY)

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    print("\nClosing connections...")
    portHandler.closePort()
    pygame.quit()
    print("Shutdown complete")