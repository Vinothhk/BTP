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
SERVO_3_ID = 3
SERVO_4_ID = 4
BAUDRATE = 115200

DEVICENAME = '/dev/ttyUSB0'

# Gripper servo limits (Absolute position control)
GRIPPER_MIN_POSITION = 200
GRIPPER_MAX_POSITION = 875
GRIPPER_MOVING_SPEED = 1500
GRIPPER_MOVING_ACC = 50

# Base servo limits (ID 1)
BASE_MIN_POSITION = 200
BASE_MAX_POSITION = 1430
BASE_MAX_SPEED = 100  # Speed for incremental movements

# Servo 2 limits (ID 2)
SERVO_2_MIN_POSITION = 1050
SERVO_2_MAX_POSITION = 3090
SERVO_2_MAX_SPEED = 100

# Servo 3 limits (ID 3)
SERVO_3_MIN_POSITION = 970
SERVO_3_MAX_POSITION = 2900
SERVO_3_MAX_SPEED = 100

# Servo 4 limits (ID 4)
SERVO_4_MIN_POSITION = 695
SERVO_4_MAX_POSITION = 2550
SERVO_4_MAX_SPEED = 100

# Acceleration for all servos
SERVO_ACC = 50

# ==============================================================================
# 2. JOYSTICK AND CONTROL CONFIGURATION
# ==============================================================================
AXIS_0 = 0  # Left Stick Y (Vertical) - Controls Servo 2
AXIS_1 = 1  # Left Stick X (Horizontal) - Controls Base Servo 1
AXIS_3 = 3  # Controls Servo 4
AXIS_4 = 4  # Controls Servo 3
AXIS_5 = 5  # RT trigger - Gripper control

# Deadzones 
DEADZONE = 0.08

# Incremental step size (smaller = more precise control)
POSITION_STEP = 20  # Position units to move per loop when joystick active

# Gripper position change threshold
GRIPPER_POS_THRESHOLD = 5

# Feedback reading interval (read every N loops)
FEEDBACK_READ_INTERVAL = 2

# ==============================================================================
# 3. UTILITY FUNCTIONS
# ==============================================================================
def clip_axis_to_0_1(value):
    """Clip joystick axis from range [-1, 1] to [0, 1]"""
    return max(0.0, min(1.0, (value + 1.0) / 2.0))

def map_to_servo_position(normalized_value, min_pos, max_pos):
    """Map 0-1 range to servo position range"""
    return int(min_pos + normalized_value * (max_pos - min_pos))

def apply_deadzone(value, deadzone):
    """Apply deadzone to axis value"""
    if abs(value) < deadzone:
        return 0.0
    return value

def read_servo_position(packet_handler, servo_id):
    """Read current position from servo"""
    scs_present_position, scs_comm_result, scs_error = packet_handler.ReadPos(servo_id)
    if scs_comm_result == COMM_SUCCESS:
        return scs_present_position
    return None

def send_immediate_position(packet_handler, servo_id, position, speed, acc):
    """
    Send position command that will be executed immediately.
    This creates a 'virtual target' very close to current position for instant stop.
    """
    scs_comm_result, scs_error = packetHandler.WritePosEx(servo_id, position, speed, acc)
    return scs_comm_result == COMM_SUCCESS

# ==============================================================================
# 4. INITIALIZATION
# ==============================================================================
pygame.init()
pygame.joystick.init()
clock = pygame.time.Clock()
LOOP_FREQUENCY = 50  # 50Hz for responsive control

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

# Ping Test
for servo_id in [GRIPPER_ID, BASE_SERVO_ID, SERVO_2_ID, SERVO_3_ID, SERVO_4_ID]:
    scs_model, scs_comm_result, scs_error = packetHandler.ping(servo_id)
    if scs_comm_result != COMM_SUCCESS:
        print(f"FATAL: Cannot find servo ID {servo_id}!")
        portHandler.closePort()
        pygame.quit()
        quit()
    print(f"Servo ID {servo_id} connected! Model: {scs_model}")

# Read initial positions from servos
print("\nReading initial positions...")
base_current_position = read_servo_position(packetHandler, BASE_SERVO_ID)
servo_2_current_position = read_servo_position(packetHandler, SERVO_2_ID)
servo_3_current_position = read_servo_position(packetHandler, SERVO_3_ID)
servo_4_current_position = read_servo_position(packetHandler, SERVO_4_ID)
gripper_current_position = read_servo_position(packetHandler, GRIPPER_ID)

# Fallback to center positions if read fails
if base_current_position is None:
    base_current_position = (BASE_MIN_POSITION + BASE_MAX_POSITION) // 2
if servo_2_current_position is None:
    servo_2_current_position = (SERVO_2_MIN_POSITION + SERVO_2_MAX_POSITION) // 2
if servo_3_current_position is None:
    servo_3_current_position = (SERVO_3_MIN_POSITION + SERVO_3_MAX_POSITION) // 2
if servo_4_current_position is None:
    servo_4_current_position = (SERVO_4_MIN_POSITION + SERVO_4_MAX_POSITION) // 2
if gripper_current_position is None:
    gripper_current_position = GRIPPER_MIN_POSITION

print(f"Base (ID {BASE_SERVO_ID}): {base_current_position}")
print(f"Servo 2 (ID {SERVO_2_ID}): {servo_2_current_position}")
print(f"Servo 3 (ID {SERVO_3_ID}): {servo_3_current_position}")
print(f"Servo 4 (ID {SERVO_4_ID}): {servo_4_current_position}")
print(f"Gripper (ID {GRIPPER_ID}): {gripper_current_position}")

last_gripper_position = gripper_current_position
feedback_counter = 0

# Track previous joystick states to detect when stick returns to center
prev_axis_0 = 0.0
prev_axis_1 = 0.0
prev_axis_3 = 0.0
prev_axis_4 = 0.0

print("\n" + "=" * 60)
print("🤖 ROBOT ARM CONTROL ACTIVE - IMMEDIATE STOP MODE")
print("=" * 60)
print(f"Axis {AXIS_0} (Left Stick Y) - Servo 2 (ID {SERVO_2_ID})")
print(f"Axis {AXIS_1} (Left Stick X) - Base Servo (ID {BASE_SERVO_ID})")
print(f"Axis {AXIS_4} - Servo 3 (ID {SERVO_3_ID})")
print(f"Axis {AXIS_3} - Servo 4 (ID {SERVO_4_ID})")
print(f"Axis {AXIS_5} (RT Trigger) - Gripper (ID {GRIPPER_ID})")
print("✅ SERVOS STOP IMMEDIATELY WHEN JOYSTICK RELEASED")
print("✅ CONTINUOUS POSITION FEEDBACK ENABLED")
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
        axis_3_value = joystick.get_axis(AXIS_3) if joystick.get_numaxes() > AXIS_3 else 0.0
        axis_4_value = joystick.get_axis(AXIS_4) if joystick.get_numaxes() > AXIS_4 else 0.0
        axis_5_value = joystick.get_axis(AXIS_5) if joystick.get_numaxes() > AXIS_5 else -1.0
        
        # === Read Feedback from All Servos (Periodically) ===
        feedback_counter += 1
        if feedback_counter >= FEEDBACK_READ_INTERVAL:
            feedback_counter = 0
            
            # Read all servo positions
            base_fb = read_servo_position(packetHandler, BASE_SERVO_ID)
            servo_2_fb = read_servo_position(packetHandler, SERVO_2_ID)
            servo_3_fb = read_servo_position(packetHandler, SERVO_3_ID)
            servo_4_fb = read_servo_position(packetHandler, SERVO_4_ID)
            gripper_fb = read_servo_position(packetHandler, GRIPPER_ID)
            
            # Update stored positions if read successful
            if base_fb is not None:
                base_current_position = base_fb
            if servo_2_fb is not None:
                servo_2_current_position = servo_2_fb
            if servo_3_fb is not None:
                servo_3_current_position = servo_3_fb
            if servo_4_fb is not None:
                servo_4_current_position = servo_4_fb
            if gripper_fb is not None:
                gripper_current_position = gripper_fb
        
        
        # === SERVO 2 CONTROL (ID 2, controlled by AXIS 0) ===
        axis_0_filtered = apply_deadzone(axis_0_value, DEADZONE)
        
        if abs(axis_0_filtered) > 0.001:
            # Joystick is active - calculate small incremental target
            direction = 1 if axis_0_filtered > 0 else -1
            step_size = int(abs(axis_0_filtered) * POSITION_STEP)
            target_position = servo_2_current_position + (direction * step_size)
            
            # Clamp to limits
            target_position = max(SERVO_2_MIN_POSITION, min(SERVO_2_MAX_POSITION, target_position))
            
            # Send command with high speed for immediate response
            send_immediate_position(packetHandler, SERVO_2_ID, target_position, SERVO_2_MAX_SPEED, SERVO_ACC)
            
            if abs(axis_0_filtered - prev_axis_0) > 0.05:  # Only print on significant changes
                dir_name = "DOWN" if direction > 0 else "UP"
                print(f"[Servo2] Target: {target_position:4d} | Current: {servo_2_current_position:4d} | {dir_name}")
        
        elif abs(prev_axis_0) > 0.001:  # Joystick just released
            # STOP: Send current position as target
            send_immediate_position(packetHandler, SERVO_2_ID, servo_2_current_position, SERVO_2_MAX_SPEED, SERVO_ACC)
            print(f"[Servo2] 🛑 STOP at position {servo_2_current_position}")
        
        prev_axis_0 = axis_0_filtered


        # === BASE SERVO CONTROL (ID 1, controlled by AXIS 1) ===
        axis_1_filtered = apply_deadzone(axis_1_value, DEADZONE)
        
        if abs(axis_1_filtered) > 0.001:
            direction = 1 if axis_1_filtered > 0 else -1
            step_size = int(abs(axis_1_filtered) * POSITION_STEP)
            target_position = base_current_position + (direction * step_size)
            
            target_position = max(BASE_MIN_POSITION, min(BASE_MAX_POSITION, target_position))
            
            send_immediate_position(packetHandler, BASE_SERVO_ID, target_position, BASE_MAX_SPEED, SERVO_ACC)
            
            if abs(axis_1_filtered - prev_axis_1) > 0.05:
                dir_name = "RIGHT" if direction > 0 else "LEFT"
                print(f"[Base]   Target: {target_position:4d} | Current: {base_current_position:4d} | {dir_name}")
        
        elif abs(prev_axis_1) > 0.001:
            send_immediate_position(packetHandler, BASE_SERVO_ID, base_current_position, BASE_MAX_SPEED, SERVO_ACC)
            print(f"[Base]   🛑 STOP at position {base_current_position}")
        
        prev_axis_1 = axis_1_filtered
                
        
        # === SERVO 3 CONTROL (ID 3, controlled by AXIS 4) ===
        axis_4_filtered = apply_deadzone(axis_4_value, DEADZONE)
        
        if abs(axis_4_filtered) > 0.001:
            direction = 1 if axis_4_filtered > 0 else -1
            step_size = int(abs(axis_4_filtered) * POSITION_STEP)
            target_position = servo_3_current_position + (direction * step_size)
            
            target_position = max(SERVO_3_MIN_POSITION, min(SERVO_3_MAX_POSITION, target_position))
            
            send_immediate_position(packetHandler, SERVO_3_ID, target_position, SERVO_3_MAX_SPEED, SERVO_ACC)
            
            if abs(axis_4_filtered - prev_axis_4) > 0.05:
                dir_name = "POS" if direction > 0 else "NEG"
                print(f"[Servo3] Target: {target_position:4d} | Current: {servo_3_current_position:4d} | {dir_name}")
        
        elif abs(prev_axis_4) > 0.001:
            send_immediate_position(packetHandler, SERVO_3_ID, servo_3_current_position, SERVO_3_MAX_SPEED, SERVO_ACC)
            print(f"[Servo3] 🛑 STOP at position {servo_3_current_position}")
        
        prev_axis_4 = axis_4_filtered


        # === SERVO 4 CONTROL (ID 4, controlled by AXIS 3) ===
        axis_3_filtered = apply_deadzone(axis_3_value, DEADZONE)
        
        if abs(axis_3_filtered) > 0.001:
            direction = 1 if axis_3_filtered > 0 else -1
            step_size = int(abs(axis_3_filtered) * POSITION_STEP)
            target_position = servo_4_current_position + (direction * step_size)
            
            target_position = max(SERVO_4_MIN_POSITION, min(SERVO_4_MAX_POSITION, target_position))
            
            send_immediate_position(packetHandler, SERVO_4_ID, target_position, SERVO_4_MAX_SPEED, SERVO_ACC)
            
            if abs(axis_3_filtered - prev_axis_3) > 0.05:
                dir_name = "POS" if direction > 0 else "NEG"
                print(f"[Servo4] Target: {target_position:4d} | Current: {servo_4_current_position:4d} | {dir_name}")
        
        elif abs(prev_axis_3) > 0.001:
            send_immediate_position(packetHandler, SERVO_4_ID, servo_4_current_position, SERVO_4_MAX_SPEED, SERVO_ACC)
            print(f"[Servo4] 🛑 STOP at position {servo_4_current_position}")
        
        prev_axis_3 = axis_3_filtered


        # --- GRIPPER CONTROL (ID 6, Axis 5) - ABSOLUTE POSITION MODE ---
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

        # --- Loop Timing ---
        clock.tick(LOOP_FREQUENCY)

except KeyboardInterrupt:
    print("\n⚠️ Interrupted by user")

finally:
    # EMERGENCY STOP - Send current position as target for all servos
    print("\n🛑 EMERGENCY STOP - Halting all servos...")
    
    # Read final positions
    base_final = read_servo_position(packetHandler, BASE_SERVO_ID)
    servo_2_final = read_servo_position(packetHandler, SERVO_2_ID)
    servo_3_final = read_servo_position(packetHandler, SERVO_3_ID)
    servo_4_final = read_servo_position(packetHandler, SERVO_4_ID)
    
    # Send stop commands
    if base_final is not None:
        packetHandler.WritePosEx(BASE_SERVO_ID, base_final, 0, SERVO_ACC)
    if servo_2_final is not None:
        packetHandler.WritePosEx(SERVO_2_ID, servo_2_final, 0, SERVO_ACC)
    if servo_3_final is not None:
        packetHandler.WritePosEx(SERVO_3_ID, servo_3_final, 0, SERVO_ACC)
    if servo_4_final is not None:
        packetHandler.WritePosEx(SERVO_4_ID, servo_4_final, 0, SERVO_ACC)
    
    time.sleep(0.1)  # Give servos time to process stop command
    
    print("Closing connections...")
    portHandler.closePort()
    pygame.quit()
    print("✅ Shutdown complete")