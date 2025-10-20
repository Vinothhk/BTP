# #!/usr/bin/env python
# import sys
# import os
# import pygame
# import time

# sys.path.append("..")
# from scservo_sdk import *

# # Servo Configuration
# GRIPPER_ID = 6
# BAUDRATE = 115200 #1000000
# DEVICENAME = '/dev/ttyUSB0'
# SCS_MINIMUM_POSITION = 200
# SCS_MAXIMUM_POSITION = 875
# SCS_MOVING_SPEED = 1500
# SCS_MOVING_ACC = 50

# # Joystick axes
# AXIS_2 = 2  # First gripper control
# AXIS_5 = 5  # Second gripper control

# def clip_axis_to_0_1(value):
#     """Clip joystick axis from range [-1, 1] to [0, 1]"""
#     return max(0.0, min(1.0, (value + 1.0) / 2.0))

# def map_to_servo_position(normalized_value):
#     """Map 0-1 range to servo position range"""
#     return int(SCS_MINIMUM_POSITION + normalized_value * (SCS_MAXIMUM_POSITION - SCS_MINIMUM_POSITION))

# # Initialize Pygame and Joystick
# pygame.init()
# pygame.joystick.init()
# clock = pygame.time.Clock()

# # Check for joystick
# joystick_count = pygame.joystick.get_count()
# if joystick_count == 0:
#     print("No joystick found!")
#     quit()

# joystick = pygame.joystick.Joystick(0)
# joystick.init()
# print(f"Joystick Connected: {joystick.get_name()}")
# print(f"Number of Axes: {joystick.get_numaxes()}")

# # Verify axes exist
# if joystick.get_numaxes() < max(AXIS_2, AXIS_5) + 1:
#     print(f"Warning: Joystick doesn't have axis {max(AXIS_2, AXIS_5)}")
#     print("Available axes:", joystick.get_numaxes())

# # Initialize Servo Communication
# portHandler = PortHandler(DEVICENAME)
# packetHandler = sms_sts(portHandler)

# # Open port
# if not portHandler.openPort():
#     print(f"Failed to open port {DEVICENAME}")
#     pygame.quit()
#     quit()
# print("Servo port opened successfully")

# # Set baudrate
# if not portHandler.setBaudRate(BAUDRATE):
#     print("Failed to set baudrate")
#     portHandler.closePort()
#     pygame.quit()
#     quit()

# # Test servo connection
# print(f"Testing connection to gripper servo ID {GRIPPER_ID}...")
# scs_model, scs_comm_result, scs_error = packetHandler.ping(GRIPPER_ID)
# if scs_comm_result != COMM_SUCCESS:
#     print(f"Cannot find servo ID {GRIPPER_ID}!")
#     portHandler.closePort()
#     pygame.quit()
#     quit()
# print(f"Gripper servo connected! Model: {scs_model}\n")

# print("=" * 60)
# print("GRIPPER CONTROL ACTIVE")
# print("=" * 60)
# print(f"Use Axis {AXIS_2} or Axis {AXIS_5} to control gripper")
# print("Axis range -1 to 1 will be clipped to 0 to 1")
# print("0 = Fully Open, 1 = Fully Closed")
# print("Press ESC or close window to exit")
# print("=" * 60)

# running = True
# last_position = -1
# update_interval = 0.05  # Update servo every 50ms

# try:
#     while running:
#         for event in pygame.event.get():
#             if event.type == pygame.QUIT:
#                 running = False
#             elif event.type == pygame.KEYDOWN:
#                 if event.key == pygame.K_ESCAPE:
#                     running = False

#         # Read axis values
#         axis_2_value = joystick.get_axis(AXIS_2) if joystick.get_numaxes() > AXIS_2 else 0
#         axis_5_value = joystick.get_axis(AXIS_5) if joystick.get_numaxes() > AXIS_5 else 0

#         # Clip both axes to 0-1 range
#         axis_2_clipped = clip_axis_to_0_1(axis_2_value)
#         axis_5_clipped = clip_axis_to_0_1(axis_5_value)

#         # Use the maximum value from either axis (so either can control the gripper)
#         gripper_control = max(axis_2_clipped, axis_5_clipped)

#         # Map to servo position
#         target_position = map_to_servo_position(gripper_control)

#         # Only send command if position changed significantly (reduce communication overhead)
#         if abs(target_position - last_position) > 10:
#             # Send position command
#             scs_comm_result, scs_error = packetHandler.WritePosEx(
#                 GRIPPER_ID,
#                 target_position,
#                 SCS_MOVING_SPEED,
#                 SCS_MOVING_ACC
#             )

#             if scs_comm_result == COMM_SUCCESS and scs_error == 0:
#                 last_position = target_position
                
#                 # Display status
#                 print(f"Axis2: {axis_2_value:+.3f} ({axis_2_clipped:.3f}) | "
#                       f"Axis5: {axis_5_value:+.3f} ({axis_5_clipped:.3f}) | "
#                       f"Control: {gripper_control:.3f} | "
#                       f"Position: {target_position:4d}")
#             else:
#                 print(f"Communication error with servo")

#         clock.tick(20)  # Run at 20 Hz
#         time.sleep(update_interval)

# except KeyboardInterrupt:
#     print("\nInterrupted by user")

# finally:
#     print("\nClosing connections...")
#     portHandler.closePort()
#     pygame.quit()
#     print("Shutdown complete")

#!/usr/bin/env python
import sys
import os
import pygame
import time

sys.path.append("..")
from scservo_sdk import *

# Servo Configuration
GRIPPER_ID = 6
BAUDRATE = 115200
DEVICENAME = '/dev/ttyUSB0'
SCS_MINIMUM_POSITION = 180
SCS_MAXIMUM_POSITION = 875
SCS_MOVING_SPEED = 1500
SCS_MOVING_ACC = 50

# Joystick axes
AXIS_5 = 5  # RT trigger - gripper control

def clip_axis_to_0_1(value):
    """Clip joystick axis from range [-1, 1] to [0, 1]"""
    return max(0.0, min(1.0, (value + 1.0) / 2.0))

def map_to_servo_position(normalized_value):
    """Map 0-1 range to servo position range"""
    return int(SCS_MINIMUM_POSITION + normalized_value * (SCS_MAXIMUM_POSITION - SCS_MINIMUM_POSITION))

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
if joystick.get_numaxes() < AXIS_5 + 1:
    print(f"Warning: Joystick doesn't have axis {AXIS_5}")
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

# Test servo connection
print(f"Testing connection to gripper servo ID {GRIPPER_ID}...")
scs_model, scs_comm_result, scs_error = packetHandler.ping(GRIPPER_ID)
if scs_comm_result != COMM_SUCCESS:
    print(f"Cannot find servo ID {GRIPPER_ID}!")
    portHandler.closePort()
    pygame.quit()
    quit()
print(f"Gripper servo connected! Model: {scs_model}\n")

print("=" * 60)
print("GRIPPER CONTROL ACTIVE")
print("=" * 60)
print(f"Use RT Trigger (Axis {AXIS_5}) to control gripper")
print("Release trigger = Gripper CLOSED (default position)")
print("Press trigger = Gripper OPEN")
print("Press ESC or close window to exit")
print("=" * 60)

running = True
last_position = -1
update_interval = 0.05  # Update servo every 50ms

try:
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        # Read axis 5 value (RT trigger)
        axis_5_value = joystick.get_axis(AXIS_5) if joystick.get_numaxes() > AXIS_5 else -1.0

        # Clip axis to 0-1 range
        axis_5_clipped = clip_axis_to_0_1(axis_5_value)

        # Use axis 5 for gripper control
        gripper_control = axis_5_clipped

        # Map to servo position
        target_position = map_to_servo_position(gripper_control)

        # Only send command if position changed significantly (reduce communication overhead)
        if abs(target_position - last_position) > 10:
            # Send position command
            scs_comm_result, scs_error = packetHandler.WritePosEx(
                GRIPPER_ID,
                target_position,
                SCS_MOVING_SPEED,
                SCS_MOVING_ACC
            )

            if scs_comm_result == COMM_SUCCESS and scs_error == 0:
                last_position = target_position
                
                # Display status
                print(f"RT Trigger: {axis_5_value:+.3f} → Clipped: {axis_5_clipped:.3f} → Position: {target_position:4d} "
                      f"({'CLOSED' if gripper_control < 0.1 else 'OPEN' if gripper_control > 0.9 else 'PARTIAL'})")
            else:
                print(f"Communication error with servo")

        clock.tick(20)  # Run at 20 Hz
        time.sleep(update_interval)

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    print("\nClosing connections...")
    portHandler.closePort()
    pygame.quit()
    print("Shutdown complete")