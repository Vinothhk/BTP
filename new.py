#!/usr/bin/env python3
#
# *********     Gen Write Example (Linux)     *********
#
# Compatible with ST Servos (STS3215 / STS3020 / STS3025, etc.)
#

import sys
import os
import tty
import termios

# Linux getch implementation
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

sys.path.append("..")
from scservo_sdk import *   # Uses SC Servo SDK library

# Default setting
SCS_ID                      = 1                 # SC Servo ID : 1
BAUDRATE                    = 1000000         # SC Servo default baudrate : 1000000
DEVICENAME                  = '/dev/ttyUSB0'    # Adjust this if your USB port differs
SCS_MINIMUM_POSITION_VALUE  = 0                 # Minimum position
SCS_MAXIMUM_POSITION_VALUE  = 4095              # Maximum position
SCS_MOVING_SPEED            = 2400              # SC Servo moving speed
SCS_MOVING_ACC              = 50                # SC Servo moving acceleration

index = 0
scs_goal_position = [SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE]

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = sms_sts(portHandler)

# Open port
if portHandler.openPort():
    print("✅ Succeeded to open the port")
else:
    print("❌ Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("✅ Succeeded to change the baudrate")
else:
    print("❌ Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

while True:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):  # ESC key
        break

    # Write SC Servo goal position, speed, and acceleration
    scs_comm_result, scs_error = packetHandler.WritePosEx(
        SCS_ID, scs_goal_position[index], SCS_MOVING_SPEED, SCS_MOVING_ACC
    )
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

    while True:
        # Read SC Servo present position and speed
        scs_present_position, scs_present_speed, scs_comm_result, scs_error = packetHandler.ReadPosSpeed(SCS_ID)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))
        else:
            print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" %
                  (SCS_ID, scs_goal_position[index], scs_present_position, scs_present_speed))

        if scs_error != 0:
            print(packetHandler.getRxPacketError(scs_error))

        # Check if moving
        moving, scs_comm_result, scs_error = packetHandler.ReadMoving(SCS_ID)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))

        if moving == 0:
            break

    # Toggle goal position
    index = 1 - index

# Close port
portHandler.closePort()
