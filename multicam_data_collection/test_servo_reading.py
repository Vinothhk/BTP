#!/usr/bin/env python
"""
Test script to verify servo position/velocity reading
Run this BEFORE data collection to ensure servos are readable
"""

import sys
import time
sys.path.append("..")
from scservo_sdk import *
import numpy as np

DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 115200
SERVO_IDS = [1, 2, 3, 4, 5, 6]

def test_servo_reading():
    """Test reading positions and velocities from all servos"""
    
    print("🔧 Servo Reading Test")
    print("="*60)
    
    # Open port
    portHandler = PortHandler(DEVICENAME)
    packetHandler = sms_sts(portHandler)
    
    if not portHandler.openPort():
        print(f"❌ Failed to open port {DEVICENAME}")
        return False
    
    if not portHandler.setBaudRate(BAUDRATE):
        print(f"❌ Failed to set baudrate {BAUDRATE}")
        return False
    
    print(f"✅ Port opened: {DEVICENAME} @ {BAUDRATE}")
    
    # Ping servos
    print("\n📡 Pinging servos...")
    for servo_id in SERVO_IDS:
        model, result, error = packetHandler.ping(servo_id)
        if result != COMM_SUCCESS:
            print(f"❌ Servo ID {servo_id} not found")
        else:
            print(f"✅ Servo ID {servo_id} - Model: {model}")
    
    # Test reading positions
    print("\n📊 Reading positions (10 samples)...")
    print("-"*60)
    
    for i in range(10):
        positions = []
        velocities = []
        
        # Read positions
        for servo_id in SERVO_IDS:
            pos, result, error = packetHandler.ReadPos(servo_id)
            if result == COMM_SUCCESS:
                positions.append(pos)
            else:
                positions.append(0)
                print(f"⚠️  Failed to read position from servo {servo_id}")
        
        # Read velocities
        for servo_id in SERVO_IDS:
            vel, result, error = packetHandler.ReadSpeed(servo_id)
            if result == COMM_SUCCESS:
                velocities.append(vel)
            else:
                velocities.append(0)
                print(f"⚠️  Failed to read velocity from servo {servo_id}")
        
        positions = np.array(positions, dtype=np.float32)
        velocities = np.array(velocities, dtype=np.float32)
        
        # Check for all zeros
        if np.all(positions == 0):
            print(f"❌ Sample {i+1}: All positions are ZERO!")
        else:
            print(f"✅ Sample {i+1}: Pos={positions}")
        
        if np.all(velocities == 0):
            print(f"   Velocities: All ZERO (might be stationary)")
        else:
            print(f"   Velocities: {velocities}")
        
        time.sleep(0.1)
    
    # Close port
    portHandler.closePort()
    print("\n" + "="*60)
    print("✅ Test complete")
    
    return True


if __name__ == "__main__":
    try:
        test_servo_reading()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()