#!/usr/bin/env python
import sys
import os
import pygame # type: ignore
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
SERVO_5_ID = 5 
BAUDRATE = 115200

DEVICENAME = '/dev/ttyUSB0'

# Gripper servo limits (Absolute position control)
GRIPPER_MIN_POSITION = 80
GRIPPER_MAX_POSITION = 875
GRIPPER_MOVING_SPEED = 1500
GRIPPER_MOVING_ACC = 50

# Base servo limits (ID 1)
BASE_MIN_POSITION = 250
BASE_MAX_POSITION = 1100
BASE_MAX_SPEED = 200

# Servo 2 limits (ID 2)
SERVO_2_MIN_POSITION = 1130
SERVO_2_MAX_POSITION = 2400
SERVO_2_MAX_SPEED = 200

# Servo 3 limits (ID 3)
SERVO_3_MIN_POSITION = 1370
SERVO_3_MAX_POSITION = 2600
SERVO_3_MAX_SPEED = 200

# Servo 4 limits (ID 4)
SERVO_4_MIN_POSITION = 1080
SERVO_4_MAX_POSITION = 2550
SERVO_4_MAX_SPEED = 200

# Servo 5 limits (ID 5)
SERVO_5_MIN_POSITION = 900
SERVO_5_MAX_POSITION = 3500
SERVO_5_MAX_SPEED = 200

# Acceleration for smooth motion
SERVO_ACC = 150

# ==============================================================================
# 2. JOYSTICK AND CONTROL CONFIGURATION
# ==============================================================================
AXIS_0 = 0  # Left Stick Y (Vertical) - Controls Servo 2
AXIS_1 = 1  # Left Stick X (Horizontal) - Controls Base Servo 1
AXIS_2 = 2  # LT (Left Trigger) - Controls Servo 5
AXIS_3 = 3  # Controls Servo 4
AXIS_4 = 4  # Controls Servo 3
AXIS_5 = 5  # RT trigger - Gripper control

BUTTON_HOME = 2

# Deadzones 
DEADZONE = 0.08

# Home Position Configuration
HOME_POSITIONS = {
    BASE_SERVO_ID: 496,
    SERVO_2_ID: 1170,
    SERVO_3_ID: 2232,
    SERVO_4_ID: 2861,
    SERVO_5_ID: 557,      
    GRIPPER_ID: 80
}

# HOME_POSITIONS = {
#     BASE_SERVO_ID: 815,
#     SERVO_2_ID: 1159,
#     SERVO_3_ID: 2340,
#     SERVO_4_ID: 2805,
#     SERVO_5_ID: 500,      
#     GRIPPER_ID: 215
# }

HOME_SPEED = 150
HOME_ACC = 100

# Lookahead distance multiplier
LOOKAHEAD_MULTIPLIER = 180

# Smoothing parameters
SPEED_SMOOTHING_FACTOR = 0.3
MIN_SPEED = 30
STOP_DECEL_DISTANCE = 25

# Gripper position change threshold
GRIPPER_POS_THRESHOLD = 5

# Feedback reading interval
FEEDBACK_READ_INTERVAL = 1

# ==============================================================================
# 3. UTILITY FUNCTIONS
# ==============================================================================
def clip_axis_to_0_1(value):
    """Clip joystick axis from range [-1, 1] to [0, 1]"""
    # This function is specifically for the Gripper (RT)
    return max(0.0, min(1.0, (value + 1.0) / 2.0))

def map_to_servo_position(normalized_value, min_pos, max_pos):
    """Map 0-1 range to servo position range"""
    return int(min_pos + normalized_value * (max_pos - min_pos))

def apply_deadzone(value, deadzone):
    """Apply deadzone to axis value"""
    if abs(value) < deadzone:
        return 0.0
    return value

def smooth_speed(current_speed, target_speed, smoothing_factor):
    """Exponential smoothing for speed changes"""
    return current_speed + (target_speed - current_speed) * smoothing_factor

def read_servo_position(packet_handler, servo_id):
    """Read current position from servo"""
    scs_present_position, scs_comm_result, scs_error = packet_handler.ReadPos(servo_id)
    if scs_comm_result == COMM_SUCCESS:
        return scs_present_position
    return None

# ==============================================================================
# 4. SERVO STATE TRACKING CLASS
# ==============================================================================
class ServoState:
    def __init__(self, servo_id, min_pos, max_pos, max_speed):
        self.servo_id = servo_id
        self.min_pos = min_pos
        self.max_pos = max_pos
        self.max_speed = max_speed
        self.current_position = (min_pos + max_pos) // 2
        self.last_target = self.current_position
        self.current_speed = 0.0
        self.is_stopping = False
        self.stop_start_pos = None
        
    def update_position(self, pos):
        if pos is not None:
            self.current_position = pos
    
    def calculate_smooth_motion(self, axis_value, lookahead_mult, smoothing_factor):
        """Calculate smooth target position and speed"""
        if abs(axis_value) > 0.001:
            # Active motion
            self.is_stopping = False
            
            target_speed_raw = abs(axis_value) * self.max_speed
            target_speed_raw = max(MIN_SPEED, target_speed_raw)
            
            self.current_speed = smooth_speed(self.current_speed, target_speed_raw, smoothing_factor)
            
            direction = 1 if axis_value > 0 else -1
            lookahead_distance = int(axis_value * lookahead_mult)
            target_position = self.current_position + lookahead_distance
            
            target_position = max(self.min_pos, min(self.max_pos, target_position))
            
            return target_position, int(self.current_speed), False
        
        else:
            # Stopping - smooth deceleration
            if not self.is_stopping:
                self.is_stopping = True
                self.stop_start_pos = self.current_position
            
            if self.current_speed > 5:
                decel_speed = max(0, self.current_speed * 0.6)
                self.current_speed = decel_speed
                
                target_position = self.current_position + int(STOP_DECEL_DISTANCE * (self.current_speed / self.max_speed))
                target_position = max(self.min_pos, min(self.max_pos, target_position))
                
                return target_position, max(5, int(self.current_speed)), False
            else:
                # Fully stopped
                self.current_speed = 0
                return self.current_position, 0, True

# ==============================================================================
# 5. INITIALIZATION
# ==============================================================================
pygame.init()
pygame.joystick.init()
clock = pygame.time.Clock()
LOOP_FREQUENCY = 60

# Joystick Setup
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joystick found! Please connect a controller.")
    quit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Joystick Connected: {joystick.get_name()}")
required_axes = max(AXIS_0, AXIS_1, AXIS_2, AXIS_3, AXIS_4, AXIS_5) + 1 
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
for servo_id in [GRIPPER_ID, BASE_SERVO_ID, SERVO_2_ID, SERVO_3_ID, SERVO_4_ID, SERVO_5_ID]:
    scs_model, scs_comm_result, scs_error = packetHandler.ping(servo_id)
    if scs_comm_result != COMM_SUCCESS:
        print(f"FATAL: Cannot find servo ID {servo_id}!")
        portHandler.closePort()
        pygame.quit()
        quit()
    print(f"Servo ID {servo_id} connected! Model: {scs_model}")

# Initialize servo state trackers for ALL incremental servos
servo_2_state = ServoState(SERVO_2_ID, SERVO_2_MIN_POSITION, SERVO_2_MAX_POSITION, SERVO_2_MAX_SPEED)
base_state = ServoState(BASE_SERVO_ID, BASE_MIN_POSITION, BASE_MAX_POSITION, BASE_MAX_SPEED)
servo_3_state = ServoState(SERVO_3_ID, SERVO_3_MIN_POSITION, SERVO_3_MAX_POSITION, SERVO_3_MAX_SPEED)
servo_4_state = ServoState(SERVO_4_ID, SERVO_4_MIN_POSITION, SERVO_4_MAX_POSITION, SERVO_4_MAX_SPEED)
servo_5_state = ServoState(SERVO_5_ID, SERVO_5_MIN_POSITION, SERVO_5_MAX_POSITION, SERVO_5_MAX_SPEED)

# Read initial positions
print("\nReading initial positions...")
all_servo_states = [base_state, servo_2_state, servo_3_state, servo_4_state, servo_5_state]
for state in all_servo_states:
    pos = read_servo_position(packetHandler, state.servo_id)
    if pos is not None:
        state.update_position(pos)
        state.last_target = pos
    print(f"Servo ID {state.servo_id}: {state.current_position}")

gripper_current_position = read_servo_position(packetHandler, GRIPPER_ID)
if gripper_current_position is None:
    gripper_current_position = GRIPPER_MIN_POSITION
print(f"Gripper (ID {GRIPPER_ID}): {gripper_current_position}")

last_gripper_position = gripper_current_position
feedback_counter = 0

# Home position state tracking
going_home = False
home_button_pressed_last = False

# Track previous axis values for state change detection
prev_axis_0 = 0.0
prev_axis_1 = 0.0
prev_axis_2 = 0.0
prev_axis_3 = 0.0
prev_axis_4 = 0.0

print("\n" + "=" * 60)
print("🤖 ROBOT ARM CONTROL")
print("=" * 60)
print(f"Axis {AXIS_0} (Left Stick Y) - Servo 2 (ID {SERVO_2_ID}) - SMOOTH")
print(f"Axis {AXIS_1} (Left Stick X) - Base Servo (ID {BASE_SERVO_ID}) - SMOOTH")
print(f"Axis {AXIS_2} (Left Trigger) - Servo 5 (ID {SERVO_5_ID}) - SMOOTH") 
print(f"Axis {AXIS_4} - Servo 3 (ID {SERVO_3_ID}) - SMOOTH")
print(f"Axis {AXIS_3} - Servo 4 (ID {SERVO_4_ID}) - SMOOTH")
print(f"Axis {AXIS_5} (RT Trigger) - Gripper (ID {GRIPPER_ID})")
print(f"Button {BUTTON_HOME} - 🏠 Go to Home Position")
print("=" * 60)

# ==============================================================================
# 6. MAIN CONTROL LOOP
# ==============================================================================
running = True
try:
    while running:
        # --- Pygame Event Handling ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False

        # --- Read Button States ---
        home_button_pressed = joystick.get_button(BUTTON_HOME) if joystick.get_numbuttons() > BUTTON_HOME else False
        
        # Detect button press (rising edge)
        if home_button_pressed and not home_button_pressed_last:
            going_home = True
            
            # Send all servos to home position
            packetHandler.WritePosEx(BASE_SERVO_ID, HOME_POSITIONS[BASE_SERVO_ID], HOME_SPEED, HOME_ACC)
            packetHandler.WritePosEx(SERVO_2_ID, HOME_POSITIONS[SERVO_2_ID], HOME_SPEED, HOME_ACC)
            packetHandler.WritePosEx(SERVO_3_ID, HOME_POSITIONS[SERVO_3_ID], HOME_SPEED, HOME_ACC)
            packetHandler.WritePosEx(SERVO_4_ID, HOME_POSITIONS[SERVO_4_ID], HOME_SPEED, HOME_ACC)
            packetHandler.WritePosEx(SERVO_5_ID, HOME_POSITIONS[SERVO_5_ID], HOME_SPEED, HOME_ACC) 
            packetHandler.WritePosEx(GRIPPER_ID, HOME_POSITIONS[GRIPPER_ID], GRIPPER_MOVING_SPEED, GRIPPER_MOVING_ACC)
            
            # CRITICAL FIX: Update current positions/targets for ALL incremental servos after sending home command
            # This ensures manual control resumes from the home position.
            for state in all_servo_states:
                state.current_speed = 0
                state.last_target = HOME_POSITIONS[state.servo_id]
                state.current_position = HOME_POSITIONS[state.servo_id]
            
            # Print initial home command status
            print("\n🏠 GOING TO HOME POSITION...")
            
        home_button_pressed_last = home_button_pressed
        
        # === Read Feedback from All Servos (Now runs every loop) ===
        feedback_counter += 1
        if feedback_counter >= FEEDBACK_READ_INTERVAL:
            feedback_counter = 0
            
            for state in all_servo_states:
                pos = read_servo_position(packetHandler, state.servo_id)
                state.update_position(pos)
            
            gripper_fb = read_servo_position(packetHandler, GRIPPER_ID)
            if gripper_fb is not None:
                gripper_current_position = gripper_fb
        
        # Check if home movement is complete
        if going_home:
            # Check if all incremental servos are close to home position
            all_at_home = True
            for state, home_pos in [(state, HOME_POSITIONS[state.servo_id]) for state in all_servo_states]:
                if abs(state.current_position - home_pos) > 50: # Tolerance of 50
                    all_at_home = False
                    break
            
            if all_at_home:
                going_home = False
                print("✅ HOME POSITION REACHED. Manual control unlocked.\n")
        
        # Skip manual control while going home
        if going_home:
            clock.tick(LOOP_FREQUENCY)
            continue
        
        # --- Read Axis Values ---
        axis_0_value = joystick.get_axis(AXIS_0) if joystick.get_numaxes() > AXIS_0 else 0.0
        axis_1_value = joystick.get_axis(AXIS_1) if joystick.get_numaxes() > AXIS_1 else 0.0
        axis_2_value = joystick.get_axis(AXIS_2) if joystick.get_numaxes() > AXIS_2 else -1.0 # LT axis read
        axis_3_value = joystick.get_axis(AXIS_3) if joystick.get_numaxes() > AXIS_3 else 0.0
        axis_4_value = joystick.get_axis(AXIS_4) if joystick.get_numaxes() > AXIS_4 else 0.0
        axis_5_value = joystick.get_axis(AXIS_5) if joystick.get_numaxes() > AXIS_5 else -1.0
        
        
        # --- SMOOTH CONTROL BLOCKS (Servos 1, 2, 3, 4, 5) ---
        
        # === SERVO 2 CONTROL (ID 2, controlled by AXIS 0) ===
        axis_0_filtered = apply_deadzone(axis_0_value, DEADZONE)
        target_pos, target_speed, fully_stopped = servo_2_state.calculate_smooth_motion(
            axis_0_filtered, LOOKAHEAD_MULTIPLIER, SPEED_SMOOTHING_FACTOR
        )
        if abs(target_pos - servo_2_state.last_target) > 3 or not fully_stopped:
            packetHandler.WritePosEx(SERVO_2_ID, target_pos, target_speed, SERVO_ACC)
            servo_2_state.last_target = target_pos
            if abs(axis_0_filtered) > 0.001 and abs(axis_0_filtered - prev_axis_0) > 0.1:
                dir_name = "DOWN" if axis_0_filtered > 0 else "UP"
                print(f"[Servo2] Speed: {target_speed:3d} | Pos: {servo_2_state.current_position:4d} | {dir_name}")
            elif fully_stopped and abs(prev_axis_0) > 0.001:
                print(f"[Servo2] ✓ Smooth stop at {servo_2_state.current_position}")
        prev_axis_0 = axis_0_filtered


        # === BASE SERVO CONTROL (ID 1, controlled by AXIS 1) ===
        axis_1_filtered = apply_deadzone(axis_1_value, DEADZONE)
        target_pos, target_speed, fully_stopped = base_state.calculate_smooth_motion(
            axis_1_filtered, LOOKAHEAD_MULTIPLIER, SPEED_SMOOTHING_FACTOR
        )
        if abs(target_pos - base_state.last_target) > 3 or not fully_stopped:
            packetHandler.WritePosEx(BASE_SERVO_ID, target_pos, target_speed, SERVO_ACC)
            base_state.last_target = target_pos
            if abs(axis_1_filtered) > 0.001 and abs(axis_1_filtered - prev_axis_1) > 0.1:
                dir_name = "RIGHT" if axis_1_filtered > 0 else "LEFT"
                print(f"[Base]   Speed: {target_speed:3d} | Pos: {base_state.current_position:4d} | {dir_name}")
            elif fully_stopped and abs(prev_axis_1) > 0.001:
                print(f"[Base]   ✓ Smooth stop at {base_state.current_position}")
        prev_axis_1 = axis_1_filtered
                
        
        # === SERVO 3 CONTROL (ID 3, controlled by AXIS 4) ===
        axis_4_filtered = apply_deadzone(axis_4_value, DEADZONE)
        target_pos, target_speed, fully_stopped = servo_3_state.calculate_smooth_motion(
            axis_4_filtered, LOOKAHEAD_MULTIPLIER, SPEED_SMOOTHING_FACTOR
        )
        if abs(target_pos - servo_3_state.last_target) > 3 or not fully_stopped:
            packetHandler.WritePosEx(SERVO_3_ID, target_pos, target_speed, SERVO_ACC)
            servo_3_state.last_target = target_pos
            if abs(axis_4_filtered) > 0.001 and abs(axis_4_filtered - prev_axis_4) > 0.1:
                dir_name = "POS" if axis_4_filtered > 0 else "NEG"
                print(f"[Servo3] Speed: {target_speed:3d} | Pos: {servo_3_state.current_position:4d} | {dir_name}")
            elif fully_stopped and abs(prev_axis_4) > 0.001:
                print(f"[Servo3] ✓ Smooth stop at {servo_3_state.current_position}")
        prev_axis_4 = axis_4_filtered


        # === SERVO 4 CONTROL (ID 4, controlled by AXIS 3) ===
        axis_3_filtered = apply_deadzone(axis_3_value, DEADZONE)
        target_pos, target_speed, fully_stopped = servo_4_state.calculate_smooth_motion(
            axis_3_filtered, LOOKAHEAD_MULTIPLIER, SPEED_SMOOTHING_FACTOR
        )
        if abs(target_pos - servo_4_state.last_target) > 3 or not fully_stopped:
            packetHandler.WritePosEx(SERVO_4_ID, target_pos, target_speed, SERVO_ACC)
            servo_4_state.last_target = target_pos
            if abs(axis_3_filtered) > 0.001 and abs(axis_3_filtered - prev_axis_3) > 0.1:
                dir_name = "POS" if axis_3_filtered > 0 else "NEG"
                print(f"[Servo4] Speed: {target_speed:3d} | Pos: {servo_4_state.current_position:4d} | {dir_name}")
            elif fully_stopped and abs(prev_axis_3) > 0.001:
                print(f"[Servo4] ✓ Smooth stop at {servo_4_state.current_position}")
        prev_axis_3 = axis_3_filtered

        
        # === SERVO 5 CONTROL (ID 5, controlled by AXIS 2 - LT) ===
        
        # CRITICAL FIX: Clip LT axis value (AXIS_2) from [-1.0, 1.0] to [0.0, 1.0].
        # This makes 'released' (-1.0) register as 0.0, ensuring the smooth stop logic works.
        # This behavior is typical for joystick triggers used for linear motion.
        axis_2_clipped_0_1 = max(0.0, (axis_2_value + 1.0) / 2.0)
        
        # Now, filter this [0.0, 1.0] range for deadzone and use it for motion calculation.
        # The motion should be unidirectional (only positive movement from 0.0 to 1.0)
        axis_2_filtered = apply_deadzone(axis_2_clipped_0_1, DEADZONE)
        
        target_pos, target_speed, fully_stopped = servo_5_state.calculate_smooth_motion(
            axis_2_filtered, LOOKAHEAD_MULTIPLIER, SPEED_SMOOTHING_FACTOR
        )
        
        if abs(target_pos - servo_5_state.last_target) > 3 or not fully_stopped:
            packetHandler.WritePosEx(SERVO_5_ID, target_pos, target_speed, SERVO_ACC)
            servo_5_state.last_target = target_pos
            
            # Print logic adjusted for unidirectional axis (0.0 to 1.0)
            if abs(axis_2_filtered) > 0.001 and abs(axis_2_filtered - prev_axis_2) > 0.1:
                dir_name = "FWD" if target_pos > servo_5_state.current_position else "BWD" # Direction based on position change
                print(f"[Servo5] Trigger: {axis_2_value:+.3f} | Speed: {target_speed:3d} | Pos: {servo_5_state.current_position:4d} | {dir_name}")
            elif fully_stopped and abs(prev_axis_2) > 0.001:
                print(f"[Servo5] ✓ Smooth stop at {servo_5_state.current_position}")
        
        prev_axis_2 = axis_2_filtered


        # --- GRIPPER CONTROL (ID 6, Axis 5) ---
        gripper_control = clip_axis_to_0_1(axis_5_value)
        target_gripper_position = map_to_servo_position(gripper_control, GRIPPER_MIN_POSITION, GRIPPER_MAX_POSITION)

        if abs(target_gripper_position - last_gripper_position) >= GRIPPER_POS_THRESHOLD:
            scs_comm_result, scs_error = packetHandler.WritePosEx(
                GRIPPER_ID, target_gripper_position, GRIPPER_MOVING_SPEED, GRIPPER_MOVING_ACC
            )

            if scs_comm_result == COMM_SUCCESS and scs_error == 0:
                last_gripper_position = target_gripper_position
                status = 'CLOSED' if gripper_control < 0.1 else 'OPEN' if gripper_control > 0.9 else 'PARTIAL'
                print(f"[Gripper] Position: {target_gripper_position:4d} ({status})")

        # --- Loop Timing ---
        clock.tick(LOOP_FREQUENCY)

except KeyboardInterrupt:
    print("\n⚠️ Interrupted by user")

finally:
    # EMERGENCY STOP with smooth deceleration
    print("\n🛑 EMERGENCY STOP - Smooth shutdown...")
    
    # Define a list of all servo states for final stop
    all_servo_states = [base_state, servo_2_state, servo_3_state, servo_4_state, servo_5_state]
    
    for state in all_servo_states:
        pos = read_servo_position(packetHandler, state.servo_id)
        if pos is not None:
            # Send current position with very low speed for gentle stop
            packetHandler.WritePosEx(state.servo_id, pos, 10, SERVO_ACC)
    
    time.sleep(0.2)
    
    print("Closing connections...")
    portHandler.closePort()
    pygame.quit()
    print("✅ Shutdown complete")