# #!/usr/bin/env python
# """
# Robot Teleoperation with Dual-Camera Recording
# MATCHES YOUR EXACT CONTROL LOGIC:
# - Incremental gripper control (LB/RB buttons)
# - Bidirectional Servo 5 (LT + RT triggers)
# - Proper holding speed
# """

# import sys
# import os
# import pygame
# import time
# import numpy as np

# sys.path.append("..")
# from scservo_sdk import *
# from data_collector import MultiCameraDataCollector, ServoReader

# # ==============================================================================
# # SERVO CONFIGURATION (MATCHES YOUR ORIGINAL)
# # ==============================================================================
# SERVO_CONFIG = {
#     'port': '/dev/ttyUSB0',
#     'baudrate': 115200,
#     'ids': {
#         'base': 1,
#         'servo_2': 2,
#         'servo_3': 3,
#         'servo_4': 4,
#         'servo_5': 5,
#         'gripper': 6
#     },
#     'limits': {
#         1: {'min': 250, 'max': 1100, 'speed': 200},
#         2: {'min': 1130, 'max': 2400, 'speed': 200},
#         3: {'min': 1370, 'max': 2600, 'speed': 200},
#         4: {'min': 1080, 'max': 2850, 'speed': 200},
#         5: {'min': 200, 'max': 3500, 'speed': 200},
#         6: {'min': 80, 'max': 875, 'speed': 1500}
#     },
#     'home': {
#         1: 496,
#         2: 1170,
#         3: 2232,
#         4: 2861,
#         5: 760
#         # Gripper NOT included in home
#     }
# }

# # Camera Configuration
# CAMERA_CONFIG = {
#     'indices': [2, 4],
#     'cam_size': (640, 480),
#     'target_size': (96, 96)
# }

# # Data Collection Configuration
# DATA_CONFIG = {
#     'dataset_path': './collected_data',
#     'fps': 10,
#     'buffer_size': 50
# }

# # Joystick Configuration (MATCHES YOUR ORIGINAL)
# JOYSTICK_CONFIG = {
#     'axes': {
#         'base': 0,      # Left Stick Y → Base
#         'servo_2': 1,   # Left Stick X → Servo 2
#         'servo_5_fwd': 2,   # LT → Servo 5 forward
#         'servo_4': 3,   # → Servo 4
#         'servo_3': 4,   # → Servo 3
#         'servo_5_bwd': 5    # RT → Servo 5 backward
#     },
#     'buttons': {
#         'start_record': 0,   # A button
#         'stop_record': 1,    # B button
#         'home': 2,           # X button
#         'gripper_open': 4,   # LB button
#         'gripper_close': 5   # RB button
#     },
#     'deadzone': 0.08
# }

# # Control Parameters (MATCHES YOUR ORIGINAL)
# CONTROL_PARAMS = {
#     'lookahead': 180,
#     'smoothing': 0.3,
#     'min_speed': 30,
#     'decel_distance': 25,
#     'acc': 150,
#     'holding_speed': 150,  # ✅ CRITICAL: Torque when stopped
#     'home_speed': 150,
#     'home_acc': 100,
#     'gripper_increment': 30,  # ✅ Incremental gripper control
#     'gripper_acc': 50
# }

# LOOP_FREQUENCY = 60

# # ==============================================================================
# # SERVO STATE CLASS (MATCHES YOUR ORIGINAL)
# # ==============================================================================
# class ServoState:
#     def __init__(self, servo_id, min_pos, max_pos, max_speed):
#         self.servo_id = servo_id
#         self.min_pos = min_pos
#         self.max_pos = max_pos
#         self.max_speed = max_speed
#         self.current_position = (min_pos + max_pos) // 2
#         self.last_target = self.current_position
#         self.current_speed = 0.0
#         self.is_stopping = False
#         self.hold_position = None  # ✅ Locked position when stopped
    
#     def update_position(self, pos):
#         if pos is not None:
#             self.current_position = pos
    
#     def calculate_smooth_motion(self, axis_value, lookahead, smoothing, min_speed, decel_dist, holding_speed):
#         if abs(axis_value) > 0.001:
#             # Active motion
#             self.is_stopping = False
#             self.hold_position = None
            
#             target_speed_raw = abs(axis_value) * self.max_speed
#             target_speed_raw = max(min_speed, target_speed_raw)
#             self.current_speed = self.current_speed + (target_speed_raw - self.current_speed) * smoothing
            
#             lookahead_distance = int(axis_value * lookahead)
#             target_position = self.current_position + lookahead_distance
#             target_position = max(self.min_pos, min(self.max_pos, target_position))
            
#             return target_position, int(self.current_speed), False
#         else:
#             # Stopping - smooth deceleration
#             if not self.is_stopping:
#                 self.is_stopping = True
#                 self.hold_position = None
            
#             if self.current_speed > 5:
#                 decel_speed = max(0, self.current_speed * 0.6)
#                 self.current_speed = decel_speed
#                 target_position = self.current_position + int(decel_dist * (self.current_speed / self.max_speed))
#                 target_position = max(self.min_pos, min(self.max_pos, target_position))
#                 return target_position, max(5, int(self.current_speed)), False
#             else:
#                 # ✅ CRITICAL: Hold with torque
#                 self.current_speed = 0
#                 if self.hold_position is None:
#                     self.hold_position = self.current_position
#                 return self.hold_position, holding_speed, True

# # ==============================================================================
# # UTILITY FUNCTIONS
# # ==============================================================================
# def apply_deadzone(value, deadzone):
#     return 0.0 if abs(value) < deadzone else value

# # ==============================================================================
# # MAIN PROGRAM
# # ==============================================================================
# def main():
#     # Initialize Pygame
#     pygame.init()
#     pygame.joystick.init()
#     clock = pygame.time.Clock()
    
#     if pygame.joystick.get_count() == 0:
#         print("❌ No joystick found!")
#         return
    
#     joystick = pygame.joystick.Joystick(0)
#     joystick.init()
#     print(f"🎮 Joystick: {joystick.get_name()}")
    
#     # Servo setup
#     portHandler = PortHandler(SERVO_CONFIG['port'])
#     packetHandler = sms_sts(portHandler)
    
#     if not portHandler.openPort() or not portHandler.setBaudRate(SERVO_CONFIG['baudrate']):
#         print(f"❌ Failed to open port {SERVO_CONFIG['port']}")
#         pygame.quit()
#         return
#     print("✅ Servo port opened")
    
#     # Ping servos
#     for name, servo_id in SERVO_CONFIG['ids'].items():
#         model, result, error = packetHandler.ping(servo_id)
#         if result != COMM_SUCCESS:
#             print(f"❌ Cannot find servo {name} (ID {servo_id})")
#             portHandler.closePort()
#             pygame.quit()
#             return
#         print(f"✅ Servo {name} (ID {servo_id}) connected")
    
#     # Initialize servo states (excluding gripper)
#     servo_states = {}
#     for servo_id, limits in SERVO_CONFIG['limits'].items():
#         if servo_id != SERVO_CONFIG['ids']['gripper']:
#             servo_states[servo_id] = ServoState(
#                 servo_id, limits['min'], limits['max'], limits['speed']
#             )
    
#     # Read initial positions
#     print("\n📍 Reading initial positions...")
#     for servo_id, state in servo_states.items():
#         pos, _, _ = packetHandler.ReadPos(servo_id)
#         if pos is not None:
#             state.update_position(pos)
#             state.last_target = pos
#         print(f"  Servo {servo_id}: {state.current_position}")
    
#     gripper_id = SERVO_CONFIG['ids']['gripper']
#     gripper_pos, _, _ = packetHandler.ReadPos(gripper_id)
#     if gripper_pos is None:
#         gripper_pos = SERVO_CONFIG['limits'][gripper_id]['min']
#     print(f"  Gripper: {gripper_pos}")
    
#     # ==============================================================================
#     # DATA COLLECTOR INITIALIZATION
#     # ==============================================================================
#     print("\n🎥 Initializing data collector...")
#     data_collector = MultiCameraDataCollector(
#         dataset_path=DATA_CONFIG['dataset_path'],
#         cam_indices=CAMERA_CONFIG['indices'],
#         cam_size=CAMERA_CONFIG['cam_size'],
#         target_size=CAMERA_CONFIG['target_size'],
#         fps=DATA_CONFIG['fps'],
#         buffer_size=DATA_CONFIG['buffer_size']
#     )
    
#     # ✅ CRITICAL FIX: Share the packet handler with ServoReader
#     servo_reader = ServoReader(packet_handler=packetHandler)
    
#     # State tracking
#     going_home = False
#     prev_buttons = {name: False for name in JOYSTICK_CONFIG['buttons'].keys()}
#     last_record_time = time.time()
#     feedback_counter = 0
    
#     print("\n" + "="*70)
#     print("🤖 ROBOT ARM CONTROL WITH DUAL-CAMERA RECORDING")
#     print("="*70)
#     print("RECORDING CONTROLS:")
#     print("  Button A (0) - 🎬 START EPISODE")
#     print("  Button B (1) - ⏹️  STOP EPISODE")
#     print("\nARM CONTROLS:")
#     print("  Button X (2) - 🏠 GO HOME (ARM ONLY)")
#     print("  Left Stick Y - Base Servo")
#     print("  Left Stick X - Servo 2")
#     print("  LT (Axis 2)  - Servo 5 FORWARD")
#     print("  RT (Axis 5)  - Servo 5 BACKWARD")
#     print("  Axis 3       - Servo 4")
#     print("  Axis 4       - Servo 3")
#     print("\nGRIPPER CONTROLS:")
#     print("  LB (Button 4) - 🤏 OPEN incrementally")
#     print("  RB (Button 5) - 🤌 CLOSE incrementally")
#     print("\nESC - Exit and save")
#     print("="*70)
    
#     # ==============================================================================
#     # MAIN CONTROL LOOP
#     # ==============================================================================
#     running = True
#     try:
#         while running:
#             # Event handling
#             for event in pygame.event.get():
#                 if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
#                     running = False
            
#             # Read button states
#             buttons = {
#                 name: joystick.get_button(idx) if joystick.get_numbuttons() > idx else False
#                 for name, idx in JOYSTICK_CONFIG['buttons'].items()
#             }
            
#             # === RECORDING CONTROLS ===
#             if buttons['start_record'] and not prev_buttons['start_record']:
#                 if not data_collector.is_recording:
#                     data_collector.start_episode()
            
#             if buttons['stop_record'] and not prev_buttons['stop_record']:
#                 if data_collector.is_recording:
#                     data_collector.end_episode()
            
#             # === GRIPPER INCREMENTAL CONTROL (BUTTONS) ===
#             if buttons['gripper_open'] and not prev_buttons['gripper_open']:
#                 new_gripper_pos = min(
#                     gripper_pos + CONTROL_PARAMS['gripper_increment'],
#                     SERVO_CONFIG['limits'][gripper_id]['max']
#                 )
#                 packetHandler.WritePosEx(
#                     gripper_id, new_gripper_pos,
#                     SERVO_CONFIG['limits'][gripper_id]['speed'],
#                     CONTROL_PARAMS['gripper_acc']
#                 )
#                 gripper_pos = new_gripper_pos
#                 print(f"[Gripper] OPENING → {new_gripper_pos}")
            
#             if buttons['gripper_close'] and not prev_buttons['gripper_close']:
#                 new_gripper_pos = max(
#                     gripper_pos - CONTROL_PARAMS['gripper_increment'],
#                     SERVO_CONFIG['limits'][gripper_id]['min']
#                 )
#                 packetHandler.WritePosEx(
#                     gripper_id, new_gripper_pos,
#                     SERVO_CONFIG['limits'][gripper_id]['speed'],
#                     CONTROL_PARAMS['gripper_acc']
#                 )
#                 gripper_pos = new_gripper_pos
#                 print(f"[Gripper] CLOSING → {new_gripper_pos}")
            
#             # === HOME BUTTON ===
#             if buttons['home'] and not prev_buttons['home']:
#                 going_home = True
#                 print("\n🏠 GOING HOME (ARM ONLY)...")
#                 for servo_id, home_pos in SERVO_CONFIG['home'].items():
#                     packetHandler.WritePosEx(
#                         servo_id, home_pos,
#                         CONTROL_PARAMS['home_speed'],
#                         CONTROL_PARAMS['home_acc']
#                     )
#                 for state in servo_states.values():
#                     state.current_speed = 0
#                     state.last_target = SERVO_CONFIG['home'][state.servo_id]
#                     state.hold_position = SERVO_CONFIG['home'][state.servo_id]
            
#             prev_buttons = buttons.copy()
            
#             # === FEEDBACK READING ===
#             feedback_counter += 1
#             if feedback_counter >= 1:
#                 feedback_counter = 0
#                 for servo_id, state in servo_states.items():
#                     pos, _, _ = packetHandler.ReadPos(servo_id)
#                     state.update_position(pos)
#                 gripper_fb, _, _ = packetHandler.ReadPos(gripper_id)
#                 if gripper_fb is not None:
#                     gripper_pos = gripper_fb
            
#             # Check if home complete
#             if going_home:
#                 all_at_home = all(
#                     abs(state.current_position - SERVO_CONFIG['home'][state.servo_id]) < 50
#                     for state in servo_states.values()
#                 )
#                 if all_at_home:
#                     going_home = False
#                     print("✅ HOME REACHED\n")
            
#             if going_home:
#                 clock.tick(LOOP_FREQUENCY)
#                 continue
            
#             # === READ AXES ===
#             axes = {
#                 name: joystick.get_axis(idx) if joystick.get_numaxes() > idx else (-1.0 if 'servo_5' in name else 0.0)
#                 for name, idx in JOYSTICK_CONFIG['axes'].items()
#             }
            
#             # === CONTROL SERVOS ===
            
#             # Base Servo (Axis 0)
#             axis_filtered = apply_deadzone(axes['base'], JOYSTICK_CONFIG['deadzone'])
#             target_pos, target_speed, need_update = servo_states[1].calculate_smooth_motion(
#                 axis_filtered, CONTROL_PARAMS['lookahead'], CONTROL_PARAMS['smoothing'],
#                 CONTROL_PARAMS['min_speed'], CONTROL_PARAMS['decel_distance'],
#                 CONTROL_PARAMS['holding_speed']
#             )
#             if need_update or abs(target_pos - servo_states[1].last_target) > 3:
#                 packetHandler.WritePosEx(1, target_pos, target_speed, CONTROL_PARAMS['acc'])
#                 servo_states[1].last_target = target_pos
            
#             # Servo 2 (Axis 1)
#             axis_filtered = apply_deadzone(axes['servo_2'], JOYSTICK_CONFIG['deadzone'])
#             target_pos, target_speed, need_update = servo_states[2].calculate_smooth_motion(
#                 axis_filtered, CONTROL_PARAMS['lookahead'], CONTROL_PARAMS['smoothing'],
#                 CONTROL_PARAMS['min_speed'], CONTROL_PARAMS['decel_distance'],
#                 CONTROL_PARAMS['holding_speed']
#             )
#             if need_update or abs(target_pos - servo_states[2].last_target) > 3:
#                 packetHandler.WritePosEx(2, target_pos, target_speed, CONTROL_PARAMS['acc'])
#                 servo_states[2].last_target = target_pos
            
#             # Servo 3 (Axis 4)
#             axis_filtered = apply_deadzone(axes['servo_3'], JOYSTICK_CONFIG['deadzone'])
#             target_pos, target_speed, need_update = servo_states[3].calculate_smooth_motion(
#                 axis_filtered, CONTROL_PARAMS['lookahead'], CONTROL_PARAMS['smoothing'],
#                 CONTROL_PARAMS['min_speed'], CONTROL_PARAMS['decel_distance'],
#                 CONTROL_PARAMS['holding_speed']
#             )
#             if need_update or abs(target_pos - servo_states[3].last_target) > 3:
#                 packetHandler.WritePosEx(3, target_pos, target_speed, CONTROL_PARAMS['acc'])
#                 servo_states[3].last_target = target_pos
            
#             # Servo 4 (Axis 3)
#             axis_filtered = apply_deadzone(axes['servo_4'], JOYSTICK_CONFIG['deadzone'])
#             target_pos, target_speed, need_update = servo_states[4].calculate_smooth_motion(
#                 axis_filtered, CONTROL_PARAMS['lookahead'], CONTROL_PARAMS['smoothing'],
#                 CONTROL_PARAMS['min_speed'], CONTROL_PARAMS['decel_distance'],
#                 CONTROL_PARAMS['holding_speed']
#             )
#             if need_update or abs(target_pos - servo_states[4].last_target) > 3:
#                 packetHandler.WritePosEx(4, target_pos, target_speed, CONTROL_PARAMS['acc'])
#                 servo_states[4].last_target = target_pos
            
#             # ✅ Servo 5 BIDIRECTIONAL (LT + RT)
#             axis_2_clipped = max(0.0, (axes['servo_5_fwd'] + 1.0) / 2.0)  # LT forward
#             axis_5_clipped = max(0.0, (axes['servo_5_bwd'] + 1.0) / 2.0)  # RT backward
#             combined = axis_2_clipped - axis_5_clipped  # Bidirectional
#             axis_filtered = apply_deadzone(combined, JOYSTICK_CONFIG['deadzone'])
            
#             target_pos, target_speed, need_update = servo_states[5].calculate_smooth_motion(
#                 axis_filtered, CONTROL_PARAMS['lookahead'], CONTROL_PARAMS['smoothing'],
#                 CONTROL_PARAMS['min_speed'], CONTROL_PARAMS['decel_distance'],
#                 CONTROL_PARAMS['holding_speed']
#             )
#             if need_update or abs(target_pos - servo_states[5].last_target) > 3:
#                 packetHandler.WritePosEx(5, target_pos, target_speed, CONTROL_PARAMS['acc'])
#                 servo_states[5].last_target = target_pos
            
#             # === DATA RECORDING ===
#             current_time = time.time()
#             if data_collector.is_recording and (current_time - last_record_time) >= (1.0 / DATA_CONFIG['fps']):
#                 # Read all servo data
#                 joint_pos, joint_vel = servo_reader.read_all()
                
#                 # Action = last commanded positions
#                 action = np.array([
#                     servo_states[1].last_target,
#                     servo_states[2].last_target,
#                     servo_states[3].last_target,
#                     servo_states[4].last_target,
#                     servo_states[5].last_target,
#                     gripper_pos
#                 ], dtype=np.float32)
                
#                 # Record
#                 data_collector.record_step(action, joint_pos, joint_vel)
#                 last_record_time = current_time
            
#             clock.tick(LOOP_FREQUENCY)
    
#     except KeyboardInterrupt:
#         print("\n⚠️  Interrupted by user")
    
#     finally:
#         print("\n🛑 Shutting down...")
        
#         # ✅ EMERGENCY STOP with holding position
#         for servo_id, state in servo_states.items():
#             pos, _, _ = packetHandler.ReadPos(servo_id)
#             if pos is not None:
#                 packetHandler.WritePosEx(servo_id, pos, CONTROL_PARAMS['holding_speed'], CONTROL_PARAMS['acc'])
        
#         # Lock gripper
#         gripper_fb, _, _ = packetHandler.ReadPos(gripper_id)
#         if gripper_fb is not None:
#             packetHandler.WritePosEx(
#                 gripper_id, gripper_fb,
#                 SERVO_CONFIG['limits'][gripper_id]['speed'],
#                 CONTROL_PARAMS['gripper_acc']
#             )
        
#         time.sleep(0.2)
        
#         # Close connections
#         data_collector.close()
#         servo_reader.close()  # Won't close port (shared)
#         portHandler.closePort()  # Close once at the end
#         pygame.quit()
        
#         # Show stats
#         data_collector.get_stats()
        
#         print("\n✅ Shutdown complete")

# if __name__ == "__main__":
#     main()


#!/usr/bin/env python
"""
Robot Teleoperation with Dual-Camera Recording
MATCHES YOUR EXACT CONTROL LOGIC:
- Incremental gripper control (LB/RB buttons)
- Bidirectional Servo 5 (LT + RT triggers)
- Proper holding speed
"""

import sys
import os
import pygame
import time
import numpy as np

sys.path.append("..")
from scservo_sdk import *
from data_collector import MultiCameraDataCollector, ServoReader

# ==============================================================================
# SERVO CONFIGURATION (MATCHES YOUR ORIGINAL)
# ==============================================================================
SERVO_CONFIG = {
    'port': '/dev/ttyUSB0',
    'baudrate': 115200,
    'ids': {
        'base': 1,
        'servo_2': 2,
        'servo_3': 3,
        'servo_4': 4,
        'servo_5': 5,
        'gripper': 6
    },
    'limits': {
        1: {'min': 250, 'max': 1100, 'speed': 200},
        2: {'min': 1130, 'max': 2400, 'speed': 200},
        3: {'min': 1370, 'max': 2600, 'speed': 200},
        4: {'min': 1080, 'max': 2850, 'speed': 200},
        5: {'min': 200, 'max': 3500, 'speed': 200},
        6: {'min': 80, 'max': 875, 'speed': 2500}  # **IMPROVED: 1500 → 2500**
    },
    'home': {
        1: 496,
        2: 1170,
        3: 2232,
        4: 2861,
        5: 760
        # Gripper NOT included in home
    }
}

# Camera Configuration
CAMERA_CONFIG = {
    'indices': [2, 4],
    'cam_size': (640, 480),
    'target_size': (96, 96)
}

# Data Collection Configuration
DATA_CONFIG = {
    'dataset_path': './collected_data',
    'fps': 10,
    'buffer_size': 50
}

# Joystick Configuration (MATCHES YOUR ORIGINAL)
JOYSTICK_CONFIG = {
    'axes': {
        'base': 0,      # Left Stick Y → Base
        'servo_2': 1,   # Left Stick X → Servo 2
        'servo_5_fwd': 2,   # LT → Servo 5 forward
        'servo_4': 3,   # → Servo 4
        'servo_3': 4,   # → Servo 3
        'servo_5_bwd': 5    # RT → Servo 5 backward
    },
    'buttons': {
        'start_record': 0,   # A button
        'stop_record': 1,    # B button
        'home': 2,           # X button
        'gripper_open': 4,   # LB button
        'gripper_close': 5   # RB button
    },
    'deadzone': 0.08
}

# Control Parameters (MATCHES YOUR ORIGINAL)
CONTROL_PARAMS = {
    'lookahead': 180,
    'smoothing': 0.3,
    'min_speed': 30,
    'acc': 150,
    'holding_speed': 150,  # ✅ CRITICAL: Torque when stopped
    'home_speed': 150,
    'home_acc': 100,
    'gripper_increment': 50,  # **IMPROVED: 30 → 50**
    'gripper_acc': 100  # **IMPROVED: 50 → 100**
}

LOOP_FREQUENCY = 60
GRIPPER_BUTTON_DELAY = 2  # **IMPROVED: Frames between gripper commands**

# ==============================================================================
# SERVO STATE CLASS (IMPROVED: Immediate stop response)
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
        self.hold_position = None
    
    def update_position(self, pos):
        if pos is not None:
            self.current_position = pos
    
    def calculate_smooth_motion(self, axis_value, lookahead, smoothing, min_speed, decel_dist, holding_speed):
        if abs(axis_value) > 0.001:
            # Active motion
            self.is_stopping = False
            self.hold_position = None
            
            target_speed_raw = abs(axis_value) * self.max_speed
            target_speed_raw = max(min_speed, target_speed_raw)
            self.current_speed = self.current_speed + (target_speed_raw - self.current_speed) * smoothing
            
            lookahead_distance = int(axis_value * lookahead)
            target_position = self.current_position + lookahead_distance
            target_position = max(self.min_pos, min(self.max_pos, target_position))
            
            return target_position, int(self.current_speed), False
        else:
            # **IMPROVEMENT 2: IMMEDIATE STOP - No deceleration phase**
            self.current_speed = 0
            if self.hold_position is None:
                self.hold_position = self.current_position
            return self.hold_position, holding_speed, True

# ==============================================================================
# UTILITY FUNCTIONS
# ==============================================================================
def apply_deadzone(value, deadzone):
    return 0.0 if abs(value) < deadzone else value

# ==============================================================================
# MAIN PROGRAM
# ==============================================================================
def main():
    # Initialize Pygame
    pygame.init()
    pygame.joystick.init()
    clock = pygame.time.Clock()
    
    if pygame.joystick.get_count() == 0:
        print("❌ No joystick found!")
        return
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"🎮 Joystick: {joystick.get_name()}")
    
    # Servo setup
    portHandler = PortHandler(SERVO_CONFIG['port'])
    packetHandler = sms_sts(portHandler)
    
    if not portHandler.openPort() or not portHandler.setBaudRate(SERVO_CONFIG['baudrate']):
        print(f"❌ Failed to open port {SERVO_CONFIG['port']}")
        pygame.quit()
        return
    print("✅ Servo port opened")
    
    # Ping servos
    for name, servo_id in SERVO_CONFIG['ids'].items():
        model, result, error = packetHandler.ping(servo_id)
        if result != COMM_SUCCESS:
            print(f"❌ Cannot find servo {name} (ID {servo_id})")
            portHandler.closePort()
            pygame.quit()
            return
        print(f"✅ Servo {name} (ID {servo_id}) connected")
    
    # Initialize servo states (excluding gripper)
    servo_states = {}
    for servo_id, limits in SERVO_CONFIG['limits'].items():
        if servo_id != SERVO_CONFIG['ids']['gripper']:
            servo_states[servo_id] = ServoState(
                servo_id, limits['min'], limits['max'], limits['speed']
            )
    
    # Read initial positions
    print("\n📍 Reading initial positions...")
    for servo_id, state in servo_states.items():
        pos, _, _ = packetHandler.ReadPos(servo_id)
        if pos is not None:
            state.update_position(pos)
            state.last_target = pos
        print(f"  Servo {servo_id}: {state.current_position}")
    
    gripper_id = SERVO_CONFIG['ids']['gripper']
    gripper_pos, _, _ = packetHandler.ReadPos(gripper_id)
    if gripper_pos is None:
        gripper_pos = SERVO_CONFIG['limits'][gripper_id]['min']
    print(f"  Gripper: {gripper_pos}")
    
    # ==============================================================================
    # DATA COLLECTOR INITIALIZATION
    # ==============================================================================
    print("\n🎥 Initializing data collector...")
    data_collector = MultiCameraDataCollector(
        dataset_path=DATA_CONFIG['dataset_path'],
        cam_indices=CAMERA_CONFIG['indices'],
        cam_size=CAMERA_CONFIG['cam_size'],
        target_size=CAMERA_CONFIG['target_size'],
        fps=DATA_CONFIG['fps'],
        buffer_size=DATA_CONFIG['buffer_size']
    )
    
    servo_reader = ServoReader(packet_handler=packetHandler)
    
    # State tracking
    going_home = False
    prev_buttons = {name: False for name in JOYSTICK_CONFIG['buttons'].keys()}
    last_record_time = time.time()
    feedback_counter = 0
    gripper_button_cooldown = 0  # **IMPROVED: Cooldown tracker**
    
    print("\n" + "="*70)
    print("🤖 ROBOT ARM CONTROL WITH DUAL-CAMERA RECORDING")
    print("="*70)
    print("RECORDING CONTROLS:")
    print("  Button A (0) - 🎬 START EPISODE")
    print("  Button B (1) - ⏹️  STOP EPISODE")
    print("\nARM CONTROLS:")
    print("  Button X (2) - 🏠 GO HOME (ARM ONLY)")
    print("  Left Stick Y - Base Servo")
    print("  Left Stick X - Servo 2")
    print("  LT (Axis 2)  - Servo 5 FORWARD")
    print("  RT (Axis 5)  - Servo 5 BACKWARD")
    print("  Axis 3       - Servo 4")
    print("  Axis 4       - Servo 3")
    print("\nGRIPPER CONTROLS:")
    print("  LB (Button 4) - 🤏 OPEN (hold for continuous)")
    print("  RB (Button 5) - 🤌 CLOSE (hold for continuous)")
    print("\nESC - Exit and save")
    print("="*70)
    
    # ==============================================================================
    # MAIN CONTROL LOOP
    # ==============================================================================
    running = True
    try:
        while running:
            # Event handling
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    running = False
            
            # Read button states
            buttons = {
                name: joystick.get_button(idx) if joystick.get_numbuttons() > idx else False
                for name, idx in JOYSTICK_CONFIG['buttons'].items()
            }
            
            # === RECORDING CONTROLS ===
            if buttons['start_record'] and not prev_buttons['start_record']:
                if not data_collector.is_recording:
                    data_collector.start_episode()
            
            if buttons['stop_record'] and not prev_buttons['stop_record']:
                if data_collector.is_recording:
                    data_collector.end_episode()
            
            # **IMPROVEMENT 1: Continuous gripper control with rate limiting**
            gripper_button_cooldown = max(0, gripper_button_cooldown - 1)
            
            if gripper_button_cooldown == 0:
                if buttons['gripper_open']:
                    new_gripper_pos = min(
                        gripper_pos + CONTROL_PARAMS['gripper_increment'],
                        SERVO_CONFIG['limits'][gripper_id]['max']
                    )
                    packetHandler.WritePosEx(
                        gripper_id, new_gripper_pos,
                        SERVO_CONFIG['limits'][gripper_id]['speed'],
                        CONTROL_PARAMS['gripper_acc']
                    )
                    gripper_pos = new_gripper_pos
                    gripper_button_cooldown = GRIPPER_BUTTON_DELAY
                    print(f"[Gripper] OPENING → {new_gripper_pos}")
                
                elif buttons['gripper_close']:
                    new_gripper_pos = max(
                        gripper_pos - CONTROL_PARAMS['gripper_increment'],
                        SERVO_CONFIG['limits'][gripper_id]['min']
                    )
                    packetHandler.WritePosEx(
                        gripper_id, new_gripper_pos,
                        SERVO_CONFIG['limits'][gripper_id]['speed'],
                        CONTROL_PARAMS['gripper_acc']
                    )
                    gripper_pos = new_gripper_pos
                    gripper_button_cooldown = GRIPPER_BUTTON_DELAY
                    print(f"[Gripper] CLOSING → {new_gripper_pos}")
            
            # === HOME BUTTON ===
            if buttons['home'] and not prev_buttons['home']:
                going_home = True
                print("\n🏠 GOING HOME (ARM ONLY)...")
                for servo_id, home_pos in SERVO_CONFIG['home'].items():
                    packetHandler.WritePosEx(
                        servo_id, home_pos,
                        CONTROL_PARAMS['home_speed'],
                        CONTROL_PARAMS['home_acc']
                    )
                for state in servo_states.values():
                    state.current_speed = 0
                    state.last_target = SERVO_CONFIG['home'][state.servo_id]
                    state.hold_position = SERVO_CONFIG['home'][state.servo_id]
            
            prev_buttons = buttons.copy()
            
            # === FEEDBACK READING ===
            feedback_counter += 1
            if feedback_counter >= 1:
                feedback_counter = 0
                for servo_id, state in servo_states.items():
                    pos, _, _ = packetHandler.ReadPos(servo_id)
                    state.update_position(pos)
                gripper_fb, _, _ = packetHandler.ReadPos(gripper_id)
                if gripper_fb is not None:
                    gripper_pos = gripper_fb
            
            # Check if home complete
            if going_home:
                all_at_home = all(
                    abs(state.current_position - SERVO_CONFIG['home'][state.servo_id]) < 50
                    for state in servo_states.values()
                )
                if all_at_home:
                    going_home = False
                    print("✅ HOME REACHED\n")
            
            if going_home:
                clock.tick(LOOP_FREQUENCY)
                continue
            
            # === READ AXES ===
            axes = {
                name: joystick.get_axis(idx) if joystick.get_numaxes() > idx else (-1.0 if 'servo_5' in name else 0.0)
                for name, idx in JOYSTICK_CONFIG['axes'].items()
            }
            
            # === CONTROL SERVOS ===
            
            # Base Servo (Axis 0)
            axis_filtered = apply_deadzone(axes['base'], JOYSTICK_CONFIG['deadzone'])
            target_pos, target_speed, need_update = servo_states[1].calculate_smooth_motion(
                axis_filtered, CONTROL_PARAMS['lookahead'], CONTROL_PARAMS['smoothing'],
                CONTROL_PARAMS['min_speed'], 0,  # decel_distance unused now
                CONTROL_PARAMS['holding_speed']
            )
            if need_update or abs(target_pos - servo_states[1].last_target) > 3:
                packetHandler.WritePosEx(1, target_pos, target_speed, CONTROL_PARAMS['acc'])
                servo_states[1].last_target = target_pos
            
            # Servo 2 (Axis 1)
            axis_filtered = apply_deadzone(axes['servo_2'], JOYSTICK_CONFIG['deadzone'])
            target_pos, target_speed, need_update = servo_states[2].calculate_smooth_motion(
                axis_filtered, CONTROL_PARAMS['lookahead'], CONTROL_PARAMS['smoothing'],
                CONTROL_PARAMS['min_speed'], 0,
                CONTROL_PARAMS['holding_speed']
            )
            if need_update or abs(target_pos - servo_states[2].last_target) > 3:
                packetHandler.WritePosEx(2, target_pos, target_speed, CONTROL_PARAMS['acc'])
                servo_states[2].last_target = target_pos
            
            # Servo 3 (Axis 4)
            axis_filtered = apply_deadzone(axes['servo_3'], JOYSTICK_CONFIG['deadzone'])
            target_pos, target_speed, need_update = servo_states[3].calculate_smooth_motion(
                axis_filtered, CONTROL_PARAMS['lookahead'], CONTROL_PARAMS['smoothing'],
                CONTROL_PARAMS['min_speed'], 0,
                CONTROL_PARAMS['holding_speed']
            )
            if need_update or abs(target_pos - servo_states[3].last_target) > 3:
                packetHandler.WritePosEx(3, target_pos, target_speed, CONTROL_PARAMS['acc'])
                servo_states[3].last_target = target_pos
            
            # Servo 4 (Axis 3)
            axis_filtered = apply_deadzone(axes['servo_4'], JOYSTICK_CONFIG['deadzone'])
            target_pos, target_speed, need_update = servo_states[4].calculate_smooth_motion(
                axis_filtered, CONTROL_PARAMS['lookahead'], CONTROL_PARAMS['smoothing'],
                CONTROL_PARAMS['min_speed'], 0,
                CONTROL_PARAMS['holding_speed']
            )
            if need_update or abs(target_pos - servo_states[4].last_target) > 3:
                packetHandler.WritePosEx(4, target_pos, target_speed, CONTROL_PARAMS['acc'])
                servo_states[4].last_target = target_pos
            
            # ✅ Servo 5 BIDIRECTIONAL (LT + RT)
            axis_2_clipped = max(0.0, (axes['servo_5_fwd'] + 1.0) / 2.0)  # LT forward
            axis_5_clipped = max(0.0, (axes['servo_5_bwd'] + 1.0) / 2.0)  # RT backward
            combined = axis_2_clipped - axis_5_clipped  # Bidirectional
            axis_filtered = apply_deadzone(combined, JOYSTICK_CONFIG['deadzone'])
            
            target_pos, target_speed, need_update = servo_states[5].calculate_smooth_motion(
                axis_filtered, CONTROL_PARAMS['lookahead'], CONTROL_PARAMS['smoothing'],
                CONTROL_PARAMS['min_speed'], 0,
                CONTROL_PARAMS['holding_speed']
            )
            if need_update or abs(target_pos - servo_states[5].last_target) > 3:
                packetHandler.WritePosEx(5, target_pos, target_speed, CONTROL_PARAMS['acc'])
                servo_states[5].last_target = target_pos
            
            # === DATA RECORDING ===
            current_time = time.time()
            if data_collector.is_recording and (current_time - last_record_time) >= (1.0 / DATA_CONFIG['fps']):
                # Read all servo data
                joint_pos, joint_vel = servo_reader.read_all()
                
                # Action = last commanded positions
                action = np.array([
                    servo_states[1].last_target,
                    servo_states[2].last_target,
                    servo_states[3].last_target,
                    servo_states[4].last_target,
                    servo_states[5].last_target,
                    gripper_pos
                ], dtype=np.float32)
                
                # Record
                data_collector.record_step(action, joint_pos, joint_vel)
                last_record_time = current_time
            
            clock.tick(LOOP_FREQUENCY)
    
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    
    finally:
        print("\n🛑 Shutting down...")
        
        # ✅ EMERGENCY STOP with holding position
        for servo_id, state in servo_states.items():
            pos, _, _ = packetHandler.ReadPos(servo_id)
            if pos is not None:
                packetHandler.WritePosEx(servo_id, pos, CONTROL_PARAMS['holding_speed'], CONTROL_PARAMS['acc'])
        
        # Lock gripper
        gripper_fb, _, _ = packetHandler.ReadPos(gripper_id)
        if gripper_fb is not None:
            packetHandler.WritePosEx(
                gripper_id, gripper_fb,
                SERVO_CONFIG['limits'][gripper_id]['speed'],
                CONTROL_PARAMS['gripper_acc']
            )
        
        time.sleep(0.2)
        
        # Close connections
        data_collector.close()
        servo_reader.close()
        portHandler.closePort()
        pygame.quit()
        
        # Show stats
        data_collector.get_stats()
        
        print("\n✅ Shutdown complete")

if __name__ == "__main__":
    main()