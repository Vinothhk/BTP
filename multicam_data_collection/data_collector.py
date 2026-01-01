#!/usr/bin/env python
"""
Multi-Camera Data Collection System for Robot Manipulation
Efficiently collects:
- 2 camera views (RGB images)
- Joint positions (6D)
- Joint velocities (6D)
- Actions (6D)
- Timestamps

Writes to Zarr format for fast I/O and compatibility with diffusion policy training.
"""

import cv2
import threading
import numpy as np
import time
import zarr
from pathlib import Path
from datetime import datetime
from collections import deque
import sys
import os

sys.path.append("..")
from scservo_sdk import *


# ==============================================================================
# CAMERA READER (Your optimized version)
# ==============================================================================
class CameraReader:
    """Optimized camera reader with background thread"""
    
    def __init__(self, idx, width=640, height=480):
        self.idx = idx

        self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {idx}")

        # 🔥 CRITICAL SETTINGS
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)

        # Warm-up (VERY IMPORTANT)
        for _ in range(10):
            self.cap.read()
            time.sleep(0.05)

        self.frame = None
        self.running = True
        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        
        print(f"✅ Camera {idx} initialized")

    def _loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.lock:
                    self.frame = frame
            time.sleep(0.001)

    def read(self, size):
        """
        Read and resize frame.
        Returns RGB image (not BGR!)
        """
        with self.lock:
            if self.frame is None:
                return np.zeros((size[1], size[0], 3), dtype=np.uint8)
            # Convert BGR to RGB and resize
            frame_rgb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            return cv2.resize(frame_rgb, size)

    def stop(self):
        self.running = False
        self.thread.join()
        self.cap.release()


# ==============================================================================
# SERVO COMMUNICATION (Efficient batch reading)
# ==============================================================================
class ServoReader:
    """Efficient servo position and velocity reader"""
    
    SERVO_IDS = [1, 2, 3, 4, 5, 6]  # All 6 servos
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, packet_handler=None):
        """
        Args:
            port: Serial port path (used only if packet_handler is None)
            baudrate: Baud rate (used only if packet_handler is None)
            packet_handler: Existing packet handler to reuse (recommended)
        """
        if packet_handler is not None:
            # ✅ CRITICAL FIX: Reuse existing connection
            self.packetHandler = packet_handler
            self.portHandler = None  # We don't own this
            self.own_port = False
            print(f"✅ ServoReader using shared packet handler")
        else:
            # Fallback: Create new connection
            self.portHandler = PortHandler(port)
            self.packetHandler = sms_sts(self.portHandler)
            
            if not self.portHandler.openPort() or not self.portHandler.setBaudRate(baudrate):
                raise RuntimeError(f"Failed to open port {port}")
            
            self.own_port = True
            
            # Ping all servos
            for servo_id in self.SERVO_IDS:
                model, result, error = self.packetHandler.ping(servo_id)
                if result != COMM_SUCCESS:
                    raise RuntimeError(f"Cannot find servo ID {servo_id}")
            
            print(f"✅ All {len(self.SERVO_IDS)} servos connected on {port}")
    
    def read_positions(self):
        """
        Read positions from all 6 servos.
        Returns: (6,) numpy array
        """
        positions = []
        for servo_id in self.SERVO_IDS:
            pos, result, error = self.packetHandler.ReadPos(servo_id)
            if result == COMM_SUCCESS:
                positions.append(pos)
            else:
                positions.append(0)  # Fallback
        return np.array(positions, dtype=np.float32)
    
    def read_velocities(self):
        """
        Read velocities from all 6 servos.
        Returns: (6,) numpy array
        """
        velocities = []
        for servo_id in self.SERVO_IDS:
            vel, result, error = self.packetHandler.ReadSpeed(servo_id)
            if result == COMM_SUCCESS:
                velocities.append(vel)
            else:
                velocities.append(0)
        return np.array(velocities, dtype=np.float32)
    
    def read_all(self):
        """
        Read both positions and velocities.
        Returns: (positions, velocities) both (6,) arrays
        """
        return self.read_positions(), self.read_velocities()
    
    def close(self):
        """Close port only if we own it"""
        if self.own_port and self.portHandler is not None:
            self.portHandler.closePort()
            print("✅ ServoReader closed own port")


# ==============================================================================
# MULTI-CAMERA DATA COLLECTOR
# ==============================================================================
class MultiCameraDataCollector:
    """
    Efficient data collector for dual-camera robot manipulation.
    
    Data structure:
    - cam_0: (N, H, W, 3) float32 [0-1]
    - cam_1: (N, H, W, 3) float32 [0-1]
    - joint_pos: (N, 6) float32
    - joint_vel: (N, 6) float32
    - action: (N, 6) float32
    - timestamp: (N,) float64
    """
    
    def __init__(self,
                 dataset_path="./collected_data",
                 cam_indices=[2, 4],
                 cam_size=(640, 480),
                 target_size=(96, 96),
                 fps=10,
                 buffer_size=50):
        """
        Args:
            dataset_path: Root directory for datasets
            cam_indices: List of camera indices (e.g., [2, 4])
            cam_size: Camera capture resolution (W, H)
            target_size: Target image size for storage (W, H)
            fps: Data collection frequency (Hz)
            buffer_size: Samples to buffer before writing to disk
        """
        self.dataset_path = Path(dataset_path)
        self.cam_indices = cam_indices
        self.cam_size = cam_size
        self.target_size = target_size
        self.fps = fps
        self.buffer_size = buffer_size
        
        # Create timestamped dataset directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_path = self.dataset_path / f"session_{timestamp}.zarr"
        self.save_path.mkdir(parents=True, exist_ok=True)
        
        # Initialize Zarr storage
        self.store = zarr.DirectoryStore(str(self.save_path))
        self.root = zarr.group(store=self.store, overwrite=True)
        
        # Create data and meta groups
        self.data_group = self.root.create_group('data')
        self.meta_group = self.root.create_group('meta')
        
        # Zarr arrays (expandable with chunking)
        H, W = target_size[1], target_size[0]
        
        self.cam_0_array = self.data_group.zeros(
            'cam_0',
            shape=(0, H, W, 3),
            chunks=(100, H, W, 3),
            dtype=np.float32
        )
        
        self.cam_1_array = self.data_group.zeros(
            'cam_1',
            shape=(0, H, W, 3),
            chunks=(100, H, W, 3),
            dtype=np.float32
        )
        
        self.joint_pos_array = self.data_group.zeros(
            'joint_pos',
            shape=(0, 6),
            chunks=(100, 6),
            dtype=np.float32
        )
        
        self.joint_vel_array = self.data_group.zeros(
            'joint_vel',
            shape=(0, 6),
            chunks=(100, 6),
            dtype=np.float32
        )
        
        self.action_array = self.data_group.zeros(
            'action',
            shape=(0, 6),
            chunks=(100, 6),
            dtype=np.float32
        )
        
        self.timestamp_array = self.data_group.zeros(
            'timestamp',
            shape=(0,),
            chunks=(100,),
            dtype=np.float64
        )
        
        # Episode tracking
        self.episode_ends = []
        self.current_episode = 0
        self.timestep = 0
        
        # Buffers for batched writing
        self.cam_0_buffer = deque(maxlen=buffer_size)
        self.cam_1_buffer = deque(maxlen=buffer_size)
        self.joint_pos_buffer = deque(maxlen=buffer_size)
        self.joint_vel_buffer = deque(maxlen=buffer_size)
        self.action_buffer = deque(maxlen=buffer_size)
        self.timestamp_buffer = deque(maxlen=buffer_size)
        
        # Recording state
        self.is_recording = False
        
        # Initialize cameras
        print(f"\n📷 Initializing {len(cam_indices)} cameras...")
        self.cameras = [CameraReader(idx, *cam_size) for idx in cam_indices]
        time.sleep(0.5)  # Camera warmup
        
        print(f"\n✅ MultiCameraDataCollector initialized")
        print(f"📁 Save path: {self.save_path}")
        print(f"📷 Cameras: {cam_indices}")
        print(f"🎯 Target size: {target_size}")
        print(f"⏱️  FPS: {fps}")
        print(f"💾 Buffer size: {buffer_size}")
    
    def start_episode(self):
        """Start recording a new episode"""
        self.is_recording = True
        self.current_episode += 1
        print(f"\n🎬 Episode {self.current_episode} STARTED (timestep {self.timestep})")
    
    def end_episode(self):
        """End current episode and flush buffers"""
        if not self.is_recording:
            return
        
        self.is_recording = False
        self._flush_buffers()
        
        # Record episode end index
        self.episode_ends.append(self.timestep)
        
        episode_length = len(self.cam_0_buffer) if self.cam_0_buffer else 0
        print(f"✅ Episode {self.current_episode} ENDED at timestep {self.timestep}")
        print(f"   Episode length: {episode_length} samples")
    
    def record_step(self, action, joint_pos, joint_vel):
        """
        Record one timestep of data.
        
        Args:
            action: (6,) array of commanded joint positions
            joint_pos: (6,) array of current joint positions
            joint_vel: (6,) array of current joint velocities
        """
        if not self.is_recording:
            return
        
        # ✅ DEBUG: Check for zero data
        if np.all(joint_pos == 0) or np.all(joint_vel == 0):
            print(f"⚠️  WARNING: Zero joint data detected at timestep {self.timestep}")
            print(f"   joint_pos: {joint_pos}")
            print(f"   joint_vel: {joint_vel}")
        
        # Capture camera frames
        frame_0 = self.cameras[0].read(self.target_size)
        frame_1 = self.cameras[1].read(self.target_size)
        
        # Normalize to [0, 1]
        frame_0 = frame_0.astype(np.float32) / 255.0
        frame_1 = frame_1.astype(np.float32) / 255.0
        
        # Get timestamp
        timestamp = time.time()
        
        # Add to buffers
        self.cam_0_buffer.append(frame_0)
        self.cam_1_buffer.append(frame_1)
        self.joint_pos_buffer.append(joint_pos)
        self.joint_vel_buffer.append(joint_vel)
        self.action_buffer.append(action)
        self.timestamp_buffer.append(timestamp)
        
        self.timestep += 1
        
        # Auto-flush if buffer is full
        if len(self.cam_0_buffer) >= self.buffer_size:
            self._flush_buffers()
    
    def _flush_buffers(self):
        """Write buffered data to Zarr storage"""
        if len(self.cam_0_buffer) == 0:
            return
        
        n_samples = len(self.cam_0_buffer)
        
        # Convert buffers to numpy arrays
        cam_0_batch = np.array(list(self.cam_0_buffer))
        cam_1_batch = np.array(list(self.cam_1_buffer))
        joint_pos_batch = np.array(list(self.joint_pos_buffer))
        joint_vel_batch = np.array(list(self.joint_vel_buffer))
        action_batch = np.array(list(self.action_buffer))
        timestamp_batch = np.array(list(self.timestamp_buffer))
        
        # Append to Zarr arrays
        self.cam_0_array.append(cam_0_batch, axis=0)
        self.cam_1_array.append(cam_1_batch, axis=0)
        self.joint_pos_array.append(joint_pos_batch, axis=0)
        self.joint_vel_array.append(joint_vel_batch, axis=0)
        self.action_array.append(action_batch, axis=0)
        self.timestamp_array.append(timestamp_batch, axis=0)
        
        # Clear buffers
        self.cam_0_buffer.clear()
        self.cam_1_buffer.clear()
        self.joint_pos_buffer.clear()
        self.joint_vel_buffer.clear()
        self.action_buffer.clear()
        self.timestamp_buffer.clear()
        
        print(f"💾 Flushed {n_samples} samples (total: {self.timestep})")
    
    def save_metadata(self):
        """Save episode metadata to Zarr"""
        self.meta_group.array(
            'episode_ends',
            data=np.array(self.episode_ends, dtype=np.int64)
        )
        print(f"📊 Saved metadata: {len(self.episode_ends)} episodes")
    
    def close(self):
        """Clean shutdown and save"""
        print("\n🛑 Closing MultiCameraDataCollector...")
        
        # End recording if active
        if self.is_recording:
            self.end_episode()
        
        # Flush remaining data
        self._flush_buffers()
        
        # Save metadata
        self.save_metadata()
        
        # Stop cameras
        for cam in self.cameras:
            cam.stop()
        
        print(f"✅ Saved {self.timestep} total timesteps")
        print(f"✅ Dataset saved to: {self.save_path}")
    
    def get_stats(self):
        """Print dataset statistics"""
        print("\n" + "="*70)
        print("📊 DATASET STATISTICS")
        print("="*70)
        print(f"Total episodes: {len(self.episode_ends)}")
        print(f"Total timesteps: {self.timestep}")
        if len(self.episode_ends) > 0:
            avg_length = self.timestep / len(self.episode_ends)
            print(f"Average episode length: {avg_length:.1f}")
        print(f"\nData shapes:")
        print(f"  Camera 0:    {self.cam_0_array.shape}")
        print(f"  Camera 1:    {self.cam_1_array.shape}")
        print(f"  Joint Pos:   {self.joint_pos_array.shape}")
        print(f"  Joint Vel:   {self.joint_vel_array.shape}")
        print(f"  Actions:     {self.action_array.shape}")
        print(f"  Timestamps:  {self.timestamp_array.shape}")
        print("="*70)


# ==============================================================================
# EXAMPLE USAGE / TEST
# ==============================================================================
if __name__ == "__main__":
    print("🤖 Multi-Camera Data Collection System Test")
    print("Press Ctrl+C to stop\n")
    
    # Configuration
    CAM_INDICES = [2, 4]
    CAM_SIZE = (640, 480)
    TARGET_SIZE = (96, 96)
    FPS = 10
    
    # Initialize collector
    collector = MultiCameraDataCollector(
        dataset_path="./test_dataset",
        cam_indices=CAM_INDICES,
        cam_size=CAM_SIZE,
        target_size=TARGET_SIZE,
        fps=FPS,
        buffer_size=50
    )
    
    # Initialize servo reader
    try:
        servo_reader = ServoReader(port='/dev/ttyUSB0')
    except RuntimeError as e:
        print(f"⚠️  Servo init failed: {e}")
        print("   Running in CAMERA-ONLY mode")
        servo_reader = None
    
    try:
        # Simulate 2 episodes
        for episode in range(2):
            collector.start_episode()
            
            # Collect 30 samples per episode
            for step in range(30):
                # Read servo data (if available)
                if servo_reader:
                    joint_pos, joint_vel = servo_reader.read_all()
                    action = joint_pos.copy()  # Action = current pos (for testing)
                else:
                    # Dummy data if no servos
                    joint_pos = np.random.rand(6).astype(np.float32) * 1000
                    joint_vel = np.random.randn(6).astype(np.float32) * 10
                    action = joint_pos.copy()
                
                # Record timestep
                collector.record_step(action, joint_pos, joint_vel)
                
                # Print progress
                if step % 10 == 0:
                    print(f"  Step {step}/30")
                
                # Respect FPS
                time.sleep(1.0 / FPS)
            
            collector.end_episode()
            
            # Break between episodes
            if episode < 1:
                print("\n⏸️  5 second break...")
                time.sleep(5.0)
        
        # Show final stats
        collector.get_stats()
    
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    
    finally:
        collector.close()
        if servo_reader:
            servo_reader.close()
        print("✅ Test complete")
