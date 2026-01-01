#!/usr/bin/env python
"""
Real-time Data Collection System for Robot Manipulation
Collects synchronized images, actions, and robot states into Zarr format
"""

import numpy as np
import zarr
import cv2
import time
import threading
from pathlib import Path
from datetime import datetime
from collections import deque
import sys
import os

sys.path.append("..")
from scservo_sdk import *


class DataCollector:
    """
    Efficient data collector for robot manipulation tasks.
    Handles camera, servo feedback, and Zarr storage.
    """
    
    def __init__(self, 
                 dataset_path="./collected_data",
                 camera_id=0,
                 image_size=(96, 96),
                 fps=10,
                 buffer_size=100):
        """
        Args:
            dataset_path: Where to save the Zarr dataset
            camera_id: USB camera ID (0, 1, 2...)
            image_size: Target image resolution (H, W)
            fps: Data collection frequency (Hz)
            buffer_size: Number of samples to buffer before writing
        """
        self.dataset_path = Path(dataset_path)
        self.camera_id = camera_id
        self.image_size = image_size
        self.fps = fps
        self.buffer_size = buffer_size
        
        # Create dataset directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_path = self.dataset_path / f"session_{timestamp}.zarr"
        self.save_path.mkdir(parents=True, exist_ok=True)
        
        # Initialize Zarr storage
        self.store = zarr.DirectoryStore(str(self.save_path))
        self.root = zarr.group(store=self.store, overwrite=True)
        
        # Data groups
        self.data_group = self.root.create_group('data')
        self.meta_group = self.root.create_group('meta')
        
        # Zarr arrays (expandable)
        self.img_array = self.data_group.zeros(
            'img', 
            shape=(0, *image_size, 3), 
            chunks=(10, *image_size, 3),
            dtype=np.float32
        )
        self.action_array = self.data_group.zeros(
            'action',
            shape=(0, 6),
            chunks=(100, 6),
            dtype=np.float32
        )
        self.state_array = self.data_group.zeros(
            'state',
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
        
        # Buffers for efficiency
        self.img_buffer = deque(maxlen=buffer_size)
        self.action_buffer = deque(maxlen=buffer_size)
        self.state_buffer = deque(maxlen=buffer_size)
        self.timestamp_buffer = deque(maxlen=buffer_size)
        
        # Camera setup
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {camera_id}")
        
        # Set camera properties for speed
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce latency
        
        # Threading for camera
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.camera_thread = None
        self.running = False
        
        # Recording state
        self.is_recording = False
        self.last_action = np.zeros(6, dtype=np.float32)
        
        print(f"✅ DataCollector initialized")
        print(f"📁 Save path: {self.save_path}")
        print(f"📷 Camera: {camera_id}")
        print(f"🎯 Image size: {image_size}")
        print(f"⏱️  FPS: {fps}")
    
    
    def _camera_loop(self):
        """Background thread for continuous camera capture"""
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.latest_frame = frame
            time.sleep(1.0 / 60)  # 60 FPS camera polling
    
    
    def start_camera(self):
        """Start camera capture thread"""
        self.running = True
        self.camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self.camera_thread.start()
        print("📷 Camera thread started")
    
    
    def get_frame(self):
        """Get latest camera frame (thread-safe)"""
        with self.frame_lock:
            if self.latest_frame is not None:
                # Resize and normalize
                frame = cv2.resize(self.latest_frame, self.image_size)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = frame.astype(np.float32) / 255.0
                return frame
        return None
    
    
    def start_episode(self):
        """Start a new episode"""
        self.is_recording = True
        self.current_episode += 1
        print(f"\n🎬 Episode {self.current_episode} started (timestep {self.timestep})")
    
    
    def end_episode(self):
        """End current episode and flush buffers"""
        if not self.is_recording:
            return
        
        self.is_recording = False
        self._flush_buffers()
        
        # Record episode end
        self.episode_ends.append(self.timestep)
        
        print(f"✅ Episode {self.current_episode} ended at timestep {self.timestep}")
        print(f"   Duration: {len(self.img_buffer)} samples")
    
    
    def record_step(self, action, state):
        """
        Record one timestep of data.
        
        Args:
            action: (6,) array of servo position commands
            state: (6,) array of current servo positions
        """
        if not self.is_recording:
            return
        
        # Get camera frame
        frame = self.get_frame()
        if frame is None:
            print("⚠️  Frame capture failed, skipping timestep")
            return
        
        # Get timestamp
        timestamp = time.time()
        
        # Add to buffers
        self.img_buffer.append(frame)
        self.action_buffer.append(action)
        self.state_buffer.append(state)
        self.timestamp_buffer.append(timestamp)
        
        self.timestep += 1
        
        # Flush if buffer is full
        if len(self.img_buffer) >= self.buffer_size:
            self._flush_buffers()
    
    
    def _flush_buffers(self):
        """Write buffered data to Zarr storage"""
        if len(self.img_buffer) == 0:
            return
        
        n_samples = len(self.img_buffer)
        
        # Convert buffers to arrays
        imgs = np.array(list(self.img_buffer))
        actions = np.array(list(self.action_buffer))
        states = np.array(list(self.state_buffer))
        timestamps = np.array(list(self.timestamp_buffer))
        
        # Append to Zarr arrays
        current_size = self.img_array.shape[0]
        self.img_array.append(imgs, axis=0)
        self.action_array.append(actions, axis=0)
        self.state_array.append(states, axis=0)
        self.timestamp_array.append(timestamps, axis=0)
        
        # Clear buffers
        self.img_buffer.clear()
        self.action_buffer.clear()
        self.state_buffer.clear()
        self.timestamp_buffer.clear()
        
        print(f"💾 Flushed {n_samples} samples (total: {self.timestep})")
    
    
    def save_metadata(self):
        """Save episode metadata"""
        self.meta_group.array(
            'episode_ends',
            data=np.array(self.episode_ends, dtype=np.int64)
        )
        print(f"📊 Saved metadata: {len(self.episode_ends)} episodes")
    
    
    def close(self):
        """Clean shutdown"""
        print("\n🛑 Closing DataCollector...")
        
        # Stop recording if active
        if self.is_recording:
            self.end_episode()
        
        # Flush any remaining data
        self._flush_buffers()
        
        # Save metadata
        self.save_metadata()
        
        # Stop camera
        self.running = False
        if self.camera_thread:
            self.camera_thread.join(timeout=1.0)
        self.cap.release()
        
        print(f"✅ Saved {self.timestep} total timesteps")
        print(f"✅ Dataset saved to: {self.save_path}")
    
    
    def get_stats(self):
        """Print dataset statistics"""
        print("\n" + "="*60)
        print("📊 DATASET STATISTICS")
        print("="*60)
        print(f"Total timesteps: {self.timestep}")
        print(f"Total episodes: {len(self.episode_ends)}")
        if len(self.episode_ends) > 0:
            avg_length = self.timestep / len(self.episode_ends)
            print(f"Average episode length: {avg_length:.1f}")
        print(f"Image shape: {self.img_array.shape}")
        print(f"Action shape: {self.action_array.shape}")
        print(f"State shape: {self.state_array.shape}")
        print("="*60)


# ==============================================================================
# SERVO COMMUNICATION HELPER
# ==============================================================================
class ServoReader:
    """Helper class to read servo positions efficiently"""
    
    SERVO_IDS = [1, 2, 3, 4, 5, 6]  # All servo IDs
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.portHandler = PortHandler(port)
        self.packetHandler = sms_sts(self.portHandler)
        
        if not self.portHandler.openPort() or not self.portHandler.setBaudRate(baudrate):
            raise RuntimeError(f"Failed to open port {port}")
        
        print(f"✅ Servo communication initialized on {port}")
    
    
    def read_all_positions(self):
        """
        Read positions from all 6 servos.
        Returns (6,) numpy array
        """
        positions = []
        for servo_id in self.SERVO_IDS:
            pos, result, error = self.packetHandler.ReadPos(servo_id)
            if result == COMM_SUCCESS:
                positions.append(pos)
            else:
                positions.append(0)  # Fallback
        
        return np.array(positions, dtype=np.float32)
    
    
    def close(self):
        """Close serial port"""
        self.portHandler.closePort()


# ==============================================================================
# EXAMPLE USAGE
# ==============================================================================
if __name__ == "__main__":
    print("🤖 Data Collection System Test")
    print("Press Ctrl+C to stop\n")
    
    # Initialize collector
    collector = DataCollector(
        dataset_path="./test_dataset",
        camera_id=2,
        image_size=(96, 96),
        fps=10
    )
    
    # Initialize servo reader
    servo_reader = ServoReader(port='/dev/ttyUSB0')
    
    # Start camera
    collector.start_camera()
    time.sleep(1.0)  # Wait for camera warmup
    
    try:
        # Simulate 2 episodes
        for episode in range(2):
            collector.start_episode()
            
            # Collect 50 samples per episode
            for step in range(50):
                # Read servo positions
                state = servo_reader.read_all_positions()
                
                # Action = state (for testing)
                action = state.copy()
                
                # Record
                collector.record_step(action, state)
                
                # Print progress
                if step % 10 == 0:
                    print(f"  Step {step}/50")
                
                time.sleep(1.0 / collector.fps)
            
            collector.end_episode()
            
            # Wait between episodes
            if episode < 1:
                print("⏸️  5 second break...")
                time.sleep(5.0)
        
        # Show stats
        collector.get_stats()
    
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    
    finally:
        collector.close()
        servo_reader.close()
        print("✅ Test complete")