#!/usr/bin/env python
"""
Quick Camera Verification Script
Tests your dual-camera setup before data collection
"""

import cv2
import numpy as np
import time
import threading


class CameraReader:
    """Your optimized camera reader"""
    
    def __init__(self, idx, width=640, height=480):
        self.idx = idx
        self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {idx}")

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)

        # Warm-up
        for _ in range(10):
            self.cap.read()
            time.sleep(0.05)

        self.frame = None
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.lock:
                    self.frame = frame
            time.sleep(0.001)

    def read(self, size):
        with self.lock:
            if self.frame is None:
                return np.zeros((size[1], size[0], 3), dtype=np.uint8)
            return cv2.resize(self.frame, size)

    def stop(self):
        self.running = False
        self.thread.join()
        self.cap.release()


def verify_cameras():
    """Test dual-camera setup"""
    
    print("🎥 Camera Verification Tool")
    print("="*60)
    
    # Configuration
    cam_indices = [2, 4]
    cam_size = (640, 480)
    display_size = (640, 480)
    
    # Try to initialize cameras
    cameras = []
    for idx in cam_indices:
        try:
            print(f"Initializing camera {idx}...", end=" ")
            cam = CameraReader(idx, *cam_size)
            cameras.append(cam)
            print("✅")
        except RuntimeError as e:
            print(f"❌ {e}")
            return False
    
    if len(cameras) != 2:
        print("\n❌ Failed to initialize both cameras!")
        return False
    
    print("\n✅ Both cameras initialized successfully!")
    print("\nPress 'q' to quit")
    print("Press 's' to save test images")
    print("="*60)
    
    # Create windows
    cv2.namedWindow("Camera 0 (Index 2)")
    cv2.namedWindow("Camera 1 (Index 4)")
    
    # FPS tracking
    fps_counter = 0
    fps_start = time.time()
    current_fps = 0
    
    try:
        while True:
            # Read frames
            frame_0 = cameras[0].read(display_size)
            frame_1 = cameras[1].read(display_size)
            
            # Convert RGB to BGR for display
            frame_0_bgr = cv2.cvtColor(frame_0, cv2.COLOR_RGB2BGR)
            frame_1_bgr = cv2.cvtColor(frame_1, cv2.COLOR_RGB2BGR)
            
            # Add FPS overlay
            fps_counter += 1
            if time.time() - fps_start >= 1.0:
                current_fps = fps_counter
                fps_counter = 0
                fps_start = time.time()
            
            cv2.putText(frame_0_bgr, f"FPS: {current_fps}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame_1_bgr, f"FPS: {current_fps}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Display
            cv2.imshow("Camera 0 (Index 2)", frame_0_bgr)
            cv2.imshow("Camera 1 (Index 4)", frame_1_bgr)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("\n✅ Quitting...")
                break
            
            elif key == ord('s'):
                # Save test images
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(f"test_cam0_{timestamp}.jpg", frame_0_bgr)
                cv2.imwrite(f"test_cam1_{timestamp}.jpg", frame_1_bgr)
                print(f"💾 Saved test images: test_cam[0,1]_{timestamp}.jpg")
    
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    
    finally:
        for cam in cameras:
            cam.stop()
        cv2.destroyAllWindows()
        print("✅ Cameras closed")
    
    return True


def list_available_cameras():
    """List all available camera devices"""
    print("\n🔍 Scanning for available cameras...")
    print("="*60)
    
    available = []
    for idx in range(10):
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                h, w = frame.shape[:2]
                print(f"✅ Camera {idx}: {w}x{h}")
                available.append(idx)
            cap.release()
    
    if not available:
        print("❌ No cameras found!")
    else:
        print(f"\n✅ Found {len(available)} camera(s): {available}")
    
    print("="*60)
    return available


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "--scan":
        # Scan mode
        list_available_cameras()
    else:
        # Verification mode
        success = verify_cameras()
        sys.exit(0 if success else 1)