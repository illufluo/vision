#!/usr/bin/env python3
"""
Robot Controller with Vision
Integrates OpenCV vision with Arduino serial control for a Pick-and-Place task.
"""

import cv2
import numpy as np
import serial
import time
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass

# --- Serial Configuration ---
# Update this to match your Raspberry Pi setup (e.g., '/dev/ttyUSB0' or '/dev/ttyACM0')
SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 9600

@dataclass
class DetectedObject:
    """Data class for detected objects (blocks or sheets)"""
    color: str
    center_x: int
    center_y: int
    area: float
    bbox: Tuple[int, int, int, int]  # (x, y, width, height)
    aspect_ratio: float

class VisionSystem:
    """
    Vision system for detecting small colored blocks and vertical A4 sheets.
    """
    
    def __init__(self, camera_index=0, resolution=(640, 480)):
        self.camera_index = camera_index
        self.resolution = resolution
        self.camera = None
        
        # HSV color ranges
        self.color_ranges = {
            'red': [
                (np.array([0, 100, 100]), np.array([10, 255, 255])),
                (np.array([160, 100, 100]), np.array([180, 255, 255]))
            ],
            'yellow': [
                (np.array([20, 100, 100]), np.array([35, 255, 255]))
            ],
            'blue': [
                (np.array([100, 100, 80]), np.array([130, 255, 255]))
            ],
            'black': [
                (np.array([0, 0, 0]), np.array([180, 255, 50]))
            ]
        }
        
        # Detection parameters
        self.block_min_area = 300
        self.block_max_area = 6000  # Slightly increased
        self.sheet_min_area = 8000
        self.sheet_max_area = 150000
        
        self.block_aspect_ratio_range = (0.5, 2.0)
        self.sheet_aspect_ratio_range = (0.3, 1.2) # Slightly relaxed
        
        self.kernel = np.ones((5, 5), np.uint8)
        self.frame_center_x = resolution[0] // 2
        self.frame_center_y = resolution[1] // 2
        
        self.setup_camera()
    
    def setup_camera(self):
        self.camera = cv2.VideoCapture(self.camera_index)
        if not self.camera.isOpened():
            print(f"Warning: Failed to open camera {self.camera_index}")
            return
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        for _ in range(5): self.camera.read()
    
    def capture_frame(self) -> Optional[np.ndarray]:
        if not self.camera: return None
        ret, frame = self.camera.read()
        return frame if ret else None
    
    def preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        return cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    def create_color_mask(self, hsv_frame: np.ndarray, color: str) -> np.ndarray:
        if color not in self.color_ranges: return np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
        mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
        for lower, upper in self.color_ranges[color]:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv_frame, lower, upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
    
    def extract_objects(self, mask: np.ndarray, color: str, min_area, max_area, ar_range) -> List[DetectedObject]:
        objects = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if not (min_area < area < max_area): continue
            x, y, w, h = cv2.boundingRect(contour)
            ar = w / h if h > 0 else 0
            if not (ar_range[0] < ar < ar_range[1]): continue
            
            M = cv2.moments(contour)
            cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else x + w // 2
            cy = int(M["m01"] / M["m00"]) if M["m00"] != 0 else y + h // 2
            
            objects.append(DetectedObject(color, cx, cy, area, (x, y, w, h), ar))
        
        objects.sort(key=lambda o: o.area, reverse=True)
        return objects

    def detect_blocks(self, frame, colors=['red', 'yellow', 'blue']):
        hsv = self.preprocess_frame(frame)
        all_objs = []
        for c in colors:
            mask = self.create_color_mask(hsv, c)
            all_objs.extend(self.extract_objects(mask, c, self.block_min_area, self.block_max_area, self.block_aspect_ratio_range))
        all_objs.sort(key=lambda o: o.area, reverse=True)
        return all_objs

    def detect_sheets(self, frame, target_color):
        hsv = self.preprocess_frame(frame)
        mask = self.create_color_mask(hsv, target_color)
        return self.extract_objects(mask, target_color, self.sheet_min_area, self.sheet_max_area, self.sheet_aspect_ratio_range)

    def cleanup(self):
        if self.camera: self.camera.release()

class RobotController:
    def __init__(self):
        self.vision = VisionSystem()
        self.ser = None
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2) # Wait for Arduino reset
            print(f"Serial connected on {SERIAL_PORT}")
        except Exception as e:
            print(f"Serial Error: {e}")
            print("Running in VISION ONLY mode (no robot motion)")

        self.target_color = None
        self.state = "SEARCH_BLOCK"
        self.center_tolerance = 40
        
        # Tweak these areas based on real world camera distance
        self.stop_area_block = 3000 
        self.stop_area_sheet = 25000 

    def send(self, cmd: str):
        """Send command to Arduino ending with newline"""
        if self.ser and self.ser.is_open:
            print(f"CMD: {cmd}")
            self.ser.write(f"{cmd}\n".encode('utf-8'))
            # self.ser.flush() 

    def move_and_stop(self, cmd, duration):
        """Move for a duration then stop"""
        self.send(cmd)
        time.sleep(duration)
        self.send("S")

    def run(self):
        print("Starting Robot Loop...")
        try:
            while True:
                frame = self.vision.capture_frame()
                if frame is None: continue

                # Visualization
                display_frame = frame.copy()
                cv2.line(display_frame, (320, 0), (320, 480), (0, 255, 0), 1)
                cv2.putText(display_frame, f"State: {self.state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                if self.state == "SEARCH_BLOCK":
                    blocks = self.vision.detect_blocks(frame)
                    
                    # Draw blocks
                    for b in blocks:
                        cv2.rectangle(display_frame, (b.bbox[0], b.bbox[1]), (b.bbox[0]+b.bbox[2], b.bbox[1]+b.bbox[3]), (0,255,0), 2)

                    if blocks:
                        best_block = blocks[0]
                        self.target_color = best_block.color
                        print(f"Found {self.target_color} block! Approach.")
                        self.state = "APPROACH_BLOCK"
                    else:
                        # Spin slowly to find
                        self.move_and_stop("rC", 0.1)
                        time.sleep(0.1) # Wait for image to stabilize

                elif self.state == "APPROACH_BLOCK":
                    # Track the SPECIFIC color we found
                    blocks = self.vision.detect_blocks(frame, [self.target_color])
                    
                    if not blocks:
                        print("Lost block, searching...")
                        self.state = "SEARCH_BLOCK"
                        continue
                    
                    target = blocks[0]
                    error = target.center_x - 320
                    
                    cv2.circle(display_frame, (target.center_x, target.center_y), 5, (0, 0, 255), -1)

                    # 1. Align (Rotation)
                    if abs(error) > self.center_tolerance:
                        if error > 0:
                            self.move_and_stop("rC", 0.05) # Rotate Right/CW
                        else:
                            self.move_and_stop("rA", 0.05) # Rotate Left/CCW
                    
                    # 2. Distance check
                    elif target.area < self.stop_area_block:
                        # Forward
                        self.move_and_stop("A", 0.2)
                    
                    else:
                        # Centered and Close enough
                        print("Block aligned. Performing blind approach...")
                        # Overkill forward to ensure it's within gripper reach
                        self.move_and_stop("A", 1.0) 
                        self.send("S") # Force stop
                        time.sleep(0.5)
                        self.state = "GRAB"

                elif self.state == "GRAB":
                    print("Grabbing...")
                    self.send("go") # Triggers approach->clip->rise
                    # "go" takes about 4-5 seconds on Arduino
                    time.sleep(6) 
                    print("Grab complete.")
                    self.state = "SEARCH_SHEET"

                elif self.state == "SEARCH_SHEET":
                    if self.target_color is None: 
                         self.state = "SEARCH_BLOCK"
                         continue

                    cv2.putText(display_frame, f"Target: {self.target_color} Sheet", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    
                    sheets = self.vision.detect_sheets(frame, self.target_color)
                    
                    if sheets:
                        print(f"Found {self.target_color} sheet.")
                        self.state = "APPROACH_SHEET"
                    else:
                        # Spin to find wall
                        self.move_and_stop("rC", 0.1)
                        time.sleep(0.1)

                elif self.state == "APPROACH_SHEET":
                    sheets = self.vision.detect_sheets(frame, self.target_color)
                    
                    if not sheets:
                        print("Lost sheet...")
                        self.state = "SEARCH_SHEET"
                        continue

                    target = sheets[0]
                    error = target.center_x - 320
                    
                    cv2.rectangle(display_frame, (target.bbox[0], target.bbox[1]), (target.bbox[0]+target.bbox[2], target.bbox[1]+target.bbox[3]), (255,0,0), 2)

                    if abs(error) > self.center_tolerance:
                        if error > 0:
                            self.move_and_stop("rC", 0.05)
                        else:
                            self.move_and_stop("rA", 0.05)
                    elif target.area < self.stop_area_sheet:
                         self.move_and_stop("A", 0.2)
                    else:
                        print("Arrived at sheet.")
                        self.send("S")
                        time.sleep(0.5)
                        self.state = "RELEASE"

                elif self.state == "RELEASE":
                    print("Releasing...")
                    self.send("rel")
                    time.sleep(2)
                    
                    print("Backing off...")
                    self.move_and_stop("B", 1.5)
                    
                    self.target_color = None
                    self.state = "SEARCH_BLOCK"


                cv2.imshow("Robot Vision", display_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            self.send("S")
            self.vision.cleanup()
            if self.ser: self.ser.close()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    bot = RobotController()
    bot.run()
