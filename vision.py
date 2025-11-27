#!/usr/bin/env python3
"""
Vision based Robot Controller for Block Sorting Task.
Platform: Raspberry Pi + Arduino
"""

import cv2
import numpy as np
import serial
import time
import enum
from typing import List, Tuple, Optional
from dataclasses import dataclass

# --- Configuration ---
SERIAL_PORT = '/dev/ttyUSB0'  # Raspberry Pi default usually ttyUSB0 or ttyACM0
BAUD_RATE = 9600
CAMERA_INDEX = 0
RESOLUTION = (640, 480)

# Thresholds
BLOCK_GRAB_AREA_THRESHOLD = 3500      # Area to trigger grab sequence
SHEET_STOP_AREA_THRESHOLD = 25000     # Area to trigger release sequence
CENTER_TOLERANCE = 40                 # Pixels tolerance for centering
SEARCH_TIMEOUT = 20                   # Seconds to search before giving up (optional)

@dataclass
class DetectedObject:
    color: str
    center_x: int
    center_y: int
    area: float
    bbox: Tuple[int, int, int, int]
    aspect_ratio: float

class State(enum.Enum):
    IDLE = 0
    SEARCH_BLOCK = 1
    ALIGN_BLOCK = 2
    APPROACH_BLOCK = 3
    GRAB_BLOCK = 4
    SEARCH_SHEET = 5
    ALIGN_SHEET = 6
    APPROACH_SHEET = 7
    RELEASE_BLOCK = 8
    BACK_OFF = 9

class RobotController:
    def __init__(self, port=SERIAL_PORT, baud=BAUD_RATE):
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # Wait for Arduino reset
            print(f"Connected to {port}")
        except serial.SerialException as e:
            print(f"Error connecting to serial: {e}")
            print("Running in dummy mode (no serial output)")

    def send_command(self, cmd: str):
        """Send command to Arduino"""
        if self.ser and self.ser.is_open:
            try:
                # Arduino expects commands, some are single chars, some strings
                # The Arduino code uses Serial.readStringUntil('\n') in Serialmove()
                # OR single char switch case in UART_Control().
                # Since TESTMODE is false, it uses UART_Control() which reads single chars for movement,
                # BUT checking Arduino code again:
                # It calls UART_Control() in loop.
                # UART_Control reads Serial3 (Bluetooth) OR Serial (USB).
                # For USB Serial:
                # It looks for '(' for servo control.
                # Wait, standard movement chars ('A', 'B'...) are in the switch(BT_Data) block.
                # This block ONLY processes data from Serial3 (Bluetooth) in the original code structure:
                #   if (Serial3.available()) { BT_Data = Serial3.read(); ... }
                #   switch (BT_Data) { ... }
                #
                # HOWEVER, looking closely at UART_Control in Arduino:
                # It reads SERIAL (USB) looking for brackets for servo control.
                # IT DOES NOT seem to pass USB chars to the movement switch statement easily unless we change Arduino code
                # OR we use the Bluetooth port.
                #
                # Let's look at Arduino line 296: It reads USB Serial.
                # Line 342: It reads Serial3 (BT).
                # Line 361: switch(BT_Data).
                #
                # CRITICAL: With current Arduino code, 'A', 'B', 'C' etc only work if sent to Serial3 (Bluetooth)
                # OR if we modify Arduino to accept them from USB Serial.
                #
                # ASSUMPTION: You have the Pi connected via USB.
                # We might need to modify Arduino to process USB characters for movement too.
                #
                # Let's assume for now we need to send to USB and rely on a small fix in Arduino
                # OR we use the `Serialmove` function which is used in TESTMODE?
                # No, TESTMODE is false.
                #
                # Let's fix this by sending the servo control format for movement? No, that controls pan/tilt.
                #
                # RECOMMENDATION: I will modify the send_command to be compatible with what the Arduino LIKELY needs
                # after we fix the Arduino code to read from USB for movement as well.
                #
                # For now, I will write the bytes.
                self.ser.write(cmd.encode('utf-8'))
                # Some commands might need a newline if using readStringUntil, but UART_Control uses single chars.
                # Let's enable flushing.
                self.ser.flush()
            except Exception as e:
                print(f"Serial write error: {e}")
        else:
            print(f"[SIM] CMD: {cmd}")

    def stop(self):
        self.send_command('Z')

    def move_forward(self):
        self.send_command('A')

    def move_backward(self):
        self.send_command('E')

    def turn_left(self):
        self.send_command('C') # rotate_1 (check direction in field)

    def turn_right(self):
        self.send_command('G') # rotate_2 (check direction in field)

    def strafe_left(self):
        self.send_command('d')

    def strafe_right(self):
        self.send_command('b')
        
    def grab_sequence(self):
        # Arduino "go" command triggers: approach -> clip -> rise
        # The Arduino reads this string in Serialmove() usually, but UART_Control is active.
        # UART_Control does NOT seem to handle "go" string.
        # We need to check how to trigger grab in UART_Control.
        # It seems UART_Control ONLY handles single chars A-Z.
        # We need to map 'go' to a char or use the Serialmove logic.
        #
        # Let's use a workaround or fix Arduino code. 
        # Assuming we will add 'P' for Pick and 'R' for Release in Arduino.
        print("Triggering Grab Sequence...")
        # We might need to send a special sequence or modify Arduino.
        # For this code, I will assume 'P' triggers grab (I will add this to Arduino later).
        self.send_command('P') 
        time.sleep(4) # Wait for mechanical action

    def release_sequence(self):
        print("Triggering Release Sequence...")
        # Assuming 'O' for Open/Release
        self.send_command('O')
        time.sleep(2)

class VisionSystem:
    def __init__(self, camera_index=CAMERA_INDEX, resolution=RESOLUTION):
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.width = resolution[0]
        self.height = resolution[1]
        self.center_x = self.width // 2
        
        # HSV Ranges
        self.colors = {
            'red': [
                (np.array([0, 120, 70]), np.array([10, 255, 255])),
                (np.array([170, 120, 70]), np.array([180, 255, 255]))
            ],
            'yellow': [(np.array([20, 100, 100]), np.array([30, 255, 255]))],
            'blue': [(np.array([100, 150, 0]), np.array([140, 255, 255]))]
        }

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret: return None
        return frame

    def detect_largest_object(self, frame, target_color, is_sheet=False) -> Optional[DetectedObject]:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blurred = cv2.GaussianBlur(hsv, (11, 11), 0)
        
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        
        if target_color in self.colors:
            for (lower, upper) in self.colors[target_color]:
                mask = cv2.bitwise_or(mask, cv2.inRange(blurred, lower, upper))
        
        # Clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        largest_obj = None
        max_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # Filter noise
            if area < 500: continue
            
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h
            
            # Filter by shape type
            if is_sheet:
                # Sheet should be vertical (height > width) or large
                if aspect_ratio > 0.9: continue # Skip wide objects
            else:
                # Block should be roughly square
                if aspect_ratio < 0.5 or aspect_ratio > 2.0: continue
            
            if area > max_area:
                max_area = area
                M = cv2.moments(cnt)
                cx = int(M['m10']/M['m00']) if M['m00'] > 0 else x + w//2
                cy = int(M['m01']/M['m00']) if M['m00'] > 0 else y + h//2
                
                largest_obj = DetectedObject(
                    color=target_color,
                    center_x=cx, center_y=cy,
                    area=area, bbox=(x, y, w, h),
                    aspect_ratio=aspect_ratio
                )
                
        return largest_obj

    def cleanup(self):
        self.cap.release()

def main():
    robot = RobotController()
    vision = VisionSystem()
    
    state = State.SEARCH_BLOCK
    target_color_sequence = ['red', 'yellow', 'blue'] # Sequence to find
    current_color_idx = 0
    held_block_color = None
    
    print("Starting Mission...")
    
    try:
        while True:
            frame = vision.get_frame()
            if frame is None: break
            
            annotated_frame = frame.copy()
            
            # Visualization info
            cv2.putText(annotated_frame, f"State: {state.name}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            if state == State.SEARCH_BLOCK:
                target_color = target_color_sequence[current_color_idx]
                obj = vision.detect_largest_object(frame, target_color, is_sheet=False)
                
                if obj:
                    print(f"Found {target_color} block!")
                    state = State.ALIGN_BLOCK
                else:
                    # Rotate to search
                    robot.turn_left()
                    time.sleep(0.1)
                    robot.stop()
            
            elif state == State.ALIGN_BLOCK:
                target_color = target_color_sequence[current_color_idx]
                obj = vision.detect_largest_object(frame, target_color, is_sheet=False)
                
                if not obj:
                    state = State.SEARCH_BLOCK # Lost it
                    continue
                
                # Draw box
                x, y, w, h = obj.bbox
                cv2.rectangle(annotated_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                error = obj.center_x - vision.center_x
                
                if abs(error) > CENTER_TOLERANCE:
                    if error > 0:
                        robot.turn_right()
                        time.sleep(0.05) # Short bursts
                    else:
                        robot.turn_left()
                        time.sleep(0.05)
                    robot.stop()
                else:
                    print("Aligned!")
                    state = State.APPROACH_BLOCK

            elif state == State.APPROACH_BLOCK:
                target_color = target_color_sequence[current_color_idx]
                obj = vision.detect_largest_object(frame, target_color, is_sheet=False)
                
                if not obj:
                    state = State.SEARCH_BLOCK
                    continue
                    
                # Move forward until close enough
                if obj.area < BLOCK_GRAB_AREA_THRESHOLD:
                    robot.move_forward()
                    time.sleep(0.1)
                    robot.stop()
                else:
                    # BLIND FORWARD: Move a bit more to ensure block is in claw
                    print("Blind approach...")
                    robot.move_forward()
                    time.sleep(0.5) # Adjust based on speed
                    robot.stop()
                    state = State.GRAB_BLOCK

            elif state == State.GRAB_BLOCK:
                robot.grab_sequence()
                held_block_color = target_color_sequence[current_color_idx]
                print(f"Grabbed {held_block_color} block")
                state = State.SEARCH_SHEET

            elif state == State.SEARCH_SHEET:
                # Search for sheet of same color as held block
                target_sheet = held_block_color
                obj = vision.detect_largest_object(frame, target_sheet, is_sheet=True)
                
                if obj:
                    print(f"Found {target_sheet} sheet!")
                    state = State.ALIGN_SHEET
                else:
                    robot.turn_left()
                    time.sleep(0.1)
                    robot.stop()

            elif state == State.ALIGN_SHEET:
                target_sheet = held_block_color
                obj = vision.detect_largest_object(frame, target_sheet, is_sheet=True)
                
                if not obj:
                    state = State.SEARCH_SHEET
                    continue
                
                x, y, w, h = obj.bbox
                cv2.rectangle(annotated_frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                
                error = obj.center_x - vision.center_x
                
                if abs(error) > CENTER_TOLERANCE:
                    if error > 0: robot.turn_right()
                    else: robot.turn_left()
                    time.sleep(0.05)
                    robot.stop()
                else:
                    state = State.APPROACH_SHEET

            elif state == State.APPROACH_SHEET:
                target_sheet = held_block_color
                obj = vision.detect_largest_object(frame, target_sheet, is_sheet=True)
                
                if not obj:
                    state = State.SEARCH_SHEET
                    continue
                
                # For sheets, we stop BEFORE hitting them
                if obj.area < SHEET_STOP_AREA_THRESHOLD:
                    robot.move_forward()
                    time.sleep(0.1)
                    robot.stop()
                else:
                    print("Arrived at sheet")
                    robot.stop()
                    state = State.RELEASE_BLOCK

            elif state == State.RELEASE_BLOCK:
                robot.release_sequence()
                print("Block Released")
                
                # Move back a bit
                robot.move_backward()
                time.sleep(1.0)
                robot.stop()
                
                # Switch to next color
                current_color_idx = (current_color_idx + 1) % len(target_color_sequence)
                state = State.SEARCH_BLOCK
            
            cv2.imshow('Robot Vision', annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        robot.stop()
        vision.cleanup()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
