#!/usr/bin/env python3
"""
Soccer Robot Controller - Complete Rewrite
Robot head is mounted on CREATE3 back, so all directions are inverted
Left arm = carrying/dribbling arm
Right arm = shooting arm
"""

import cv2
import numpy as np
import socket
import time
import json
import os
import math
from dt_apriltags import Detector  # AprilTag detection library

class SoccerRobot:
    def __init__(self, robot_ip, robot_port=5000):
        """Initialize soccer robot with inverted controls for rear-mounted camera"""
        
        # ==========================================
        # NETWORK SETUP
        # ==========================================
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.sock = None
        self.connect_to_robot()
        
        # ==========================================
        # CAMERA SETUP
        # ==========================================
        self.cap = self._initialize_camera()
        if not self.cap or not self.cap.isOpened():
            print("ERROR: Camera not available. Exiting.")
            self.cleanup()
            exit()
        
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.center_x = self.frame_width // 2
        self.center_y = self.frame_height // 2
        
        # ==========================================
        # COLOR CALIBRATION
        # ==========================================
        self.ball_hsv_lower = None
        self.ball_hsv_upper = None
        self.goal_hsv_lower = None
        self.goal_hsv_upper = None
        self.calibration_file = "soccer_calibration.json"
        self.CALIBRATION_SAMPLES = 200  # Back to 200 samples
        self.CALIBRATION_DURATION = 5.0  # 5 seconds duration
        
        # ==========================================
        # POSSESSION DETECTION
        # ==========================================
        # Ball is in possession when:
        # 1. It crosses the possession line, OR
        # 2. It disappears off the bottom of the screen (y > frame_height)
        self.POSSESSION_LINE_OFFSET = 100  # pixels from bottom (increased for earlier detection)
        self.possession_line_y = self.frame_height - self.POSSESSION_LINE_OFFSET
        self.POSSESSION_SETTLE_SPEED = 0.08  # slow creep to secure ball
        self.POSSESSION_SETTLE_TIME = 0.7    # seconds to settle
        
        # Track if we recently had the ball (for when it disappears)
        self.last_ball_y = None
        self.ball_approaching_bottom = False
        
        # ==========================================
        # BALL TRACKING
        # ==========================================
        self.Kp_angular = 0.5   # proportional gain for turning to ball
        self.Kp_forward = 0.3   # proportional gain for moving to ball
        self.MIN_BALL_RADIUS = 8
        self.MIN_BALL_CIRCULARITY = 0.6
        
        # ==========================================
        # APRILTAG DETECTION FOR HOME MARKER
        # ==========================================
        # AprilTag marks the "home" shooting position
        self.apriltag_detector = Detector(families='tag36h11')
        self.HOME_TAG_ID = 0  # AprilTag ID for home position (change if needed)
        self.home_position = None  # (x, y) in frame when home tag is detected
        self.HOME_ALIGNMENT_TOLERANCE = 50  # pixels - how close to home center
        
        # ==========================================
        # GOAL ALIGNMENT & SHOOTING
        # ==========================================
        self.GOAL_CENTER_TOLERANCE = 30  # pixels - how centered goal must be
        
        # ==========================================
        # TUNABLE SHOOTING PARAMETERS - BACKUP & SPIN RIGHT
        # ==========================================
        # Simple shooting: backup then spin right fast to strike with right arm
        self.SHOOT_BACKUP_SPEED = 0.5       # speed to backup before strike
        self.SHOOT_BACKUP_DISTANCE = 1.0    # how long to backup (seconds)
        self.SHOOT_SPIN_SPEED = 1.0         # max speed spinning right (clockwise)
        self.SHOOT_SPIN_DURATION = 3.0      # how long to spin right (seconds)
        
        # Easy tuning guide:
        # - Increase SHOOT_BACKUP_DISTANCE for more windup space
        # - Increase SHOOT_BACKUP_SPEED for faster backup
        # - Increase SHOOT_SPIN_DURATION for longer/more powerful strike
        # - SHOOT_SPIN_SPEED is already at max (1.0)
        
        # ==========================================
        # STATE MACHINE
        # ==========================================
        # States: IDLE, SEEKING_BALL, SECURING_BALL, GOAL_SEARCH, ALIGNING_GOAL, 
        #         MOVING_TO_HOME, ALIGNING_HOME, SHOOTING
        self.state = "IDLE"
        self.running = False
        
        # ==========================================
        # DEBUG/DISPLAY
        # ==========================================
        self.debug_message = "System initialized"
        
        # ==========================================
        # CALIBRATION UI STATE
        # ==========================================
        self._calib_sampling = False
        self._calib_samples = []
        self._calib_start_time = None
        self._calib_cursor_pos = (self.center_x, self.center_y)
        
        print("=" * 60)
        print("SOCCER ROBOT INITIALIZED")
        print(f"Camera: {self.frame_width}x{self.frame_height}")
        print(f"Robot IP: {self.robot_ip}:{self.robot_port}")
        print("NOTE: Camera on robot BACK - all directions inverted")
        print("=" * 60)

    # ==========================================
    # CAMERA INITIALIZATION
    # ==========================================
    def _initialize_camera(self):
        """Try to open camera from multiple indices"""
        for index in [1, 0, 2]:
            cap = cv2.VideoCapture(index)
            time.sleep(0.3)
            if cap.isOpened():
                print(f"Camera opened at index {index}")
                return cap
            cap.release()
        return None

    # ==========================================
    # NETWORK FUNCTIONS
    # ==========================================
    def connect_to_robot(self):
        """Establish TCP connection to robot"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(3)
            self.sock.connect((self.robot_ip, self.robot_port))
            self.sock.settimeout(None)
            print(f"✓ Connected to robot at {self.robot_ip}:{self.robot_port}")
        except Exception as e:
            print(f"✗ Could not connect to robot: {e}")
            self.sock = None

    def send_command(self, command):
        """Send command string to robot"""
        if not self.sock:
            self.connect_to_robot()
            if not self.sock:
                return False
        try:
            self.sock.sendall((command + "\n").encode())
            return True
        except Exception as e:
            print(f"Send error: {e}")
            self.sock = None
            return False
    
    def send_navigate(self, x_cm, y_cm, heading_deg=None, tag=""):
        """
        Send navigation command to robot
        x_cm, y_cm: target position in centimeters
        heading_deg: optional final heading in degrees
        """
        if heading_deg is not None:
            cmd = f"NAVIGATE:{x_cm}:{y_cm}:{heading_deg}"
        else:
            cmd = f"NAVIGATE:{x_cm}:{y_cm}"
        if tag:
            print(f"[{tag}] {cmd}")
        self.send_command(cmd)
    
    def send_turn(self, degrees, tag=""):
        """
        Send turn command to robot
        degrees: positive = left (counter-clockwise), negative = right (clockwise)
        """
        cmd = f"TURN:{degrees}"
        if tag:
            print(f"[{tag}] Turn {degrees}°")
        self.send_command(cmd)
    
    def reset_navigation(self, tag=""):
        """Reset robot's navigation origin"""
        cmd = "RESET_NAV"
        if tag:
            print(f"[{tag}] Resetting navigation")
        self.send_command(cmd)

    def send_velocity(self, linear, angular, tag=""):
        """
        Send velocity command to robot
        NOTE: Camera is on BACK of robot, so directions are INVERTED
        - Positive linear = robot moves BACKWARD (camera moves forward to ball)
        - Negative angular = robot turns LEFT (camera turns right)
        - Positive angular = robot turns RIGHT (camera turns left)
        """
        # Invert for rear-mounted camera
        robot_linear = -linear   # flip forward/back
        robot_angular = -angular # flip left/right
        
        # Clamp values
        robot_linear = max(min(robot_linear, 1.0), -1.0)
        robot_angular = max(min(robot_angular, 1.0), -1.0)
        
        cmd = f"VELOCITY:{robot_linear:.2f}:{robot_angular:.2f}"
        if tag:
            print(f"[{tag}] Camera: lin={linear:.2f} ang={angular:.2f} → Robot: {cmd}")
        self.send_command(cmd)

    def stop(self):
        """Stop robot movement"""
        self.send_command("STOP")
        print("[STOP] Robot stopped")

    # ==========================================
    # COLOR CALIBRATION
    # ==========================================
    def save_calibration(self):
        """Save HSV ranges to file"""
        data = {
            "ball_lower": self.ball_hsv_lower.tolist() if self.ball_hsv_lower is not None else [],
            "ball_upper": self.ball_hsv_upper.tolist() if self.ball_hsv_upper is not None else [],
            "goal_lower": self.goal_hsv_lower.tolist() if self.goal_hsv_lower is not None else [],
            "goal_upper": self.goal_hsv_upper.tolist() if self.goal_hsv_upper is not None else []
        }
        with open(self.calibration_file, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"✓ Calibration saved to {self.calibration_file}")

    def load_calibration(self):
        """Load HSV ranges from file"""
        if os.path.exists(self.calibration_file):
            try:
                with open(self.calibration_file, 'r') as f:
                    data = json.load(f)
                if data.get("ball_lower"):
                    self.ball_hsv_lower = np.array(data["ball_lower"])
                    self.ball_hsv_upper = np.array(data["ball_upper"])
                    print("✓ Ball calibration loaded")
                if data.get("goal_lower"):
                    self.goal_hsv_lower = np.array(data["goal_lower"])
                    self.goal_hsv_upper = np.array(data["goal_upper"])
                    print("✓ Goal calibration loaded")
            except Exception as e:
                print(f"Load calibration error: {e}")

    def calibrate_object(self, name):
        """
        Interactive calibration for ball or goal
        Original working method - click to start, collects samples while you move cursor
        """
        window_name = f"Calibrate_{name}"
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self._calib_mouse_callback)
        
        self._calib_samples = []
        self._calib_sampling = False
        self._calib_start_time = None
        
        print(f"\n{'='*60}")
        print(f"CALIBRATING: {name.upper()}")
        print(f"{'='*60}")
        print("INSTRUCTIONS:")
        print("1. Move cursor over the object")
        print("2. CLICK to start sampling")
        print("3. Move cursor around object during sampling")
        print(f"4. Collects up to {self.CALIBRATION_SAMPLES} samples over {self.CALIBRATION_DURATION}s")
        print("5. Press 'c' to CONFIRM, 'r' to RESET, 'q' to SKIP")
        print(f"{'='*60}\n")
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("No frame received")
                break
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            vis = frame.copy()
            
            # Draw title
            cv2.putText(vis, f"CALIBRATE: {name.upper()}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # Draw instructions
            cv2.putText(vis, "Click to START 5s sampling (move cursor around object)", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(vis, "'c' = CONFIRM | 'r' = RESET | 'q' = SKIP", 
                       (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Draw cursor crosshair
            cx, cy = self._calib_cursor_pos
            cv2.drawMarker(vis, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 12, 1)
            
            # Sampling status
            if self._calib_sampling:
                elapsed = time.time() - self._calib_start_time
                samples_collected = len(self._calib_samples)
                
                cv2.putText(vis, f"Sampling... {elapsed:.1f}s / {self.CALIBRATION_DURATION}s  samples:{samples_collected}", 
                           (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
                
                # Collect sample at cursor position
                x = max(0, min(self.frame_width - 1, cx))
                y = max(0, min(self.frame_height - 1, cy))
                sample = hsv[y, x]
                self._calib_samples.append(sample.tolist())
                
                # Stop when done
                if samples_collected >= self.CALIBRATION_SAMPLES or elapsed >= self.CALIBRATION_DURATION:
                    self._calib_sampling = False
                    print(f"✓ Sampling complete: {len(self._calib_samples)} samples collected")
            
            else:
                if len(self._calib_samples) > 0:
                    cv2.putText(vis, f"Samples collected: {len(self._calib_samples)}. Press 'c' to confirm or 'r' to reset.", 
                               (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 200), 1)
            
            cv2.imshow(window_name, vis)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("Calibration skipped")
                cv2.destroyWindow(window_name)
                return None, None
            
            elif key == ord('r'):
                print("Resetting samples...")
                self._calib_samples = []
                self._calib_sampling = False
                self._calib_start_time = None
            
            elif key == ord('c') and len(self._calib_samples) > 0:
                # Calculate HSV range from samples
                arr = np.array(self._calib_samples, dtype=np.float32)
                mean = arr.mean(axis=0)
                std = arr.std(axis=0)
                
                # Dynamic margins based on variance
                h_margin = max(10, std[0] * 2.5)
                s_margin = max(40, std[1] * 2.5)
                v_margin = max(40, std[2] * 2.5)
                
                lower = np.array([
                    max(0, mean[0] - h_margin),
                    max(0, mean[1] - s_margin),
                    max(0, mean[2] - v_margin)
                ], dtype=np.uint8)
                
                upper = np.array([
                    min(180, mean[0] + h_margin),
                    min(255, mean[1] + s_margin),
                    min(255, mean[2] + v_margin)
                ], dtype=np.uint8)
                
                print(f"✓ Calibration complete for {name}")
                print(f"  Lower HSV: {lower}")
                print(f"  Upper HSV: {upper}")
                
                cv2.destroyWindow(window_name)
                return lower, upper
            
            time.sleep(0.01)
        
        cv2.destroyWindow(window_name)
        return None, None

    def _calib_mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events during calibration"""
        self._calib_cursor_pos = (x, y)
        
        if event == cv2.EVENT_LBUTTONDOWN:
            if not self._calib_sampling:
                print("Starting sample collection...")
                self._calib_samples = []
                self._calib_sampling = True
                self._calib_start_time = time.time()

    # ==========================================
    # APRILTAG DETECTION
    # ==========================================
    def detect_home_tag(self, frame):
        """
        Detect AprilTag marking home shooting position
        Returns (x, y) center of tag, or None
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect AprilTags
        tags = self.apriltag_detector.detect(gray)
        
        # Look for our home tag ID
        for tag in tags:
            if tag.tag_id == self.HOME_TAG_ID:
                # Get center of tag
                center = tag.center
                return (int(center[0]), int(center[1]))
        
        return None
    
    def is_aligned_with_home(self, home_x):
        """Check if robot is aligned with home position"""
        if home_x is None:
            return False
        error = abs(home_x - self.center_x)
        return error < self.HOME_ALIGNMENT_TOLERANCE

    # ==========================================
    # OBJECT DETECTION
    # ==========================================
    def detect_ball(self, frame):
        """
        Detect ball and return (x, y, radius) or None
        """
        if self.ball_hsv_lower is None:
            return None
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.ball_hsv_lower, self.ball_hsv_upper)
        
        # Morphological operations to clean up mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Find best circular contour
        for c in sorted(contours, key=cv2.contourArea, reverse=True):
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            
            if radius < self.MIN_BALL_RADIUS:
                continue
            
            # Check circularity
            area = cv2.contourArea(c)
            expected_area = math.pi * (radius ** 2)
            circularity = area / expected_area if expected_area > 0 else 0
            
            if circularity >= self.MIN_BALL_CIRCULARITY:
                return (int(x), int(y), int(radius))
        
        return None

    def detect_goal(self, frame):
        """
        Detect goal and return (x, y, size) or None
        """
        if self.goal_hsv_lower is None:
            return None
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.goal_hsv_lower, self.goal_hsv_upper)
        
        # Morphological operations
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Find largest contour
        largest = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(largest) < 500:
            return None
        
        # Get bounding box
        x, y, w, h = cv2.boundingRect(largest)
        cx = x + w // 2
        cy = y + h // 2
        size = max(w, h)
        
        return (cx, cy, size)

    # ==========================================
    # POSSESSION CHECK
    # ==========================================
    def ball_in_possession(self, ball_y):
        """
        Check if ball has crossed the possession line
        Ball is in possession when it's close to bottom of screen
        """
        if ball_y is None:
            return False
        
        # Update tracking
        self.last_ball_y = ball_y
        
        # Check if ball crossed possession line
        if ball_y > self.possession_line_y:
            self.ball_approaching_bottom = True
            return True
        
        return False
    
    def ball_disappeared_at_bottom(self):
        """
        Check if ball disappeared because it went off bottom of screen
        This means we have possession
        """
        # If ball was approaching bottom and now disappeared, we have it
        return self.ball_approaching_bottom and self.last_ball_y is not None and self.last_ball_y > self.possession_line_y

    # ==========================================
    # CONTROL LOGIC - STATE MACHINE
    # ==========================================
    def update_control(self, ball_data, goal_data, home_data):
        """
        Main control state machine
        States are mutually exclusive - once we have possession, we don't check for ball again
        home_data: (x, y) of AprilTag home marker or None
        """
        
        # ============================================
        # STATE: SEEKING_BALL
        # ============================================
        if self.state == "SEEKING_BALL":
            if ball_data is None:
                # Ball disappeared - check if it's because it went off bottom (possession)
                if self.ball_disappeared_at_bottom():
                    self.stop()
                    self.state = "SECURING_BALL"
                    self.debug_message = "Ball disappeared at bottom - in possession!"
                    print(">>> BALL IN POSSESSION (disappeared at bottom)")
                else:
                    # No ball visible - search by rotating RIGHT (clockwise)
                    self.send_velocity(0.0, 0.3, tag="SEARCH_BALL_RIGHT")
                    self.debug_message = "Searching for ball (rotating right)"
                    # Reset tracking when searching
                    self.ball_approaching_bottom = False
                    self.last_ball_y = None
            else:
                bx, by, br = ball_data
                
                # Check if in possession
                if self.ball_in_possession(by):
                    self.stop()
                    self.state = "SECURING_BALL"
                    self.debug_message = "Ball crossed possession line!"
                    print(f">>> BALL IN POSSESSION (crossed line at y={by})")
                else:
                    # Track ball and move toward it
                    # Angular error: positive if ball is left of center
                    angular_error = (bx - self.center_x) / float(self.center_x)
                    angular = self.Kp_angular * angular_error
                    
                    # Forward speed based on distance
                    # Ball near bottom = slow down, ball at top = speed up
                    distance_factor = (self.frame_height - by) / float(self.frame_height)
                    forward = self.Kp_forward * distance_factor + 0.1
                    forward = min(forward, 0.5)  # cap max speed
                    
                    self.send_velocity(forward, angular, tag="TRACK_BALL")
                    self.debug_message = f"Tracking ball at ({bx}, {by})"
            return  # Always return from SEEKING_BALL state
        
        # ============================================
        # STATE: SECURING_BALL
        # ============================================
        if self.state == "SECURING_BALL":
            # Ball crossed line - creep forward slowly to ensure it's between arms
            print(">>> SECURING BALL - creeping forward")
            self.send_velocity(self.POSSESSION_SETTLE_SPEED, 0.0, tag="SECURE_BALL")
            self.debug_message = "Securing ball between arms"
            time.sleep(self.POSSESSION_SETTLE_TIME)
            self.stop()
            
            # Transition to seeking goal
            self.state = "GOAL_SEARCH"
            self.debug_message = "Ball secured - searching for goal"
            print(">>> BALL SECURED - NOW SEARCHING FOR GOAL")
            return  # Return after settling
        
        # ============================================
        # STATE: GOAL_SEARCH
        # ============================================
        if self.state == "GOAL_SEARCH":
            # Rotate RIGHT (counter-clockwise from robot's perspective) to find goal
            # Ball should stay in left (carrying) arm during left rotation
            if goal_data is None:
                self.send_velocity(0.0, 0.25, tag="SEARCH_GOAL_RIGHT")
                self.debug_message = "Searching for goal (rotating right with ball)"
            else:
                gx, gy, gs = goal_data
                
                # Check if goal is centered
                error = abs(gx - self.center_x)
                
                if error < self.GOAL_CENTER_TOLERANCE:
                    # Goal is centered - now look for home marker
                    self.stop()
                    self.state = "MOVING_TO_HOME"
                    self.debug_message = "Goal centered - moving to home position"
                    print(">>> GOAL CENTERED - NOW MOVING TO HOME POSITION")
                else:
                    # Keep rotating to center goal
                    angular_error = (gx - self.center_x) / float(self.center_x)
                    angular = self.Kp_angular * angular_error
                    self.send_velocity(0.0, angular, tag="CENTER_GOAL")
                    self.debug_message = f"Centering goal at ({gx}, {gy})"
            return  # Always return from GOAL_SEARCH state
        
        # ============================================
        # STATE: MOVING_TO_HOME
        # ============================================
        if self.state == "MOVING_TO_HOME":
            # Look for AprilTag home marker and move to it
            if home_data is None:
                # No home tag visible - rotate to find it
                self.send_velocity(0.0, 0.2, tag="SEARCH_HOME")
                self.debug_message = "Searching for home marker (AprilTag)"
            else:
                hx, hy = home_data
                
                # Check if aligned with home
                if self.is_aligned_with_home(hx):
                    # Aligned with home - ready to shoot
                    self.stop()
                    self.state = "ALIGNING_HOME"
                    self.debug_message = "At home position - preparing to shoot"
                    print(">>> AT HOME POSITION - READY TO SHOOT")
                else:
                    # Move toward home marker
                    angular_error = (hx - self.center_x) / float(self.center_x)
                    angular = self.Kp_angular * angular_error
                    # Move slowly forward while aligning
                    self.send_velocity(0.1, angular, tag="MOVE_TO_HOME")
                    self.debug_message = f"Moving to home at ({hx}, {hy})"
            return  # Always return from MOVING_TO_HOME state
        
        # ============================================
        # STATE: ALIGNING_HOME
        # ============================================
        if self.state == "ALIGNING_HOME":
            # Verify still at home and goal still centered before shooting
            if goal_data is None:
                # Lost goal - go back to searching
                self.state = "GOAL_SEARCH"
                self.debug_message = "Lost goal - re-searching"
                return
            
            if home_data is None:
                # Lost home marker - go back to moving
                self.state = "MOVING_TO_HOME"
                self.debug_message = "Lost home marker - repositioning"
                return
            
            gx, gy, gs = goal_data
            hx, hy = home_data
            
            # Check both alignments
            goal_error = abs(gx - self.center_x)
            
            if goal_error < self.GOAL_CENTER_TOLERANCE and self.is_aligned_with_home(hx):
                # Everything aligned - shoot!
                self.state = "SHOOTING"
                self.debug_message = "Executing shot sequence"
                print(">>> ALIGNED AND READY - SHOOTING!")
            else:
                # Lost alignment
                if goal_error >= self.GOAL_CENTER_TOLERANCE:
                    self.state = "GOAL_SEARCH"
                    self.debug_message = "Lost goal alignment - re-centering"
                else:
                    self.state = "MOVING_TO_HOME"
                    self.debug_message = "Lost home alignment - repositioning"
            return  # Always return from ALIGNING_HOME state
        
        # ============================================
        # STATE: ALIGNING_GOAL (DEPRECATED - now use ALIGNING_HOME)
        # ============================================
        if self.state == "ALIGNING_GOAL":
            # Goal is centered - just verify it's still there
            if goal_data is None:
                # Lost goal - go back to searching
                self.state = "GOAL_SEARCH"
                self.debug_message = "Lost goal - re-searching"
            else:
                gx, gy, gs = goal_data
                error = abs(gx - self.center_x)
                
                if error < self.GOAL_CENTER_TOLERANCE:
                    # Still centered - proceed to shoot
                    self.state = "SHOOTING"
                    self.debug_message = "Executing shot sequence"
                else:
                    # Lost alignment - go back to searching
                    self.state = "GOAL_SEARCH"
                    self.debug_message = "Lost goal alignment - re-searching"
            return  # Always return from ALIGNING_GOAL state
        
        # ============================================
        # STATE: SHOOTING
        # ============================================
        if self.state == "SHOOTING":
            self.execute_shot()
            
            # IMMEDIATELY stop after shooting
            self.stop()
            time.sleep(0.2)  # Brief pause to ensure stop command is sent
            
            # Reset to searching for next ball
            self.state = "SEEKING_BALL"
            self.ball_approaching_bottom = False
            self.last_ball_y = None
            self.debug_message = "Shot complete - searching for ball"
            return  # Always return from SHOOTING state

    # ==========================================
    # SHOOTING SEQUENCE
    # ==========================================
    def execute_shot(self):
        """
        Execute simple shooting sequence:
        1. Backup to create space
        2. Spin RIGHT (clockwise) at IMMEDIATE MAX SPEED to strike ball with right arm
        3. Stop immediately
        
        TUNABLE PARAMETERS:
        - self.SHOOT_BACKUP_SPEED
        - self.SHOOT_BACKUP_DISTANCE
        - self.SHOOT_SPIN_SPEED
        - self.SHOOT_SPIN_DURATION
        """
        
        print("\n" + "="*60)
        print("SHOOTING SEQUENCE - BACKUP & SPIN RIGHT")
        print("="*60)
        
        # Step 1: Backup to create space
        print(f"[1/3] Backup: reversing at {self.SHOOT_BACKUP_SPEED} for {self.SHOOT_BACKUP_DISTANCE}s")
        # Negative linear = backward (camera perspective)
        self.send_velocity(-self.SHOOT_BACKUP_SPEED, 0.0, tag="BACKUP")
        time.sleep(self.SHOOT_BACKUP_DISTANCE)
        self.stop()
        time.sleep(0.2)
        
        # Step 2: SPIN RIGHT AT IMMEDIATE MAX SPEED - STRIKE THE BALL!
        print(f"[2/3] STRIKE: Spinning RIGHT at MAX SPEED {self.SHOOT_SPIN_SPEED} for {self.SHOOT_SPIN_DURATION}s")
        # Use SHOOT_SPIN command to bypass smoothing for immediate max speed
        # Camera perspective: positive angular = right/clockwise
        # Robot perspective: need to invert, so send negative
        robot_angular = -self.SHOOT_SPIN_SPEED  # Invert for rear-mounted camera
        cmd = f"SHOOT_SPIN:{robot_angular:.2f}:{self.SHOOT_SPIN_DURATION}"
        print(f"[SHOOT_SPIN] {cmd}")
        self.send_command(cmd)
        time.sleep(self.SHOOT_SPIN_DURATION)
        
        # Step 3: IMMEDIATE STOP
        print(f"[3/3] STOP IMMEDIATELY")
        self.stop()
        time.sleep(0.1)
        
        print("="*60)
        print("SHOT COMPLETE - RIGHT SPIN STRIKE EXECUTED AT MAX SPEED")
        print("="*60 + "\n")

    # ==========================================
    # VISUALIZATION
    # ==========================================
    def draw_overlay(self, frame, ball_data, goal_data, home_data):
        """Draw debug overlay on frame"""
        vis = frame.copy()
        
        # Draw possession line
        cv2.line(vis, (0, self.possession_line_y), (self.frame_width, self.possession_line_y), 
                (0, 255, 255), 2)
        cv2.putText(vis, "POSSESSION LINE", (10, self.possession_line_y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Draw center crosshair
        cv2.drawMarker(vis, (self.center_x, self.center_y), (200, 200, 200),
                      cv2.MARKER_CROSS, 20, 1)
        
        # Draw state and debug info
        y_offset = 30
        cv2.putText(vis, f"STATE: {self.state}", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        y_offset += 30
        cv2.putText(vis, self.debug_message, (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Draw ball
        if ball_data:
            bx, by, br = ball_data
            cv2.circle(vis, (bx, by), br, (0, 255, 0), 3)
            cv2.circle(vis, (bx, by), 3, (0, 255, 0), -1)
            cv2.putText(vis, f"BALL ({bx},{by})", (bx + 15, by - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            if self.ball_in_possession(by):
                cv2.putText(vis, "IN POSSESSION", (bx - 50, by + 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw goal
        if goal_data:
            gx, gy, gs = goal_data
            half = gs // 2
            cv2.rectangle(vis, (gx - half, gy - half), (gx + half, gy + half),
                         (255, 0, 255), 3)
            cv2.circle(vis, (gx, gy), 5, (255, 0, 255), -1)
            cv2.putText(vis, f"GOAL ({gx},{gy})", (gx + 15, gy - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
            
            # Draw center alignment indicator
            error = abs(gx - self.center_x)
            if error < self.GOAL_CENTER_TOLERANCE:
                cv2.putText(vis, "CENTERED!", (gx - 40, gy + 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw home marker (AprilTag)
        if home_data:
            hx, hy = home_data
            # Draw large crosshair for home position
            cv2.drawMarker(vis, (hx, hy), (0, 255, 255), cv2.MARKER_TILTED_CROSS, 40, 3)
            cv2.circle(vis, (hx, hy), 60, (0, 255, 255), 3)
            cv2.putText(vis, f"HOME ({hx},{hy})", (hx + 70, hy),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Draw alignment indicator
            if self.is_aligned_with_home(hx):
                cv2.putText(vis, "AT HOME!", (hx - 50, hy - 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        return vis

    # ==========================================
    # MAIN LOOP
    # ==========================================
    def run(self):
        """Main execution loop"""
        
        print("\n" + "="*60)
        print("STARTING CALIBRATION SEQUENCE")
        print("="*60 + "\n")
        
        # Calibrate ball
        ball_lower, ball_upper = self.calibrate_object("ball")
        if ball_lower is not None:
            self.ball_hsv_lower = ball_lower
            self.ball_hsv_upper = ball_upper
        else:
            print("WARNING: Ball calibration skipped - trying to load from file")
            self.load_calibration()
        
        # Calibrate goal
        goal_lower, goal_upper = self.calibrate_object("goal")
        if goal_lower is not None:
            self.goal_hsv_lower = goal_lower
            self.goal_hsv_upper = goal_upper
        else:
            print("WARNING: Goal calibration skipped - trying to load from file")
            self.load_calibration()
        
        # Save calibration
        self.save_calibration()
        
        print("\n" + "="*60)
        print("CALIBRATION COMPLETE")
        print("="*60)
        print("Press 's' to START searching for ball")
        print("Press 'q' to QUIT")
        print("="*60 + "\n")
        
        # Main loop
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to read frame")
                    continue
                
                # Detect objects
                ball_data = self.detect_ball(frame)
                goal_data = self.detect_goal(frame)
                home_data = self.detect_home_tag(frame)  # Detect AprilTag home marker
                
                # Update control if running
                if self.running:
                    self.update_control(ball_data, goal_data, home_data)
                
                # Draw and display
                vis = self.draw_overlay(frame, ball_data, goal_data, home_data)
                cv2.imshow("Soccer Robot", vis)
                
                # Handle keyboard
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    print("Quit requested")
                    break
                
                elif key == ord('s'):
                    if not self.running:
                        print("\n" + "="*60)
                        print("STARTING SEARCH - SEEKING BALL")
                        print("="*60 + "\n")
                        self.running = True
                        self.state = "SEEKING_BALL"
                        self.debug_message = "Search started"
                    else:
                        print("Already running")
                
                elif key == ord(' '):  # spacebar to pause/resume
                    self.running = not self.running
                    if not self.running:
                        self.stop()
                        print("PAUSED")
                    else:
                        print("RESUMED")
                
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received")
        
        finally:
            self.cleanup()

    # ==========================================
    # CLEANUP
    # ==========================================
    def cleanup(self):
        """Clean shutdown"""
        print("\nShutting down...")
        try:
            self.stop()
        except:
            pass
        
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        
        if self.cap:
            self.cap.release()
        
        cv2.destroyAllWindows()
        print("Cleanup complete")


# ==========================================
# MAIN ENTRY POINT
# ==========================================
if __name__ == "__main__":
    # Change this IP to match your robot
    ROBOT_IP = "10.247.137.204"
    
    robot = SoccerRobot(ROBOT_IP)
    robot.run()
  
