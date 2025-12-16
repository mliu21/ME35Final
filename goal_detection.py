import cv2
import numpy as np
import serial
import time

# ESP32 Serial connection
ESP32_PORT = '/dev/cu.usbserial-0001'  # Change to your port: COM3 (Windows), /dev/cu.usbserial-* (Mac)
ESP32_BAUD = 115200
CAMERA_INDEX = 0  # Change this if wrong camera: 0, 1, 2, etc.

# Try to connect to ESP32
ser = None
try:
    ser = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=1)
    time.sleep(2)  # Wait for connection to establish
    print("ESP32 connected successfully")
except Exception as e:
    print(f"Could not connect to ESP32: {e}")
    print("Running in TEST MODE - goals will be detected but motor won't spin")
    ser = None

# Motion detection parameters
MOTION_THRESHOLD = 1000  # Minimum motion pixels to detect (lowered for better sensitivity)
GOAL_MARGIN = 30  # Pixels to extend goal zone (to catch balls on edges)
MIN_MOTION_AREA = 100  # Minimum contour area to count as real motion

# HSV range for red posts (tune during calibration)
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])

# Background subtraction and tracking variables
bg_subtractor = cv2.createBackgroundSubtractorMOG2(history=500, 
                                                     varThreshold=16, 
                                                     detectShadows=False)
goal_cooldown = 0

def find_goal_posts(frame):
    """Detect red goal posts and return their center positions"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Detect red posts (red wraps around in HSV, so need two ranges)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)
    
    # Find contours of red objects
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, 
                                    cv2.CHAIN_APPROX_SIMPLE)
    
    posts = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 200:  # Minimum post size (adjust if needed)
            # Calculate center of contour
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                posts.append((cx, cy))
    
    # Return the two posts (sort by x-coordinate to get left and right)
    if len(posts) >= 2:
        posts = sorted(posts, key=lambda p: p[0])
        return posts[0], posts[1]
    return None, None

def create_goal_zone_mask(frame_shape, post1, post2, margin=GOAL_MARGIN):
    """Create a mask for the rectangular region between goal posts"""
    mask = np.zeros(frame_shape[:2], dtype=np.uint8)
    
    # Get the bounding box between the two posts (with margin)
    x_min = min(post1[0], post2[0]) - margin
    x_max = max(post1[0], post2[0]) + margin
    y_min = min(post1[1], post2[1]) - margin
    y_max = max(post1[1], post2[1]) + margin
    
    # Ensure coordinates are within frame bounds
    x_min = max(0, x_min)
    x_max = min(frame_shape[1], x_max)
    y_min = max(0, y_min)
    y_max = min(frame_shape[0], y_max)
    
    # Draw the goal zone as a filled rectangle
    cv2.rectangle(mask, (x_min, y_min), (x_max, y_max), 255, -1)
    
    return mask, (x_min, y_min, x_max, y_max)

def detect_motion_in_zone(frame, goal_zone_mask):
    """Detect motion only within the goal zone"""
    # Apply background subtraction
    fg_mask = bg_subtractor.apply(frame, learningRate=0.001)
    
    # Threshold to get clear foreground
    _, fg_mask = cv2.threshold(fg_mask, 244, 255, cv2.THRESH_BINARY)
    
    # Apply the goal zone mask to only look at motion in the goal area
    motion_in_goal = cv2.bitwise_and(fg_mask, fg_mask, mask=goal_zone_mask)
    
    # Remove noise with morphological operations
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    motion_in_goal = cv2.morphologyEx(motion_in_goal, cv2.MORPH_OPEN, kernel)
    motion_in_goal = cv2.dilate(motion_in_goal, kernel, iterations=2)
    
    # Find contours to get actual moving objects
    contours, _ = cv2.findContours(motion_in_goal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Count significant motion (filter out tiny noise)
    motion_pixels = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > MIN_MOTION_AREA:
            motion_pixels += area
    
    return motion_in_goal, int(motion_pixels)

# Initialize camera
cap = cv2.VideoCapture(CAMERA_INDEX)

# Check if camera opened successfully
if not cap.isOpened():
    print(f"Error: Could not open camera at index {CAMERA_INDEX}")
    print("Try changing CAMERA_INDEX to 0, 1, or 2")
    exit()

print(f"Using camera index: {CAMERA_INDEX}")
print("Starting goal detection system...")
print("Camera view: From ABOVE the goal posts")
print("Detecting motion BETWEEN posts only")
print("Press 'q' to quit")
print("")

# Warm up the background subtractor
print("Calibrating background... (3 seconds)")
for i in range(90):  # ~3 seconds at 30fps
    ret, frame = cap.read()
    if ret:
        bg_subtractor.apply(frame)
time.sleep(0.5)

print("System ready! Watching for goals...")
print("")

# Main detection loop
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Could not read frame")
        break
    
    # Create a copy for drawing visualizations
    display = frame.copy()
    
    # Find goal posts
    post1, post2 = find_goal_posts(frame)
    
    if post1 and post2:
        # Create goal zone mask
        goal_zone_mask, (x_min, y_min, x_max, y_max) = create_goal_zone_mask(
            frame.shape, post1, post2
        )
        
        # Draw goal zone visualization (make it more visible)
        # Draw filled semi-transparent rectangle
        overlay = display.copy()
        cv2.rectangle(overlay, (x_min, y_min), (x_max, y_max), 
                     (0, 255, 0), -1)  # Filled green
        cv2.addWeighted(overlay, 0.3, display, 0.7, 0, display)
        
        # Draw border
        cv2.rectangle(display, (x_min, y_min), (x_max, y_max), 
                     (0, 255, 0), 3)
        cv2.circle(display, post1, 10, (0, 0, 255), -1)
        cv2.circle(display, post2, 10, (0, 0, 255), -1)
        
        # Show goal zone dimensions
        zone_width = x_max - x_min
        zone_height = y_max - y_min
        cv2.putText(display, f"Zone: {zone_width}x{zone_height}px", (x_min, y_min-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        status = "ESP32 Connected" if ser else "TEST MODE"
        cv2.putText(display, status, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if ser else (0, 165, 255), 2)
        
        # Detect motion in goal zone
        motion_mask, motion_pixels = detect_motion_in_zone(frame, goal_zone_mask)
        
        # Display motion amount with color coding
        motion_color = (0, 255, 0) if motion_pixels > MOTION_THRESHOLD else (255, 255, 255)
        cv2.putText(display, f"Motion: {motion_pixels} (Need: {MOTION_THRESHOLD})", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, motion_color, 2)
        
        # Debug: Show when motion is detected IN THE GOAL ZONE ONLY
        if motion_pixels > 50:  # Show any significant motion
            print(f"Motion IN GOAL ZONE: {motion_pixels} pixels (threshold: {MOTION_THRESHOLD})")
            print(f"   Goal zone bounds: x={x_min}-{x_max}, y={y_min}-{y_max}")
        
        # Check for goal (significant motion in goal zone)
        if motion_pixels > MOTION_THRESHOLD and goal_cooldown == 0:
            print(f"GOAL SCORED! Motion detected: {motion_pixels}")
            cv2.putText(display, "GOAL!", (50, 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 3,
                       (0, 255, 0), 5)
            
            # Send signal to ESP32
            if ser:
                try:
                    print("   Attempting to send: GOAL")
                    ser.write(b'GOAL')  # No newline
                    ser.flush()  # Force send immediately
                    time.sleep(0.05)  # Brief delay
                    print(f"   Sent successfully - Motor should spin!")
                except Exception as e:
                    print(f"   Error sending to ESP32: {e}")
            
            goal_cooldown = 60  # Cooldown period (adjust as needed)
        
        # Optional: Show motion mask in corner
        motion_vis = cv2.cvtColor(motion_mask, cv2.COLOR_GRAY2BGR)
        h, w = motion_vis.shape[:2]
        small_motion = cv2.resize(motion_vis, (w//4, h//4))
        display[10:10+h//4, display.shape[1]-w//4-10:display.shape[1]-10] = small_motion
        
    else:
        cv2.putText(display, "Searching for goal posts...", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    # Decrease cooldown counter
    if goal_cooldown > 0:
        goal_cooldown -= 1
        cv2.putText(display, f"Cooldown: {goal_cooldown}", (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    
    # Show the frame
    cv2.imshow("Goal Detection System", display)
    
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite('goal_detection_output.jpg', frame)  # Save last frame
        break

# Release everything when done
cap.release()
cv2.destroyAllWindows()
if ser:
    ser.close()
    print("ESP32 connection closed")
print("System shut down successfully")
