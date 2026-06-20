from object_detection import detect_objects, stop_camera
import cv2
import numpy as np
import serial
import time
#import matplotlib
#matplotlib.use('Agg')  # Use headless backend to prevent Qt/X11 GUI errors
#import matplotlib.pyplot as plt

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0)
time.sleep(3)  # wait for Arduino to initialize

last_reconnect_time = 0

def safe_serial_write(data):
    global ser, last_reconnect_time

    if ser is None or not ser.is_open:
        current_time = time.time()
        if current_time - last_reconnect_time > 3.0:
            last_reconnect_time = current_time

            try:
                if ser is not None:
                    ser.close()
            except:
                pass

            try:
                ser = serial.Serial(
                    '/dev/ttyACM0',
                    9600,
                    timeout=0,
                    write_timeout=0.1
                )
                time.sleep(0.5)
                print("✅ Serial reconnected successfully!")
            except Exception as reconnect_error:
                print(f"❌ Reconnection failed: {reconnect_error}")

        return

    try:
        ser.write(data)        # <-- THIS IS THE CORRECT LINE
    except (serial.SerialException, OSError) as e:
        print(f"⚠️ Serial write failed: {e}")

        try:
            ser.close()
        except:
            pass
def start_sweep():
    command_str = f"START_SWEEP\n"
    safe_serial_write(command_str.encode())

# ---------------------------
# 1. Perspective Transform
# ---------------------------
def perspective_transform(img):
    h, w = img.shape[:2]
    src = np.float32([
        [w*0.27, h*0.78],   # top-left
        [w*0.80, h*0.78],   # top-right
        [w*0.83, h*0.98],   # bottom-right
        [w*0.23, h*0.98]    # bottom-left
    ])
 
    dst = np.float32([
        [w*0.12, 0],     # top-left
        [w*0.88, 0],     # top-right
        [w*0.88, h],     # bottom-right
        [w*0.12, h]      # bottom-left
    ])
    debug = img.copy()
    pts = np.array(src, np.int32)
    cv2.polylines(debug, [pts], True, (0,255,0), 3)
    cv2.imshow("ROI", debug)
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = np.linalg.inv(M)
    warped = cv2.warpPerspective(img, M, (w, h))
    return warped, Minv, M

def is_inside_lane(box, left_fit, right_fit, M):
    if left_fit is None or right_fit is None:
        return False
    # box format: [x1, y1, x2, y2]
    x1, y1, x2, y2 = box
    x_center = (x1 + x2) / 2.0
    y_bottom = float(y2)
    
    # Project to warped space
    pts = np.array([[[x_center, y_bottom]]], dtype=np.float32)
    pts_warped = cv2.perspectiveTransform(pts, M)
    xw, yw = pts_warped[0][0]
    
    x_left = np.polyval(left_fit, yw)
    x_right = np.polyval(right_fit, yw)
    
    return x_left <= xw <= x_right

# ---------------------------
# 2. White Mask
# ---------------------------
def threshold_white(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv", hsv)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 40, 255])
    return cv2.inRange(hsv, lower_white, upper_white)

# ---------------------------
# 3. Sliding Window
# ---------------------------
def sliding_window(binary_warped):
    histogram = np.sum(
        binary_warped[int(binary_warped.shape[0]*0.72):, :],
        axis=0
    )
    #print(f"histogram {histogram}")
    midpoint = histogram.shape[0] // 2
    # if current_state in (STATE_LANE_CHANGE_LEFT, STATE_LANE_CHANGE_RIGHT):
    #     # Find all peaks in the histogram above a threshold
    #     peaks = []
    #     win_size = 40
    #     # Iterate from left to right (leaving some margin from borders)
    #     for i in range(win_size, len(histogram) - win_size):
    #         # Check if this point is a local maximum
    #         if histogram[i] > 1000 and histogram[i] == np.max(histogram[i - win_size : i + win_size + 1]):
    #             # Ensure it is sufficiently far from other peaks to avoid double detection of same line
    #             if not peaks or all(abs(i - p) > 200 for p in peaks):
    #                 peaks.append(i)
    #     peaks.sort()
    #     if len(peaks) >= 2:
    #         leftx_base = peaks[0]
    #         rightx_base = peaks[1]
    #     else:
    #         leftx_base = np.argmax(histogram[:midpoint])
    #         rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    # else:
    #     leftx_base = np.argmax(histogram[:midpoint])
    #     rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    #print(f"leftx_base {leftx_base}")
    #print(f"rightx_base {rightx_base}")
    nwindows = 9
    window_height = binary_warped.shape[0] // nwindows
    # IMPORTANT
    margin = 60
    minpix = 50
    # Get all white pixels
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Create debug image
    out_img = np.dstack(
        (binary_warped, binary_warped, binary_warped)
    )
    leftx_current = leftx_base
    rightx_current = rightx_base
    left_lane_inds = []
    right_lane_inds = []

    for window in range(nwindows):
        # Window boundaries in Y
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        # Window boundaries in X
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        # DRAW WINDOWS
        cv2.rectangle(
            out_img,
            (win_xleft_low, win_y_low),
            (win_xleft_high, win_y_high),
            (0, 255, 0),
            2
        )
        cv2.rectangle(
            out_img,
            (win_xright_low, win_y_low),
            (win_xright_high, win_y_high),
            (0, 255, 0),
            2
        )

        # Find white pixels inside left window
        good_left = (
            (nonzeroy >= win_y_low) &
            (nonzeroy < win_y_high) &
            (nonzerox >= win_xleft_low) &
            (nonzerox < win_xleft_high)
        ).nonzero()[0]

        # Find white pixels inside right window
        good_right = (
            (nonzeroy >= win_y_low) &
            (nonzeroy < win_y_high) &
            (nonzerox >= win_xright_low) &
            (nonzerox < win_xright_high)
        ).nonzero()[0]

        left_lane_inds.append(good_left)
        right_lane_inds.append(good_right)

        # Move left window center
        if len(good_left) > minpix:
            leftx_current = int(
                np.mean(nonzerox[good_left])
            )
        # Move right window center
        if len(good_right) > minpix:
            rightx_current = int(
                np.mean(nonzerox[good_right])
            )

    # Merge all indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # COLOR DETECTED PIXELS
    # Left lane = BLUE
    out_img[
        nonzeroy[left_lane_inds],
        nonzerox[left_lane_inds]
    ] = [255, 0, 0]
    # Right lane = RED
    out_img[
        nonzeroy[right_lane_inds],
        nonzerox[right_lane_inds]
    ] = [0, 0, 255]
    # SHOW DEBUG IMAGE
    cv2.imshow("Sliding Windows", out_img)
    return (
        nonzerox[left_lane_inds],
        nonzeroy[left_lane_inds],
        nonzerox[right_lane_inds],
        nonzeroy[right_lane_inds]
    )

# try max_adjust 130 -140 
# base_speed 135
def compute_pwm(error, base_speed=135, max_adjust=130):
    # More sensitive steering
    error = np.clip(error, -80, 80)
    adjust = (error / 80) * max_adjust # [-130-130]
    base_speed = 135
    #if abs(error) > 25:
     #   base_speed = 110
    kp = 15
    #adjust = int(error * kp)
    #if abs(error) < 10:
     #   adjust = error * 0.7
    #elif abs(error) < 30:
     #   adjust = error * 1
    #else:
     #   adjust = error * 2    
     
    left_pwm = base_speed + adjust
    right_pwm = base_speed - adjust
    left_pwm = int(np.clip(left_pwm, 0, 255))
    right_pwm = int(np.clip(right_pwm, 0, 255))
    return left_pwm, right_pwm

# ---------------------------
# 4. Fit Curves
# ---------------------------
def fit_polynomial(binary_warped):
    leftx, lefty, rightx, righty = sliding_window(binary_warped)
    w = binary_warped.shape[1]
    left_valid = len(leftx) >= 50
    right_valid = len(rightx) >= 50

    # Prevent both trackers from locking onto the same line (crossover / too close)
    if left_valid and right_valid:
        mean_left = np.mean(leftx)
        mean_right = np.mean(rightx)
        if (mean_right - mean_left) < 250:
            # They are too close, one is a duplicate
            if mean_left > w / 2:
                left_valid = False  # both are tracking the right line
            else:
                right_valid = False  # both are tracking the left line

    lane_width = 580  # Expected distance between lines in pixels
    if left_valid and right_valid:
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
    elif left_valid:
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = left_fit.copy()
        right_fit[2] += lane_width
    elif right_valid:
        right_fit = np.polyfit(righty, rightx, 2)
        left_fit = right_fit.copy()
        left_fit[2] -= lane_width
    else:
        return None, None, False, False

    # Create visualization image
    out_img = np.dstack(
        (binary_warped, binary_warped, binary_warped)
    )
    # Generate y values
    ploty = np.linspace(
        0,
        binary_warped.shape[0]-1,
        binary_warped.shape[0]
    )
    # Generate fitted x values
    left_fitx = np.polyval(left_fit, ploty)
    right_fitx = np.polyval(right_fit, ploty)

    # Draw detected pixels
    if left_valid:
        out_img[lefty, leftx] = [255, 0, 0]
    if right_valid:
        out_img[righty, rightx] = [0, 0, 255]

    # Draw polynomial curves
    for i in range(len(ploty)-1):
        cv2.line(
            out_img,
            (int(left_fitx[i]), int(ploty[i])),
            (int(left_fitx[i+1]), int(ploty[i+1])),
            (0, 255, 255),
            3
        )
        cv2.line(
            out_img,
            (int(right_fitx[i]), int(ploty[i])),
            (int(right_fitx[i+1]), int(ploty[i+1])),
            (0, 255, 255),
            3
        )
    #cv2.imshow("Polyfit Curves", out_img)
    return left_fit, right_fit, left_valid, right_valid

# ---------------------------
# 5. Steering Logic & Pure Pursuit Reference Path
# ---------------------------
STATE_LANE_FOLLOW = 'LANE_FOLLOW'
STATE_LANE_CHANGE_LEFT = 'LANE_CHANGE_LEFT'
STATE_LANE_CHANGE_RIGHT = 'LANE_CHANGE_RIGHT'
STATE_PAUSE_LEFT = 'PAUSE_LEFT'
STATE_PAUSE_RIGHT = 'PAUSE_RIGHT'
STATE_ALIGN_LEFT = 'ALIGN_LEFT'
STATE_ALIGN_RIGHT = 'ALIGN_RIGHT'
STATE_STOP_SIGN_WAIT = 'STOP_SIGN_WAIT'
STATE_STOP = 'STOP'
STATE_UTURN_STOP1 = 'UTURN_STOP1'
STATE_UTURN_FORWARD = 'UTURN_FORWARD'
STATE_UTURN_STOP2 = 'UTURN_STOP2'
STATE_UTURN_STEER = 'UTURN_STEER'
STATE_UTURN_STOP_FINAL = 'UTURN_STOP_FINAL'

current_state = STATE_LANE_FOLLOW
current_lane = 'RIGHT'  # Start lane: 'RIGHT' or 'LEFT'
state_start_time = 0

PAUSE_DURATION = 0.2     # Pause duration in seconds to stabilize camera before steering to find boundaries
LANE_WIDTH = 620          # Shift offset in pixels for lane change
MERGE_DISTANCE = 150      # S-curve merge distance in pixels
LANE_CHANGE_DURATION = 2.0  # Duration of lane change in seconds
LOOK_AHEAD_FACTOR = 0.7    # Look-ahead height factor
ALIGN_DURATION = 15.0      # Duration of counter-steering to straighten nose (in seconds)
ALIGN_STEER_OFFSET = 135   # Steering speed adjustment during alignment
UTURN_MIN_STEER_DURATION = 3.5  # Min steering duration in seconds before checking lane boundaries
UTURN_TIMEOUT = 10.0   

# Ultrasonic minimum distance (updated from Arduino serial data)
min_dist = 999.0

# Cooldowns and timers for stop signs / red lights
stop_until = 0.0
ignore_stop_until = 0.0
skip_detection_until = 0.0
uturn_cooldown_until = 0.0

prev_left_fit = None
prev_right_fit = None
consecutive_fail_count = 0
MAX_FAIL_FRAMES = 10

def get_ref_x(y, left_fit, right_fit, w, h):
    center_fit = (left_fit + right_fit) / 2.0
    base_center = np.polyval(center_fit, y)
    
    # Calculate dynamic lane width at this Y height
    x_left = np.polyval(left_fit, y)
    x_right = np.polyval(right_fit, y)
    dynamic_lane_width = x_right - x_left
    
    if current_state == STATE_LANE_CHANGE_RIGHT:
        target_center = base_center + dynamic_lane_width
    elif current_state == STATE_LANE_CHANGE_LEFT:
        target_center = base_center - dynamic_lane_width
    else:
        target_center = base_center
        
    if current_state in (STATE_LANE_CHANGE_LEFT, STATE_LANE_CHANGE_RIGHT):
        if y < h - MERGE_DISTANCE:
            return target_center
        else:
            u = (h - y) / MERGE_DISTANCE
            f_u = 3 * u**2 - 2 * u**3
            return (1 - f_u) * (w / 2) + f_u * target_center
    else:
        return target_center


def draw_reference_path(img, left_fit, right_fit, Minv, shape):
    h, w = shape[:2]
    
    ploty = np.linspace(0, h-1, 20)
    pts_warped = []
    for y in ploty:
        x = get_ref_x(y, left_fit, right_fit, w, h)
        pts_warped.append([x, y])
    
    pts_warped = np.array([pts_warped], dtype=np.float32)
    pts_unwarped = cv2.perspectiveTransform(pts_warped, Minv)
    pts_unwarped = pts_unwarped[0].astype(np.int32)
    
    # Draw path as Cyan line
    cv2.polylines(img, [pts_unwarped], False, (255, 255, 0), 3)
    
    # Draw look-ahead point as Magenta circle
    y_look_ahead = h * LOOK_AHEAD_FACTOR
    x_look_ahead = get_ref_x(y_look_ahead, left_fit, right_fit, w, h)
    la_pts = np.array([[[x_look_ahead, y_look_ahead]]], dtype=np.float32)
    la_unwarped = cv2.perspectiveTransform(la_pts, Minv)
    la_x, la_y = la_unwarped[0][0].astype(np.int32)
    cv2.circle(img, (la_x, la_y), 10, (255, 0, 255), -1)

prev_error = 0
def compute_steering(left_fit, right_fit, shape):
    global prev_error
    h, w = shape[:2]

    # Evaluate at look-ahead distance (h * LOOK_AHEAD_FACTOR)
    y_look_ahead = h * LOOK_AHEAD_FACTOR
    target_x = get_ref_x(y_look_ahead, left_fit, right_fit, w, h)

    car_center = w // 2
    error = target_x - car_center

    # smoothing
    error = 0.3 * prev_error + 0.7 * error
    prev_error = error

    return error, target_x, car_center

def get_command(error, threshold=0):
    if error > threshold:
        return 'RIGHT'
    elif error < -threshold:
        return 'LEFT'
    else:
        return 'STRAIGHT'

# ---------------------------
# 6. Draw Lane
# ---------------------------
def draw_lane(img, binary, left_fit, right_fit, Minv):
    h, w = binary.shape
    ploty = np.linspace(0, h-1, h)
    left_x = np.polyval(left_fit, ploty)
    right_x = np.polyval(right_fit, ploty)
    lane_img = np.zeros_like(img)
    pts_left = np.array([np.transpose(np.vstack([left_x, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_x, ploty])))])
    pts = np.hstack((pts_left, pts_right)).astype(np.int32)
    cv2.fillPoly(lane_img, [pts], (0, 255, 0))
    overlay = cv2.warpPerspective(lane_img, Minv, (w, h))
    return cv2.addWeighted(img, 1, overlay, 0.3, 0)

def detect_lane_end(binary_img):

    h, w = binary_img.shape

    # Focus only on lower-middle area
    roi = binary_img[int(h*0.65):int(h*0.90), :]

    lines = cv2.HoughLinesP(
        roi,
        1,
        np.pi / 180,
        threshold=50,
        minLineLength=250,
        maxLineGap=30
    )

    if lines is None:
        return False

    for line in lines:

        x1, y1, x2, y2 = line[0]

        dx = x2 - x1
        dy = y2 - y1

        slope = dy / (dx + 1e-6)

        line_length = np.sqrt(dx**2 + dy**2)

        # Detect horizontal line
        if abs(slope) < 0.15 and line_length > 300:

            return True

    return False

# ---------------------------
# MAIN
# ---------------------------
print("🚗 Unified control started (CTRL+C to stop)")

last_send_time = 0
frame_count = 0

try:
    start_sweep()
    #time.sleep(2)
    while True:
        # ========================================================
        # READ TELEMETRY FROM ARDUINO (non-blocking)
        # Expected format: ANG:90,DIST:32.4
        # ========================================================
        if ser is not None and ser.is_open:
            try:
                while ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if "DIST:" in line:
                        dist_val = float(line.split(":")[1].strip())
                        min_dist = dist_val
            except Exception as e:
                pass

        # Call NMS object detection (captures and processes the frame)
        now = time.time()
        skip_yolo = now < skip_detection_until
        frame, detections = detect_objects(skip_yolo=skip_yolo)
        frame_count += 1

        # Process lane detection
        warped, Minv, M = perspective_transform(frame)
        mask = threshold_white(warped)
        cv2.imshow("Mask", mask)
        
        # Detect lane lines with fallback support for frame drops
        left_fit, right_fit, left_valid, right_valid = fit_polynomial(mask)
        if left_fit is not None:
            prev_left_fit = left_fit
            prev_right_fit = right_fit
            consecutive_fail_count = 0
        else:
            if prev_left_fit is not None and consecutive_fail_count < MAX_FAIL_FRAMES:
                left_fit = prev_left_fit
                right_fit = prev_right_fit
                consecutive_fail_count += 1
                left_valid = False
                right_valid = False
            else:
                left_fit = None
                right_fit = None
                left_valid = False
                right_valid = False

        # Parse detections and draw bounding boxes
        obstacle_detected_in_lane = False
        yellow_detected_in_lane = False
        green_detected = False
        red_light_outside = False
        stop_sign_inside = False
        stop_sign_outside = False
        
        left_lane_has_obstacle = False
        right_lane_has_obstacle = False
        
        for detection in detections:
            conf = detection["conf"]
            if conf <= 0.85:
                continue

            x1, y1, x2, y2 = detection["box"]
            name = detection["name"]
            
            print(f"🔍 Detected {name} with conf: {conf:.2f}")
            
            # Check if object is inside current lane
            in_lane = is_inside_lane((x1, y1, x2, y2), left_fit, right_fit, M)
            
            # Choose border color: Red if inside lane, Green if outside
            color = (0, 0, 255) if in_lane else (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{name} {conf:.2f} ({'in' if in_lane else 'out'})", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            # Check left and right lane occupancy for ANY class of detected object
            if left_fit is not None and right_fit is not None:
                x_center = (x1 + x2) / 2.0
                y_bottom = float(y2)
                pts = np.array([[[x_center, y_bottom]]], dtype=np.float32)
                pts_warped = cv2.perspectiveTransform(pts, M)
                xw, yw = pts_warped[0][0]
                
                x_left = np.polyval(left_fit, yw)
                x_right = np.polyval(right_fit, yw)
                lane_width_pixels = x_right - x_left
                
                # Check current lane
                if x_left <= xw <= x_right:
                    obstacle_detected_in_lane = True
                    if current_lane == 'RIGHT':
                        right_lane_has_obstacle = True
                    else:
                        left_lane_has_obstacle = True
                # Check adjacent left lane
                elif (x_left - lane_width_pixels) <= xw < x_left:
                    if current_lane == 'RIGHT':
                        left_lane_has_obstacle = True
                # Check adjacent right lane
                elif x_right < xw <= (x_right + lane_width_pixels):
                    if current_lane == 'LEFT':
                        right_lane_has_obstacle = True
            
            # Additional flags for traffic control objects
            if name == "stop-sign":
                if in_lane:
                    stop_sign_inside = True
                else:
                    stop_sign_outside = True
            elif name == "red":
                if not in_lane:
                    red_light_outside = True
            elif name == "yellow":
                if in_lane:
                    yellow_detected_in_lane = True
            elif name == "green":
                green_detected = True

        has_obstacle = obstacle_detected_in_lane
        obstacle_dist = min_dist if has_obstacle else 999.0

        if left_fit is not None and right_fit is not None:
            left_lane_status = "OBSTACLE" if left_lane_has_obstacle else "EMPTY"
            right_lane_status = "OBSTACLE" if right_lane_has_obstacle else "EMPTY"
        else:
            left_lane_status = "UNKNOWN"
            right_lane_status = "UNKNOWN"

        # ========================================================
        # STATE MACHINE TRANSITIONS
        # ========================================================
        now = time.time()
        both_blocked_slow_down = False
        
        if current_state == STATE_LANE_FOLLOW:
            if detect_lane_end(mask) and (now > uturn_cooldown_until):
                print("🛑 Horizontal line (lane end) detected! Starting U-Turn sequence...")
                current_state = STATE_STOP
                state_start_time = now
            elif left_fit is None:
                print("🛑 Lane boundaries disappeared! Stopping.")
                current_state = STATE_STOP
                state_start_time = now
            elif (stop_sign_inside or stop_sign_outside) and (min_dist <= 50.0) and (now > ignore_stop_until):
                if stop_sign_inside:
                    print(f"🛑 Stop sign detected inside lane boundaries at {min_dist:.1f}cm! Stopping for 3 seconds...")
                else:
                    print(f"🛑 Stop sign detected outside lane boundaries at {min_dist:.1f}cm! Stopping for 3 seconds...")
                current_state = STATE_STOP_SIGN_WAIT
                state_start_time = now
                stop_until = now + 3.0
                ignore_stop_until = now + 8.0
                skip_detection_until = now + 8.0
                print("⚡ Skipping object detection for 8 seconds (during stop + resuming) to avoid duplicate detection.")
            elif red_light_outside and (now > ignore_stop_until):
                print("🛑 Traffic red light detected outside lane boundaries! Stopping for 3 seconds...")
                current_state = STATE_STOP_SIGN_WAIT
                state_start_time = now
                stop_until = now + 3.0
                ignore_stop_until = now + 8.0
            elif has_obstacle:
                # Determine lane occupancies dynamically
                current_lane_blocked = False
                other_lane_blocked = False
                
                if current_lane == 'RIGHT':
                    current_lane_blocked = right_lane_has_obstacle
                    other_lane_blocked = left_lane_has_obstacle
                else:
                    current_lane_blocked = left_lane_has_obstacle
                    other_lane_blocked = right_lane_has_obstacle
                
                if current_lane_blocked:
                    if other_lane_blocked:
                        # Both lanes blocked: slow down from 60 and stop at 30
                        if min_dist <= 30.0:
                            print(f"🛑 Both lanes blocked and distance <= 30cm ({min_dist:.1f}cm)! Stopping.")
                            current_state = STATE_STOP
                            state_start_time = now
                        elif 30.0 < min_dist <= 60.0:
                            print(f"⚠️ Both lanes blocked and distance <= 60cm ({min_dist:.1f}cm). Slowing down.")
                            both_blocked_slow_down = True
                    else:
                        # Current lane blocked, other empty: change lane immediately
                        if current_lane == 'RIGHT':
                            print(f"⬅️ Obstacle in RIGHT lane. LEFT lane is empty. Changing lane LEFT!")
                            current_state = STATE_LANE_CHANGE_LEFT
                        else:
                            print(f"➡️ Obstacle in LEFT lane. RIGHT lane is empty. Changing lane RIGHT!")
                            current_state = STATE_LANE_CHANGE_RIGHT
                        state_start_time = now

        elif current_state == STATE_LANE_CHANGE_LEFT:
            if now - state_start_time >= LANE_CHANGE_DURATION:
                current_state = STATE_PAUSE_LEFT
                state_start_time = now
                print(f"🔄 Lane change LEFT complete. Entering {STATE_PAUSE_LEFT} to find lane boundaries.")
                
        elif current_state == STATE_LANE_CHANGE_RIGHT:
            if now - state_start_time >= LANE_CHANGE_DURATION:
                current_state = STATE_PAUSE_RIGHT
                state_start_time = now
                print(f"🔄 Lane change RIGHT complete. Entering {STATE_PAUSE_RIGHT} to find lane boundaries.")
                
        elif current_state == STATE_PAUSE_LEFT:
            if now - state_start_time >= PAUSE_DURATION:
                current_state = STATE_ALIGN_RIGHT
                state_start_time = now
                print(f"🛑 Pause complete. Counter-steering RIGHT ({STATE_ALIGN_RIGHT}) to align...")
                
        elif current_state == STATE_PAUSE_RIGHT:
            if now - state_start_time >= PAUSE_DURATION:
                current_state = STATE_ALIGN_LEFT
                state_start_time = now
                print(f"🛑 Pause complete. Counter-steering LEFT ({STATE_ALIGN_LEFT}) to align...")
                
        elif current_state in (STATE_ALIGN_LEFT, STATE_ALIGN_RIGHT):
            if left_valid and right_valid:
                if current_state == STATE_ALIGN_RIGHT:
                    current_lane = 'LEFT'
                elif current_state == STATE_ALIGN_LEFT:
                    current_lane = 'RIGHT'
                current_state = STATE_LANE_FOLLOW
                state_start_time = now
                print(f"🎯 Both boundaries of the new {current_lane} lane detected! Resuming lane follow.")
            elif now - state_start_time >= ALIGN_DURATION:
                current_state = STATE_STOP
                state_start_time = now
                print(f"⚠️ Alignment timeout ({ALIGN_DURATION}s) reached! Stopping.")
                
        elif current_state == STATE_STOP_SIGN_WAIT:
            if now >= stop_until:
                current_state = STATE_LANE_FOLLOW
                print("🏁 Stop complete. Resuming lane following.")

        elif current_state == STATE_UTURN_STOP1:
            if now - state_start_time >= 0.3:
                current_state = STATE_UTURN_FORWARD
                state_start_time = now
                print("➡️ UTURN: Moving forward...")

        elif current_state == STATE_UTURN_FORWARD:
            if now - state_start_time >= 0.7:
                current_state = STATE_UTURN_STOP2
                state_start_time = now
                print("🛑 UTURN: Stopping before steering...")

        elif current_state == STATE_UTURN_STOP2:
            if now - state_start_time >= 0.3:
                current_state = STATE_UTURN_STEER
                state_start_time = now
                print("🔄 UTURN: Steering left to turn around...")

        elif current_state == STATE_UTURN_STEER:
            if now - state_start_time >= UTURN_MIN_STEER_DURATION:
                if left_valid and right_valid:
                    current_state = STATE_UTURN_STOP_FINAL
                    state_start_time = now
                    print("🎯 UTURN: Lane boundaries detected! Stopping...")
                elif now - state_start_time >= UTURN_TIMEOUT:
                    current_state = STATE_STOP
                    state_start_time = now
                    print("⚠️ UTURN: Steering timeout! Stopping.")

        elif current_state == STATE_UTURN_STOP_FINAL:
            if now - state_start_time >= 1.0:
                current_lane = 'RIGHT'
                current_state = STATE_LANE_FOLLOW
                uturn_cooldown_until = now + 8.0
                state_start_time = now
                print("🏁 UTURN complete. Resuming lane following.")

        # ========================================================
        # DRAW AND CALCULATE STEERING / MOTOR SPEED
        # ========================================================
        if left_fit is not None:
            # Draw detected lane overlay
            result = draw_lane(frame, mask, left_fit, right_fit, Minv)
            # Draw Pure Pursuit path
            draw_reference_path(result, left_fit, right_fit, Minv, frame.shape)
        else:
            result = frame.copy()

        left_pwm, right_pwm = 0, 0
        command = "STOPPED"

        if current_state == STATE_STOP or current_state == STATE_STOP_SIGN_WAIT:
            left_pwm, right_pwm = 0, 0
            command = "STOPPED"
        elif current_state in (STATE_PAUSE_LEFT, STATE_PAUSE_RIGHT):
            left_pwm, right_pwm = 0, 0
            command = "PAUSED"
        elif current_state == STATE_ALIGN_LEFT:
            command = "ALIGN_LEFT"
            left_pwm = 0
            right_pwm = 215
        elif current_state == STATE_ALIGN_RIGHT:
            command = "ALIGN_RIGHT"
            left_pwm = 215
            right_pwm = 0
        elif current_state in (STATE_UTURN_STOP1, STATE_UTURN_STOP2, STATE_UTURN_STOP_FINAL):
            left_pwm, right_pwm = 0, 0
            command = "UTURN_STOP"
        elif current_state == STATE_UTURN_FORWARD:
            left_pwm, right_pwm = 200, 200
            command = "UTURN_FORWARD"
        elif current_state == STATE_UTURN_STEER:
            left_pwm, right_pwm = 0, 215
            command = "UTURN_STEER"
        elif left_fit is not None:
            error, lane_center, car_center = compute_steering(left_fit, right_fit, frame.shape)
            
            base_speed = 135
            if current_state == STATE_LANE_FOLLOW:
                if yellow_detected_in_lane or both_blocked_slow_down:
                    base_speed = 100
                    command = get_command(error) + " (SLOW)"
                else:
                    base_speed = 135
                    command = get_command(error)
                left_pwm, right_pwm = compute_pwm(error, base_speed=base_speed)
            elif current_state == STATE_LANE_CHANGE_LEFT:
                command = "LANE_CHANGE_LEFT"
                left_pwm, right_pwm = compute_pwm(error, base_speed=125)
            elif current_state == STATE_LANE_CHANGE_RIGHT:
                command = "LANE_CHANGE_RIGHT"
                left_pwm, right_pwm = compute_pwm(error, base_speed=125)
            else:
                command = get_command(error)
                left_pwm, right_pwm = compute_pwm(error, base_speed=135)
        else:
            command = "NO LANE"
            left_pwm, right_pwm = 0, 0

        # Clip and convert speeds
        left_pwm = int(np.clip(left_pwm, 0, 255))
        right_pwm = int(np.clip(right_pwm, 0, 255))

        # OSD text
        cv2.putText(result, f"State: {current_state} | Lane: {current_lane}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(result, f"Speed: L{left_pwm} R{right_pwm} | Cmd: {command}", (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(result, f"Radar: {min_dist:.0f}", (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        left_color = (0, 255, 0) if left_lane_status == "EMPTY" else ((0, 0, 255) if left_lane_status == "OBSTACLE" else (0, 255, 255))
        right_color = (0, 255, 0) if right_lane_status == "EMPTY" else ((0, 0, 255) if right_lane_status == "OBSTACLE" else (0, 255, 255))
        cv2.putText(result, f"LEFT Lane: {left_lane_status}", (20, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, left_color, 2)
        cv2.putText(result, f"RIGHT Lane: {right_lane_status}", (20, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, right_color, 2)

        # Send motor commands to Arduino (rate limited to 20Hz / 50ms)
        current_time = time.time()
        if current_time - last_send_time >= 0.05:
            command_str = f"L{left_pwm:03d}R{right_pwm:03d}\n"
            safe_serial_write(command_str.encode())
            last_send_time = current_time
            print(f"SENT [{current_state}]: L{left_pwm:03d}R{right_pwm:03d} | Radar: {min_dist:.0f}")

        cv2.imshow("Lane Following & Obstacle Avoidance", result)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC to exit
            break
        elif key == ord('l') or key == ord('L'):
            if current_state == STATE_LANE_FOLLOW:
                current_state = STATE_LANE_CHANGE_LEFT
                state_start_time = time.time()
                print("⬅️ Manual lane change LEFT triggered")
        elif key == ord('r') or key == ord('R'):
            if current_state == STATE_LANE_FOLLOW:
                current_state = STATE_LANE_CHANGE_RIGHT
                state_start_time = time.time()
                print("➡️ Manual lane change RIGHT triggered")
        elif key == ord('f') or key == ord('F'):
            current_state = STATE_LANE_FOLLOW
            print("🔄 Reset to STATE_LANE_FOLLOW state")
        elif key == ord('u') or key == ord('U'):
            if current_state == STATE_LANE_FOLLOW:
                current_state = STATE_UTURN_STOP1
                state_start_time = time.time()
                print("🔄 Manual U-turn triggered")

except KeyboardInterrupt:
    print("\n🛑 Stopping Car...")
    command_str = f"L000R000\n"
    safe_serial_write(command_str.encode())
    time.sleep(0.5)

finally:
    # Final cleanup
    try:
        command_str = f"L000R000\n"
        safe_serial_write(command_str.encode())
    except:
        pass
    time.sleep(0.2)
    cv2.destroyAllWindows()
    stop_camera()
    ser.close()
    print("✅ Stopped safely")
    

