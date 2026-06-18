from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import time
#import matplotlib.pyplot as plt

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0)
time.sleep(3)  # wait for Arduino to initialize



# ---------------------------
# 1. Perspective Transform
# ---------------------------
def safe_serial_write(data):
    global ser
    if isinstance(data, str):
        data = data.encode()
    try:
        ser.write(data)
        ser.flush()
    except (serial.SerialException, OSError) as e:
        print(f"⚠️ Serial write failed: {e}. Attempting to reconnect...")

        try:
            ser.close()
        except:
            pass

        try:
            time.sleep(1.0)
            ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0)
            time.sleep(1.0)
            print("✅ Serial reconnected successfully!")

            # Resend command after reconnect
            ser.write(data)
            ser.flush()

        except Exception as reconnect_error:
            print(f"❌ Reconnection failed: {reconnect_error}")
            
def perspective_transform(img):
    h, w = img.shape[:2]
    #     src = np.float32([
    #     [w*0.37, h*0.68],   # top-left
    #     [w*0.67, h*0.68],   # top-right
    #     [w*0.76, h*0.98],   # bottom-right
    #     [w*0.28, h*0.98]    # bottom-left
    # ])
 
    
    # dst = np.float32([
    #     [w*0.12, 0],     # top-left
    #     [w*0.88, 0],     # top-right
    #     [w*0.88, h],     # bottom-right
    #     [w*0.12, h]      # bottom-left
    # ])
    src = np.float32([
        [w * 0.27, h * 0.78],   # top-left
        [w * 0.80, h * 0.78],   # top-right
        [w * 0.83, h * 0.98],   # bottom-right
        [w * 0.23, h * 0.98]    # bottom-left
    ])
 
    
    dst = np.float32([
        [w * 0.12, 0],     # top-left
        [w * 0.88, 0],     # top-right
        [w * 0.88, h],     # bottom-right
        [w * 0.12, h]      # bottom-left
    ])
    debug = img.copy()

    pts = np.array(src, np.int32)

    cv2.polylines(debug, [pts], True, (0,255,0), 3)

    cv2.imshow("ROI", debug)
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = np.linalg.inv(M)

    warped = cv2.warpPerspective(img, M, (w, h))
    #cv2.imshow(Warped, warped)
    return warped, Minv


# ---------------------------
# 2. White Mask
# ---------------------------
def threshold_white(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #cv2.imshow(hsv, hsv)

    lower_white = np.array([0, 0, 225])
    upper_white = np.array([180, 40, 255])

    return cv2.inRange(hsv, lower_white, upper_white)


# ---------------------------
# 3. Sliding Window
# ---------------------------
def sliding_window(binary_warped):

    histogram = np.sum(
        binary_warped[int(binary_warped.shape[0] * 0.72):, :],
        axis=0
    )

    #print(f"histogram {histogram}")

    midpoint = histogram.shape[0] // 2

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
        win_y_low = binary_warped.shape[0] - (window+1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height

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

    adjust = (error / 80.0) * max_adjust # [-130-130]
    
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
            if mean_left < w / 2:
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
STATE_STOP = 'STOP'

current_state = STATE_LANE_FOLLOW
state_start_time = 0

PAUSE_DURATION = 0.2     # Pause duration in seconds to stabilize camera before steering to find boundaries

LANE_WIDTH = 620          # Shift offset in pixels for lane change (reduced to stay on single-lane mat)
MERGE_DISTANCE = 150      # S-curve merge distance in pixels
LANE_CHANGE_DURATION = 2  # Duration of lane change in seconds
COOLDOWN_DURATION = 0.2     # Cooldown before allowing another lane change
LOOK_AHEAD_FACTOR = 0.7    # Look-ahead height factor (larger values look closer to the car, e.g. 0.80 - 0.85)
ALIGN_DURATION = 7      # Duration of counter-steering to straighten nose (in seconds)
ALIGN_STEER_OFFSET = 135   # Steering speed adjustment during alignment (steers in opposite direction)

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
# MAIN (Picamera2)
# ---------------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={'size': (800, 600)})
picam2.configure(config)
picam2.start()

print("🚗 Lane following started (CTRL+C to stop)")

last_send_time = 0
last_uturn_time = 0
frame_count = 0
try:
    while True:
        # Read all available responses from Arduino (non-blocking) to keep buffer clear
        # while ser.in_waiting > 0:
        #     try:
        #         response = ser.readline().decode('utf-8', errors='ignore').strip()
        #         if response:
        #             print(f"📟 Arduino {response}")
        #     except Exception as e:
        #         break

        if ser is not None and ser.is_open:
            try:
                while ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if "DIST:" in line:
                        dist_val = float(line.split(":")[1].strip())
                        min_dist = dist_val
            except Exception as e:
                pass
                
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        frame_count += 1

        # Process only every 3rd frame
        #if frame_count % 3 != 0:
         #   continue

        warped, Minv = perspective_transform(frame)
        mask = threshold_white(warped)
        
        warped_debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        w = mask.shape[1]
        #mask[:, int(w*0.22)] = 0
        cv2.imshow("Mask", mask)
        
        lane_end_detected = detect_lane_end(mask)
        current_time = time.time()
        
        # State machine transitions
        if current_state == STATE_LANE_CHANGE_LEFT:
            if current_time - state_start_time >= LANE_CHANGE_DURATION:
                current_state = STATE_PAUSE_LEFT
                state_start_time = current_time
                print(f"🔄 Lane change LEFT complete. Entering {STATE_PAUSE_LEFT} to find lane boundaries.")
        elif current_state == STATE_LANE_CHANGE_RIGHT:
            if current_time - state_start_time >= LANE_CHANGE_DURATION:
                current_state = STATE_PAUSE_RIGHT
                state_start_time = current_time
                print(f"🔄 Lane change RIGHT complete. Entering {STATE_PAUSE_RIGHT} to find lane boundaries.")
        elif current_state == STATE_PAUSE_LEFT:
            if current_time - state_start_time >= PAUSE_DURATION:
                current_state = STATE_ALIGN_LEFT
                state_start_time = current_time
                print(f"🛑 Pause complete. Counter-steering RIGHT ({STATE_ALIGN_LEFT}) to align...")
        elif current_state == STATE_PAUSE_RIGHT:
            if current_time - state_start_time >= PAUSE_DURATION:
                current_state = STATE_ALIGN_RIGHT
                state_start_time = current_time
                print(f"🛑 Pause complete. Counter-steering LEFT ({STATE_ALIGN_RIGHT}) to align...")
        elif current_state in (STATE_ALIGN_LEFT, STATE_ALIGN_RIGHT):
            # Check if both boundaries of the new lane are detected (using left_valid and right_valid from polyfit)
            if left_valid and right_valid:
                current_state = STATE_LANE_FOLLOW
                state_start_time = current_time
                print("🎯 Both boundaries of the new lane detected! Resuming lane follow.")
            elif current_time - state_start_time >= ALIGN_DURATION:
                current_state = STATE_STOP
                state_start_time = current_time
                print(f"⚠️ Alignment timeout ({ALIGN_DURATION}s) reached! Stopping.")

        w = mask.shape[1]
        
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

        if left_fit is not None:
            # Draw base lane overlay
            result = draw_lane(frame, mask, left_fit, right_fit, Minv)
            
            # Compute steering using Pure Pursuit path
            error, lane_center, car_center = compute_steering(
                left_fit,
                right_fit,
                frame.shape
            )
            warped_debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            center_fit = (left_fit + right_fit) / 2.0
            ploty = np.linspace(0, mask.shape[0]-1, 20)
            for y in ploty:
                x = get_ref_x(
                    y,
                    left_fit,
                    right_fit,
                    mask.shape[1],
                    mask.shape[0]
                )
                cv2.circle(
                warped_debug,
                (int(x), int(y)),
                5,
                (255, 255, 0),
                -1
                )
                cv2.imshow("Pure Pursuit Path", warped_debug)
            # Draw the S-curve reference path & look-ahead point
            draw_reference_path(result, left_fit, right_fit, Minv, frame.shape)
            
            # GREEN = target lane center at look-ahead distance (projected to unwarped space)
            y_look_ahead = frame.shape[0] * LOOK_AHEAD_FACTOR
            target_pts = np.array([[[lane_center, y_look_ahead]]], dtype=np.float32)
            target_unwarped = cv2.perspectiveTransform(target_pts, Minv)
            target_x_unwarped = int(target_unwarped[0][0][0])

            cv2.line(
                result,
                (target_x_unwarped, 0),
                (target_x_unwarped, result.shape[0]),
                (0, 255, 0),
                2
            )

            # RED = desired car center
            cv2.line(
                result,
                (int(car_center), 0),
                (int(car_center), result.shape[0]),
                (0, 0, 255),
                2
            )
            
            if current_state == STATE_STOP:
                command = "STOPPED"
                left_pwm, right_pwm = 0, 0
            elif current_state in (STATE_PAUSE_LEFT, STATE_PAUSE_RIGHT):
                command = "PAUSED"
                left_pwm, right_pwm = 0, 0
            elif current_state == STATE_ALIGN_LEFT:
                command = "ALIGN_RIGHT"
                left_pwm = 200
                right_pwm = 0
            elif current_state == STATE_ALIGN_RIGHT:
                command = "ALIGN_LEFT"
                left_pwm = 135 - ALIGN_STEER_OFFSET
                right_pwm = 135 + ALIGN_STEER_OFFSET
            else:
                command = get_command(error)
                left_pwm, right_pwm = compute_pwm(error)
        else:
            result = frame
            if current_state == STATE_STOP:
                command = "STOPPED"
                left_pwm, right_pwm = 0, 0
            elif current_state in (STATE_PAUSE_LEFT, STATE_PAUSE_RIGHT):
                command = "PAUSED"
                left_pwm, right_pwm = 0, 0
            elif current_state == STATE_ALIGN_LEFT:
                command = "ALIGN_RIGHT"
                left_pwm = 215
                right_pwm = 0
            elif current_state == STATE_ALIGN_RIGHT:
                command = "ALIGN_LEFT"
                left_pwm = 0
                right_pwm = 215
            else:
                command = "NO LANE"
                left_pwm, right_pwm = 0, 0
            
            if command == "NO LANE":
                cv2.putText(result, "NO LANE - STOP", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Apply limits and convert to int
        left_pwm = int(np.clip(left_pwm, 0, 255))
        right_pwm = int(np.clip(right_pwm, 0, 255))

        # Display current state and action info
        cv2.putText(result, f"State: {current_state}", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(result, f"Cmd: {command}", (50, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        if left_fit is not None:
            cv2.putText(result, f"Error: {int(error)}", (50, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Send motor commands to Arduino (rate limited to 20Hz)
        if current_time - last_send_time >= 0.05:
            command_str = f"L{left_pwm:03d}R{right_pwm:03d}\n"
            safe_serial_write(command_str.encode())
            last_send_time = current_time
            print(f"SENT [{current_state}]", command_str.strip())

        cv2.imshow("Lane Following", result)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC to exit
            break
        elif key == ord('r') or key == ord('R'):
            if current_state == STATE_LANE_FOLLOW:
                current_state = STATE_LANE_CHANGE_RIGHT
                state_start_time = current_time
                print("➡️ Initiated Lane Change RIGHT")
        elif key == ord('l') or key == ord('L'):
            if current_state == STATE_LANE_FOLLOW:
                current_state = STATE_LANE_CHANGE_LEFT
                state_start_time = current_time
                print("⬅️ Initiated Lane Change LEFT")
        elif key == ord('f') or key == ord('F'):
            current_state = STATE_LANE_FOLLOW
            print("🔄 Reset to LANE_FOLLOW state")
            
        #time.sleep(4)    

except KeyboardInterrupt:

    print("\n🛑 Stopping Car...")

    command_str = f"L000R000\n"
    safe_serial_write(command_str.encode())

    time.sleep(0.5)

    print("✅ Car Stopped")

# Final cleanup
command_str = f"L000R000\n"
safe_serial_write(command_str.encode())
time.sleep(0.2)

cv2.destroyAllWindows()

picam2.stop()

ser.close()
