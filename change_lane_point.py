from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import time
#import matplotlib
#matplotlib.use('Agg')  # Use headless backend to prevent Qt/X11 GUI errors
#import matplotlib.pyplot as plt

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0)
time.sleep(3)  # wait for Arduino to initialize


# Helper to write to serial with auto-reconnect support
def safe_serial_write(command_str):
    global ser
    try:
        ser.write(command_str.encode())
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
            ser.write(command_str.encode())
            ser.flush()

        except Exception as reconnect_error:
            print(f"❌ Reconnection failed: {reconnect_error}")
# ---------------------------
# 1. Perspective Transform
# ---------------------------
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
    #cv2.imshow("Warped", warped)
    return warped, Minv

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
    if current_state == STATE_STEER_RIGHT:
        # Find all peaks in the histogram above a threshold
        peaks = []
        win_size = 40
        # Iterate from left to right (leaving some margin from borders)
        for i in range(win_size, len(histogram) - win_size):
            # Check if this point is a local maximum
            if histogram[i] > 1000 and histogram[i] == np.max(histogram[i - win_size : i + win_size + 1]):
                # Ensure it is sufficiently far from other peaks to avoid double detection of same line
                if not peaks or all(abs(i - p) > 200 for p in peaks):
                    peaks.append(i)
        peaks.sort()
        if len(peaks) >= 2:
            leftx_base = peaks[0]
            rightx_base = peaks[1]
        else:
            leftx_base = np.argmax(histogram[:midpoint])
            rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    else:
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
def compute_pwm(error, base_speed=80, max_adjust=130):
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
# 5. Steering Logic
# ---------------------------
STATE_LANE_FOLLOW = "LANE_FOLLOW"
STATE_REACH_P1 = "REACH_P1"
STATE_FORWARD = "FORWARD"
STATE_PAUSE = "PAUSE"
STATE_STEER_RIGHT = "STEER_RIGHT"
STATE_STOP = "STOP"

current_state = STATE_LANE_FOLLOW
state_start_time = 0
LANE_SHIFT = 150  # shift to the left for the new lane

# Static target points for the lane change test (configurable by user)
P1 = (120, 500)
P2 = (200, 400)

# Time durations to spend reaching each point (in seconds)
P1_DURATION = 1
FORWARD_DURATION = 0.8
PAUSE_DURATION = 0.2

# Speed values for STATE_STEER_RIGHT pivot turn (configurable by user)
STEER_RIGHT_LEFT_PWM = 200
STEER_RIGHT_RIGHT_PWM = 0

# Trajectory tracking history for plotting
time_history = []
p1_x_history = []
p1_y_history = []
p2_x_history = []
p2_y_history = []
error_history = []
state_history = []
test_start_time = None

prev_error = 0
def compute_steering(left_fit, right_fit, left_valid, right_valid, shape):
    global prev_error, current_state, state_start_time
    h, w = shape[:2]
    car_center = w / 2

    # Compute base center fit if lane fits are available
    if left_fit is not None and right_fit is not None:
        center_fit = (left_fit + right_fit) / 2.0
        y_lookahead = h * 0.75
        lane_center = np.polyval(center_fit, y_lookahead)
    else:
        lane_center = car_center  # fallback when lane is lost

    error = 0.0
    elapsed = 0.0
    if state_start_time > 0:
        elapsed = time.time() - state_start_time

    if current_state == STATE_LANE_FOLLOW:
        error = lane_center - car_center

    elif current_state == STATE_REACH_P1:
        # err1 = p1 - car_center (using X coordinate of static point P1)
        error = P1[0] - car_center
        # Transition condition: based on duration
        if elapsed >= P1_DURATION:
            current_state = STATE_FORWARD
            state_start_time = time.time()
            print(f"🎯 P1 duration reached ({P1_DURATION}s)! Transitioning to STATE_FORWARD.")

    elif current_state == STATE_FORWARD:
        # drive straight: steering error = 0.0
        error = 0.0
        # Transition condition: based on duration
        if elapsed >= FORWARD_DURATION:
            current_state = STATE_PAUSE
            state_start_time = time.time()
            print(f"⏩ Forward duration reached ({FORWARD_DURATION}s)! Transitioning to STATE_PAUSE.")

    elif current_state == STATE_PAUSE:
        # stop car: steering error = 0.0
        error = 0.0
        # Transition condition: based on duration
        if elapsed >= PAUSE_DURATION:
            current_state = STATE_STEER_RIGHT
            state_start_time = time.time()
            print(f"🛑 Pause duration reached ({PAUSE_DURATION}s)! Transitioning to STATE_STEER_RIGHT.")

    elif current_state == STATE_STEER_RIGHT:
        # drive right to align with new lane
        error = 40.0
        # Transition condition: both boundaries of target lane must be detected
        if left_valid and right_valid:
            current_state = STATE_LANE_FOLLOW
            state_start_time = time.time()
            print("🎯 Both boundaries of the new lane detected! Resuming STATE_LANE_FOLLOW.")

    elif current_state == STATE_STOP:
        error = 0.0

    # smoothing
    error = 0.3 * prev_error + 0.7 * error
    prev_error = error
    return error, lane_center, car_center, P1[0], P2[0], P1[1], P2[1]

def get_command(error, threshold=0):
    if error > threshold:
        return "RIGHT"
    elif error < threshold:
        return "LEFT"
    else:
        return "STRAIGHT"

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
config = picam2.create_preview_configuration(main={"size": (800, 600)})
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
        #             print(f"📟 Arduino: {response}")
        #     except Exception as e:
        #         break

        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame_count += 1

        # Process only every 3rd frame
        #if frame_count % 3 != 0:
         #   continue

        warped, Minv = perspective_transform(frame)
        mask = threshold_white(warped)
        w = mask.shape[1]
        #mask[:, :int(w*0.22)] = 0
        cv2.imshow("Mask", mask)
        
        lane_end_detected = detect_lane_end(mask)
        current_time = time.time()
        
        w = mask.shape[1]
        #mask[:, :int(w*0.22)] = 0

        left_fit, right_fit, left_valid, right_valid = fit_polynomial(mask)

        if left_fit is not None or current_state in (STATE_REACH_P1, STATE_FORWARD, STATE_PAUSE, STATE_STEER_RIGHT):
            if left_fit is not None:
                result = draw_lane(frame, mask, left_fit, right_fit, Minv)
            else:
                result = frame.copy()
            
            error, lane_center, car_center, p1_x, p2_x, p1_y, p2_y = compute_steering(
                left_fit,
                right_fit,
                left_valid,
                right_valid,
                frame.shape
            )
            
            # Record trajectory for plotting
            if current_state in (STATE_REACH_P1, STATE_FORWARD, STATE_PAUSE, STATE_STEER_RIGHT):
                if test_start_time is None:
                    test_start_time = time.time()
                t_elapsed = time.time() - test_start_time
                time_history.append(t_elapsed)
                p1_x_history.append(p1_x)
                p1_y_history.append(p1_y)
                p2_x_history.append(p2_x)
                p2_y_history.append(p2_y)
                error_history.append(error)
                state_history.append(current_state)

            # GREEN = detected lane center
            if left_fit is not None:
                cv2.line(
                    result,
                    (int(lane_center), 0),
                    (int(lane_center), result.shape[0]),
                    (0, 255, 0),
                    3
                )

            # RED = desired car center
            cv2.line(
                result,
                (int(car_center), 0),
                (int(car_center), result.shape[0]),
                (0, 0, 255),
                3
            )
            
            # Draw p1 and p2 directly on result image using static screen coordinates
            cv2.circle(result, (int(p1_x), int(p1_y)), 8, (255, 0, 255), -1)  # Magenta circle for p1
            cv2.putText(result, f"p1({int(p1_x)}, {int(p1_y)})", (int(p1_x) + 10, int(p1_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
            cv2.circle(result, (int(p2_x), int(p2_y)), 8, (0, 255, 255), -1)  # Cyan circle for p2
            cv2.putText(result, f"p2({int(p2_x)}, {int(p2_y)})", (int(p2_x) + 10, int(p2_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            command = get_command(error)
            
            # display info
            cv2.putText(result, f"State: {current_state}", (50, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(result, f"Cmd: {command}", (50, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(result, f"Error: {int(error)}", (50, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        
            if current_state == STATE_PAUSE:
                left_pwm = 0
                right_pwm = 0
            elif current_state == STATE_STEER_RIGHT:
                left_pwm = STEER_RIGHT_LEFT_PWM
                right_pwm = STEER_RIGHT_RIGHT_PWM
            else:
                left_pwm, right_pwm = compute_pwm(error)
            current_time = time.time()
            # Rate limit sending normal motor commands to Arduino (max 20 Hz / every 50ms)
            if current_time - last_send_time >= 0.05:
                command_str = f"L{left_pwm:03d}R{right_pwm:03d}\n"
                safe_serial_write(command_str)
                last_send_time = current_time
                print("SENT:", command_str.strip())

        else:
            result = frame
            command = "NO LANE"
            
            # Send stop command if lane is lost (rate limited to 20Hz)
            current_time = time.time()
            if current_time - last_send_time >= 0.05:
                stop_cmd = "L000R000\n"
                safe_serial_write(stop_cmd)
                last_send_time = current_time
                print("SENT (NO LANE - STOP):", stop_cmd.strip())

        cv2.imshow("Lane Following", result)

        # Check if we transitioned to STOP state
        if current_state == STATE_STOP:
            print("🛑 State STOP reached. Stopping car and exiting...")
            stop_cmd = "L000R000\n"
            safe_serial_write(stop_cmd)
            time.sleep(0.5)
            break

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        elif key == ord('t') or key == ord('T'):
            if current_state == STATE_LANE_FOLLOW:
                current_state = STATE_REACH_P1
                state_start_time = time.time()
                test_start_time = time.time()
                print("🏁 Triggered: Steering to reach p1 (Y=500)")
            
        #time.sleep(4)    

except KeyboardInterrupt:
    print("\n🛑 Stopping Car...")
    stop_cmd = "L000R000\n"
    safe_serial_write(stop_cmd)
    time.sleep(0.5)
    print("✅ Car Stopped")

# Final cleanup
stop_cmd = "L000R000\n"
safe_serial_write(stop_cmd)
time.sleep(0.2)
cv2.destroyAllWindows()
picam2.stop()
ser.close()

# ---------------------------
# Plot and Save Trajectory Data
# ---------------------------
if len(time_history) > 0:
    print("\n📊 Generating lane change plots...")
    try:
        plt.figure(figsize=(12, 6))

        # Subplot 1: Spatial paths of p1 and p2 in camera image space
        plt.subplot(1, 2, 1)
        plt.plot(p1_x_history, p1_y_history, 'm.-', label='Target P1 (Magenta)')
        plt.plot(p2_x_history, p2_y_history, 'c.-', label='Target P2 (Cyan)')
        plt.axvline(x=400, color='r', linestyle='--', label='Car Center (400)')
        plt.xlim(0, 800)
        plt.ylim(600, 0)  # Invert Y axis to match image coordinates
        plt.xlabel('X (pixels)')
        plt.ylabel('Y (pixels)')
        plt.title('Spatial Target Trajectory in Image Frame')
        plt.legend()
        plt.grid(True)

        # Subplot 2: Steering Error over time
        plt.subplot(1, 2, 2)
        plt.plot(time_history, error_history, 'g-', label='Steering Error (deg)')
        plt.axhline(y=0, color='k', linestyle='--')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Error (degrees)')
        plt.title('Steering Error over Time')
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plot_filename = "lane_change_test_plot.png"
        #plt.savefig(plot_filename)
        #print(f"✅ Plot saved successfully to: {plot_filename}")
    except Exception as e:
        print(f"⚠️ Could not display/save plot: {e}")
