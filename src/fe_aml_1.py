from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import time
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0)
time.sleep(3)  # wait for Arduino to initialize



# ---------------------------
# 1. Perspective Transform
# ---------------------------
def perspective_transform(img):
    h, w = img.shape[:2]
    # src = np.float32([
    #     [w*0.42, h*0.68],   # top-left
    #     [w*0.67, h*0.68],   # top-right
    #     [w*0.76, h*0.98],   # bottom-right
    #     [w*0.33, h*0.98]    # bottom-left
    # ])
 
    
    # dst = np.float32([
    #     [w*0.12, 0],     # top-left
    #     [w*0.88, 0],     # top-right
    #     [w*0.88, h],     # bottom-right
    #     [w*0.12, h]      # bottom-left
    # ])

    # wider from left >> worked better 
	src = np.float32([
        [w*0.37, h*0.68],   # top-left
        [w*0.67, h*0.68],   # top-right
        [w*0.76, h*0.98],   # bottom-right
        [w*0.28, h*0.98]    # bottom-left
    ])
 
    
    dst = np.float32([
        [w*0.12, 0],     # top-left
        [w*0.88, 0],     # top-right
        [w*0.88, h],     # bottom-right
        [w*0.12, h]      # bottom-left
    ])

    #shorter >> worked best

    #     src = np.float32([
    #     [w*0.27, h*0.78],   # top-left
    #     [w*0.80, h*0.78],   # top-right
    #     [w*0.83, h*0.98],   # bottom-right
    #     [w*0.23, h*0.98]    # bottom-left
    # ])
 
    
    # dst = np.float32([
    #     [w*0.12, 0],     # top-left
    #     [w*0.88, 0],     # top-right
    #     [w*0.88, h],     # bottom-right
    #     [w*0.12, h]      # bottom-left
    # ])
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
        return None, None

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
    return left_fit, right_fit


# ---------------------------
# 5. Steering Logic
# ---------------------------
prev_error = 0
def compute_steering(left_fit, right_fit, shape):

    global prev_error

    h, w = shape[:2]

    # Evaluate further up the image to look ahead (anticipate curves)
    ys = [h * 0.55, h * 0.65, h * 0.75]
    lane_centers = []

    for y in ys:
        lx = np.polyval(left_fit, y)
        rx = np.polyval(right_fit, y)
        lane_centers.append((lx + rx) / 2)

    lane_center = np.mean(lane_centers)

    # tune this manually later
    car_center = w/2

    error = lane_center - car_center

    # smoothing
    error = 0.3 * prev_error + 0.7 * error
    prev_error = error

    return error, lane_center, car_center


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
        while ser.in_waiting > 0:
            try:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    print(f"📟 Arduino: {response}")
            except Exception as e:
                break

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

        left_fit, right_fit = fit_polynomial(mask)

        if left_fit is not None:
            result = draw_lane(frame, mask, left_fit, right_fit, Minv)
            
            error, lane_center, car_center = compute_steering(
                left_fit,
               right_fit,
                frame.shape
            )
            # GREEN = detected lane center
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
            command = get_command(error)
            
            # display info
            cv2.putText(result, f"{command}", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                        
            left_pwm, right_pwm = compute_pwm(error)

            current_time = time.time()
            # Rate limit sending normal motor commands to Arduino (max 20 Hz / every 50ms)
            if current_time - last_send_time >= 0.05:
                command = f"L{left_pwm:03d}R{right_pwm:03d}\n"
                ser.write(command.encode()) 
                ser.flush()
                last_send_time = current_time
                print("SENT:", command.strip())

            cv2.putText(result, f"Error: {int(error)}", (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        else:
            result = frame
            command = "NO LANE"
            # Send stop command if lane is lost (rate limited to 20Hz)
            current_time = time.time()
            if current_time - last_send_time >= 0.05:
                stop_cmd = "L000R000\n"
                ser.write(stop_cmd.encode())
                ser.flush()
                last_send_time = current_time
                print("SENT (NO LANE - STOP):", stop_cmd.strip())


        cv2.imshow("Lane Following", result)
        #cv2.imshow("Mask", mask)

        if cv2.waitKey(1) == 27:
            break
            
        #time.sleep(4)    

except KeyboardInterrupt:

    print("\n🛑 Stopping Car...")

    ser.write(b"L000R000\n")

    time.sleep(0.5)

    print("✅ Car Stopped")

# Final cleanup
ser.write(b"L000R000\n")

time.sleep(0.2)

cv2.destroyAllWindows()

picam2.stop()

ser.close()
