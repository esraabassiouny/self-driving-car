from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import time


ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0)
time.sleep(3)  # wait for Arduino to initialize


# ---------------------------
# 1. Perspective Transform
# ---------------------------
def perspective_transform(img):
    h, w = img.shape[:2]

    # src = np.float32([
    #     [w*0.38, h*0.55],   # top-left
    #     [w*0.73, h*0.55],   # top-right
    #     [w*0.95, h*0.95],   # bottom-right
    #     [w*0.1, h*0.95]    # bottom-left
    # ])

    # dst = np.float32([
    #     [w*0.20, 0],
    #     [w*0.80, 0],
    #     [w*0.80, h],
    #     [w*0.20, h]
    # ])
    src = np.float32([
        [w*0.42, h*0.68],   # top-left
        [w*0.67, h*0.68],   # top-right
        [w*0.76, h*0.98],   # bottom-right
        [w*0.33, h*0.98]    # bottom-left
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
    return warped, Minv


# ---------------------------
# 2. White Mask
# ---------------------------
def threshold_white(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Lowered V threshold: catches lines in shadows too
    lower_white = np.array([0,   0, 160])
    upper_white = np.array([180, 55, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)

    # Close small gaps caused by dirt or worn paint
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask

# ---------------------------
# 3. Sliding Window
# ---------------------------
def sliding_window(binary_warped):
    h, w = binary_warped.shape
    histogram = np.sum(binary_warped[int(h * 0.70):, :], axis=0)

    midpoint = w // 2
    leftx_base  = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # If one side has zero signal, fall back to 25%/75% of width
    if histogram[:midpoint].max() == 0:
        leftx_base = w // 4
    if histogram[midpoint:].max() == 0:
        rightx_base = w * 3 // 4

    nwindows      = 9
    window_height = h // nwindows
    margin        = 100
    minpix        = 40

    nonzero  = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    leftx_current  = leftx_base
    rightx_current = rightx_base

    left_lane_inds  = []
    right_lane_inds = []

    for window in range(nwindows):
        win_y_low  = h - (window + 1) * window_height
        win_y_high = h - window       * window_height

        good_left = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                     (nonzerox >= leftx_current  - margin) &
                     (nonzerox <  leftx_current  + margin)).nonzero()[0]

        good_right = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                      (nonzerox >= rightx_current - margin) &
                      (nonzerox <  rightx_current + margin)).nonzero()[0]

        left_lane_inds.append(good_left)
        right_lane_inds.append(good_right)

        if len(good_left)  > minpix:
            leftx_current  = int(np.mean(nonzerox[good_left]))
        if len(good_right) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right]))

    left_lane_inds  = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    return (nonzerox[left_lane_inds],  nonzeroy[left_lane_inds],
            nonzerox[right_lane_inds], nonzeroy[right_lane_inds])

def compute_pwm(error, base_speed=135, max_adjust=130):

    # More sensitive steering
    error = np.clip(error, -80, 80)

    adjust = (error / 80) * max_adjust
    left_pwm = base_speed + adjust
    right_pwm = base_speed - adjust

    left_pwm = int(np.clip(left_pwm, 0, 255))
    right_pwm = int(np.clip(right_pwm, 0, 255))

    return left_pwm, right_pwm

# ---------------------------
# 4. Fit Curves
# ---------------------------
_last_left_fit  = None
_last_right_fit = None
_known_lane_w   = None

def fit_polynomial(binary_warped):
    global _last_left_fit, _last_right_fit, _known_lane_w

    leftx, lefty, rightx, righty = sliding_window(binary_warped)

    left_ok  = len(leftx)  >= 50
    right_ok = len(rightx) >= 50

    if left_ok and right_ok:
        left_fit  = np.polyfit(lefty,  leftx,  2)
        right_fit = np.polyfit(righty, rightx, 2)
        # Learn the real lane width from the bottom of the image
        y_bot = binary_warped.shape[0] - 1
        _known_lane_w   = abs(np.polyval(right_fit, y_bot) - np.polyval(left_fit, y_bot))
        _last_left_fit  = left_fit
        _last_right_fit = right_fit
        return left_fit, right_fit

    if left_ok and not right_ok:
        left_fit = np.polyfit(lefty, leftx, 2)
        if _known_lane_w:
            right_fit = left_fit.copy()
            right_fit[2] += _known_lane_w
        elif _last_right_fit is not None:
            right_fit = _last_right_fit
        else:
            right_fit = left_fit.copy()
            right_fit[2] += 300
        _last_left_fit  = left_fit
        _last_right_fit = right_fit
        return left_fit, right_fit

    if right_ok and not left_ok:
        right_fit = np.polyfit(righty, rightx, 2)
        if _known_lane_w:
            left_fit = right_fit.copy()
            left_fit[2] -= _known_lane_w
        elif _last_left_fit is not None:
            left_fit = _last_left_fit
        else:
            left_fit = right_fit.copy()
            left_fit[2] -= 300
        _last_left_fit  = left_fit
        _last_right_fit = right_fit
        return left_fit, right_fit

    # No lanes at all — reuse last known fits
    if _last_left_fit is not None and _last_right_fit is not None:
        return _last_left_fit, _last_right_fit

    return None, None


# ---------------------------
# 5. Steering Logic
# ---------------------------
prev_error = 0

def compute_steering(left_fit, right_fit, shape):

    global prev_error

    h, w = shape[:2]

    y = int(h * 0.75)
    left_x  = np.polyval(left_fit,  y)
    right_x = np.polyval(right_fit, y)

    lane_center = (left_x + right_x) / 2

    # Fixed: use actual frame centre, not hardcoded 400
    car_center = w / 2.0

    error = lane_center - car_center

    # smoothing
    error = 0.7 * prev_error + 0.3 * error
    prev_error = error

    return error, lane_center, car_center


def get_command(error, threshold=12):
    if error > threshold:
        return "RIGHT"
    elif error < -threshold:
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
# 2.5. Lane End (U-Turn) Detection
# ---------------------------
# def detect_lane_end(mask):
#     h, w = mask.shape
#     row_sums = np.sum(mask > 0, axis=1)
    
#     # A horizontal strip will have many white pixels in a single row
#     threshold = int(w * 0.35)  # e.g., 280 pixels if w=800
    
#     # Check the middle-to-lower portion of the frame
#     search_region = row_sums[int(h*0.3):int(h*0.95)]
#     trigger_rows = np.sum(search_region > threshold)
    
#     # If 5 or more rows exceed the threshold, we detect the lane end
#     return trigger_rows >= 5
# ---------------------------
# MAIN (Picamera2)
# ---------------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (800, 600)})
picam2.configure(config)
picam2.start()

print("🚗 Lane following started (CTRL+C to stop)")

frame_count = 0
last_send_time = 0
last_uturn_time = 0
try:
    while True:
        # Read all available responses from Arduino (non-blocking)
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
        mask[:, :int(w*0.22)] = 0
		# Check for U-turn / Lane End
        current_time = time.time()
        #if detect_lane_end(mask) and (current_time - last_uturn_time > 10.0):
         #   print("🚨 UTURN DETECTED - Sending command to Arduino...")
          #  ser.write(b"UTURN_LEFT\n")
           # ser.flush()
            
            # Wait until Arduino finishes the U-Turn
            #while True:
             #   if ser.in_waiting > 0:
              #      try:
               #         msg = ser.readline().decode('utf-8', errors='ignore').strip()
                #        print(f"📟 Arduino: {msg}")
                 #       if msg == "END_U_TURN":
                  #          break
                   # except Exception as e:
                    #    pass
                #time.sleep(0.01)  # Avoid 100% CPU usage
            
           # last_uturn_time = time.time()
            #last_send_time = time.time()
            #continue
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

            # Rate limit sending to Arduino (max 20 Hz / every 50ms)
            current_time = time.time()
            if current_time - last_send_time >= 0.05:
                command = f"L{left_pwm:03d}R{right_pwm:03d}\n"
                
                #ser.write(command.encode())          
                ser.flush()
                last_send_time = current_time
            else:
                command = f"L{left_pwm:03d}R{right_pwm:03d} (Skipped - Rate Limited)"

            cv2.putText(result, f"Error: {int(error)}", (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        else:
            result = frame
            command = "NO LANE"

        print(command)

        cv2.imshow("Lane Following", result)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) == 27:
            break

except KeyboardInterrupt:
	ser.write(b"L000R000\n")
	time.sleep(0.5)
	print("\n🛑 Stopped")

cv2.destroyAllWindows()
picam2.stop()
