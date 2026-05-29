from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import time


ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)  # wait for Arduino to initialize



# ---------------------------
# 1. Perspective Transform
# ---------------------------
def perspective_transform(img):
    h, w = img.shape[:2]
     

    src = np.float32([
        [w*0.17, h*0.78],   # top-left
        [w*0.90, h*0.78],   # top-right
        [w*0.93, h*0.98],   # bottom-right
        [w*0.13, h*0.98]    # bottom-left
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
    cv2.imshow("Warped", warped)
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
    #histogram = np.sum(binary_warped[binary_warped.shape[0]//2:, :], axis=0)
    histogram = np.sum(
        binary_warped[int(binary_warped.shape[0]*0.72):, :],
        axis=0
    )
    midpoint = histogram.shape[0] // 2
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    nwindows = 9
    window_height = binary_warped.shape[0] // nwindows
    # try 140  - 160
    margin = 140
    minpix = 50

    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    leftx_current = leftx_base
    rightx_current = rightx_base

    left_lane_inds = []
    right_lane_inds = []

    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height

        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        good_left = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                     (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]

        good_right = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                      (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        left_lane_inds.append(good_left)
        right_lane_inds.append(good_right)

        if len(good_left) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left]))

        if len(good_right) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right]))

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    return (nonzerox[left_lane_inds], nonzeroy[left_lane_inds],
            nonzerox[right_lane_inds], nonzeroy[right_lane_inds])

# try max_adjust 130 -140 
# base_speed 135
def compute_pwm(error, base_speed=120, max_adjust=130):

    # More sensitive steering
    error = np.clip(error, -80, 80)

    adjust = (error / 80) * max_adjust
    #adjust = error * 0.8
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

    if len(leftx) < 50 or len(rightx) < 50:
        return None, None

    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    return left_fit, right_fit


# ---------------------------
# 5. Steering Logic
# ---------------------------
prev_error = 0

def compute_steering(left_fit, right_fit, shape):

    global prev_error

    h, w = shape[:2]

    # y = h - 1
    # y = int(h * 0.75)

    # left_x = np.polyval(left_fit, y)
    # right_x = np.polyval(right_fit, y)
    # lane_center = (left_x + right_x) / 2

    ys = [h * 0.7, h * 0.8, h * 0.9]
    lane_centers = []

    for y in ys:
        lx = np.polyval(left_fit, y)
        rx = np.polyval(right_fit, y)
        lane_centers.append((lx + rx) / 2)

    lane_center = np.mean(lane_centers)

    # tune this manually later
    car_center = 400

    error = lane_center - car_center

    # smoothing
    error = 0.7 * prev_error + 0.3 * error
    prev_error = error

    return error, lane_center, car_center


def get_command(error, threshold=0):
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
# MAIN (Picamera2)
# ---------------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (800, 600)})
picam2.configure(config)
picam2.start()

print("🚗 Lane following started (CTRL+C to stop)")

frame_count = 0
try:
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        frame_count += 1

        # Process only every 3rd frame
        #if frame_count % 3 != 0:
         #   continue

        warped, Minv = perspective_transform(frame)
        mask = threshold_white(warped)
        cv2.imshow("Mask", mask)
        lane_end_detected = detect_lane_end(mask)
        if lane_end_detected:

            print("UTURN DETECTED")

            # Send command to Arduino
            ser.write(b"UTURN_LEFT\n")

            # Wait until Arduino finishes
            while True:

                if ser.in_waiting:

                    msg = ser.readline().decode().strip()

                    print(msg)

                    if msg == "END_U_TURN":

                        break

                continue
                
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

            command = f"L{left_pwm:03d}R{right_pwm:03d}\n"
            ser.write(command.encode())          

            cv2.putText(result, f"Error: {int(error)}", (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        else:
            result = frame
            command = "NO LANE"

        print(command)

        cv2.imshow("Lane Following", result)
        #cv2.imshow("Mask", mask)

        if cv2.waitKey(1) == 27:
            break

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
	