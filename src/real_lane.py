import cv2
import numpy as np
import config

frame_id = 0

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
    cv2.polylines(debug, [pts], True, (0, 255, 0), 3)

    cv2.imshow("ROI", debug)
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = np.linalg.inv(M)

    warped = cv2.warpPerspective(img, M, (w, h))
    return warped, Minv, M, debug


# ---------------------------
# 2. White Mask
# ---------------------------
def threshold_white(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 220])
    upper_white = np.array([180, 10, 255])
    return cv2.inRange(hsv, lower_white, upper_white)


# ---------------------------
# 3. Sliding Window
# ---------------------------
def sliding_window(binary_warped):
    histogram = np.sum(
        binary_warped[int(binary_warped.shape[0]*0.72):, :],
        axis=0
    )

    midpoint = histogram.shape[0] // 2
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    nwindows = 9
    window_height = binary_warped.shape[0] // nwindows
    margin = 60
    minpix = 50

    # Get all white pixels
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    # Create debug image
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))
    
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

        # Draw green tracking windows
        cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
        cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)

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
            leftx_current = int(np.mean(nonzerox[good_left]))

        # Move right window center
        if len(good_right) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right]))

    # Merge all indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # Color detected pixels: Blue for left lane, Red for right lane
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    # Show debug image
    cv2.imshow("Sliding Windows", out_img)

    return (
        nonzerox[left_lane_inds],
        nonzeroy[left_lane_inds],
        nonzerox[right_lane_inds],
        nonzeroy[right_lane_inds],
        out_img
    )


def compute_pwm(error, base_speed=125, max_adjust=130):
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
def fit_polynomial(binary_warped):
    leftx, lefty, rightx, righty, sliding_img = sliding_window(binary_warped)
    w = binary_warped.shape[1]

    left_valid = len(leftx) >= 50
    right_valid = len(rightx) >= 50

    # Prevent both trackers from locking onto the same line (crossover / too close)
    if left_valid and right_valid:
        mean_left = np.mean(leftx)
        mean_right = np.mean(rightx)
        if (mean_right - mean_left) < 250:
            if mean_left > w / 2:
                left_valid = False
            else:
                right_valid = False

    lane_width = config.LANE_WIDTH

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
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])

    left_fitx = np.polyval(left_fit, ploty)
    right_fitx = np.polyval(right_fit, ploty)

    if left_valid:
        out_img[lefty, leftx] = [255, 0, 0]
    if right_valid:
        out_img[righty, rightx] = [0, 0, 255]

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

    return left_fit, right_fit, left_valid, right_valid


# ---------------------------
# 5. Steering Logic
# ---------------------------
prev_error = 0

def compute_steering(left_fit, right_fit, left_valid, right_valid, shape, state=None):
    global prev_error
    h, w = shape[:2]
    car_center = w / 2

    # Compute base center fit if lane fits are available
    if left_fit is not None and right_fit is not None:
        center_fit = (left_fit + right_fit) / 2.0
        y_lookahead = h * 0.75
        lane_center = np.polyval(center_fit, y_lookahead)
    else:
        lane_center = car_center

    if state is None:
        state = 'LANE_FOLLOW'

    error = 0.0

    if state == 'LANE_FOLLOW':
        error = lane_center - car_center

    elif state == 'REACH_P1':
        error = config.P1[0] - car_center

    elif state in ('FORWARD', 'FORWARD_AFTER_STEER'):
        error = 0.0

    elif state in ('PAUSE', 'PAUSE_AFTER_STEER'):
        error = 0.0

    elif state == 'STEER_RIGHT':
        error = 40.0

    elif state == 'STOP':
        error = 0.0

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
    roi = binary_img

    # 1. Row-sum projection
    row_sums = np.sum(roi == 255, axis=1)
    max_row_sum = np.max(row_sums) if len(row_sums) > 0 else 0

    # 2. Contours
    contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    lane_end_by_contour = False
    for c in contours:
        area = cv2.contourArea(c)
        if area > 2500:
            x, y, wb, hb = cv2.boundingRect(c)
            aspect = wb / hb
            if wb > 170 and aspect > 0.45:
                lane_end_by_contour = True
                break

    if max_row_sum > 150 or lane_end_by_contour:
        print(f"[Lane End] Detected by Projection/Contour: max_row_sum={max_row_sum}, contour={lane_end_by_contour}")
        return True

    # 3. Hough Lines
    lines = cv2.HoughLinesP(
        roi,
        1,
        np.pi / 180,
        threshold=30,
        minLineLength=100,
        maxLineGap=40
    )

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            dx = x2 - x1
            dy = y2 - y1
            slope = dy / (dx + 1e-6)
            line_length = np.sqrt(dx**2 + dy**2)
            if abs(slope) < 0.25 and line_length > 120:
                print(f"[Lane End] Detected by Hough: length={line_length:.1f}, slope={slope:.3f}")
                return True

    return False


if __name__ == "__main__":
    print("real_lane.py is a module and should not be run directly.")
