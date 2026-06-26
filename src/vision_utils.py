import cv2
import numpy as np

def save_frame(path, img):
    cv2.imwrite(path, img)

def perspective_transform_lane_change(frame):
    h, w = frame.shape[:2]

    src = np.float32([
        [0, 380],     # top-left
        [680, 380],   # top-right
        [790, 600],   # bottom-right
        [10, 600]     # bottom-left
    ])

    dst = np.float32([
        [0, 0],
        [800, 0],
        [800, 600],
        [0, 600]
    ])

    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)

    warped = cv2.warpPerspective(frame, M, (800, 600))

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

def estimate_distance(area):
    if area <= 0:
        return 10.0
    return (2000 / area) * 100  # area is inverse with area (big area = small distance = close obstacle)
