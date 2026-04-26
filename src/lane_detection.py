#!/usr/bin/env python3

import cv2

import numpy as np

import time

from picamera2 import Picamera2

import serial



# ---------------- Picamera2 setup ----------------

picam2 = Picamera2()

picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (800, 600)}))

picam2.start()

time.sleep(2)  # allow camera to warm up



# ---------------- Arduino serial ----------------

ser = serial.Serial('/dev/ttyACM0', 9600)

time.sleep(2)  # wait for Arduino to initialize



# ✅ RESET motors (STOP car at startup)

ser.write(b"L000R000\n")

time.sleep(1)  # small delay to ensure command is applied



# ---------------- Lane detection params ----------------

frame_width = 800

frame_height = 600

base_speed = 100  # base PWM speed

Kp = 0.8      # proportional gain

expected_lane_width = 200  # pixels, approximate width between lanes

prev_lane_center = frame_width / 2  # initialize lane center



def getFrame():

    img = picam2.capture_array()

    img = cv2.resize(img, (frame_width, frame_height))

    return img



while True:

    img = getFrame()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(gray, (5,5), 0)

    

    # Threshold: white lane lines

    ret, thresh = cv2.threshold(blur, 240, 255, cv2.THRESH_BINARY)

    

    # Focus on lower half of frame

    roi = thresh[frame_height//2:, :]

    

    # Hough lines

    lines = cv2.HoughLinesP(roi, 1, np.pi/180, 20, minLineLength=20, maxLineGap=30)

    

    left_x = []

    right_x = []

    

    if lines is not None:

        for line in lines:

            x1, y1, x2, y2 = line[0]

            slope = (y2 - y1) / (x2 - x1 + 1e-6)  # avoid divide by zero

            if slope < -0.3:  # left line

                left_x.extend([x1, x2])

                cv2.line(img, (x1, y1 + frame_height//2), (x2, y2 + frame_height//2), (255,0,0), 2)

            elif slope > 0.3:  # right line

                right_x.extend([x1, x2])

                cv2.line(img, (x1, y1 + frame_height//2), (x2, y2 + frame_height//2), (0,0,255), 2)

    

    # ---------------- Compute lane center with fallback ----------------

    if left_x and right_x:

        left_avg = np.mean(left_x)

        right_avg = np.mean(right_x)

        lane_center = (left_avg + right_avg) / 2

    elif left_x:  # only left line detected

        left_avg = np.mean(left_x)

        lane_center = left_avg + expected_lane_width / 2

    elif right_x:  # only right line detected

        right_avg = np.mean(right_x)

        lane_center = right_avg - expected_lane_width / 2

    else:  # no lines detected

        lane_center = prev_lane_center  # use previous lane center

    

    # Smooth lane center to reduce jitter

    lane_center = 0.8 * prev_lane_center + 0.2 * lane_center

    prev_lane_center = lane_center

    

    # ---------------- Compute motor speeds ----------------

    error = lane_center - frame_width / 2

    turn = int(Kp * error)

    left_pwm = max(0, min(255, base_speed - turn))

    right_pwm = max(0, min(255, base_speed + turn))

    

    # Send to Arduino

    command = f"L{left_pwm:03d}R{right_pwm:03d}\n"

    ser.write(command.encode())

    

    # Draw lane center

    cv2.line(img, (int(lane_center), frame_height//2), (int(lane_center), frame_height), (0,255,0), 2)

    

    # Show windows for debugging

    cv2.imshow('img', img)

    cv2.imshow('thresh', thresh)

    

    if cv2.waitKey(1) & 0xFF == ord('q'):

        break



cv2.destroyAllWindows()

ser.close()

