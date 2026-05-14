#!/usr/bin/env python3
from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import time

# =============================
# 🔧 CONFIG
# =============================
FRAME_WIDTH = 800
FRAME_HEIGHT = 600

BASE_SPEED = 100
KP = 0.6
EXPECTED_LANE_WIDTH = 200

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

MODEL_PATH = "/home/gp/self_driving_car/models/best.onnx"

# =============================
# 🚀 INIT
# =============================

# Load YOLO
model = YOLO(MODEL_PATH)
print("✅ YOLO model loaded")

# Camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (FRAME_WIDTH, FRAME_HEIGHT)}
))
picam2.start()
time.sleep(2)
print("✅ Camera started")

# Serial
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
time.sleep(2)
print("✅ Arduino connected")

# Stop motors initially
ser.write(b"L000R000\n")
time.sleep(1)

# =============================
# 🔁 STATE VARIABLES
# =============================
prev_lane_center = FRAME_WIDTH // 2

# =============================
# 🛣️ LANE DETECTION
# =============================
def detect_lanes(frame):
    global prev_lane_center

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    _, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)

    roi = thresh[FRAME_HEIGHT//2:, :]

    lines = cv2.HoughLinesP(roi, 1, np.pi/180, 20,
                            minLineLength=20, maxLineGap=30)

    left_x, right_x = [], []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1 + 1e-6)

            if slope < -0.3:
                left_x.extend([x1, x2])
                cv2.line(frame, (x1, y1 + FRAME_HEIGHT//2),
                         (x2, y2 + FRAME_HEIGHT//2), (255,0,0), 2)

            elif slope > 0.3:
                right_x.extend([x1, x2])
                cv2.line(frame, (x1, y1 + FRAME_HEIGHT//2),
                         (x2, y2 + FRAME_HEIGHT//2), (0,0,255), 2)

    # Compute lane center
    if left_x and right_x:
        lane_center = (np.mean(left_x) + np.mean(right_x)) / 2
    elif left_x:
        lane_center = np.mean(left_x) + EXPECTED_LANE_WIDTH / 2
    elif right_x:
        lane_center = np.mean(right_x) - EXPECTED_LANE_WIDTH / 2
    else:
        lane_center = prev_lane_center

    # Smooth
    lane_center = 0.8 * prev_lane_center + 0.2 * lane_center
    prev_lane_center = lane_center

    # Draw center
    cv2.line(frame, (int(lane_center), FRAME_HEIGHT//2),
             (int(lane_center), FRAME_HEIGHT), (0,255,0), 2)

    return lane_center, frame

# =============================
# 🧠 OBJECT DETECTION
# =============================
def detect_objects(frame):
    objects = []

    results = model(frame, imgsz=320, conf=0.3)

    if results[0].boxes is not None:
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            name = results[0].names[cls_id]

            objects.append(name)

            x1, y1, x2, y2 = map(int, box.xyxy[0])

            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.putText(frame, f"{name} {conf:.2f}",
                        (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0,255,0), 1)

    return objects, frame

# =============================
# 🎯 DECISION LOGIC
# =============================
def decide(lane_center, objects):
    error = lane_center - FRAME_WIDTH / 2
    turn = int(KP * error)

    left_pwm = BASE_SPEED + turn
    right_pwm = BASE_SPEED - turn

    # Clamp
    left_pwm = max(0, min(255, left_pwm))
    right_pwm = max(0, min(255, right_pwm))

    # 🚨 RULES
    if "stop-sign" in objects:
        print("🛑 STOP SIGN")
        return 0, 0

    if "red" in objects:
        print("🔴 RED LIGHT")
        return 0, 0

    if "toy-car" in objects:
        print("🚧 OBSTACLE - slow")
        return int(left_pwm * 0.5), int(right_pwm * 0.5)

    return int(left_pwm), int(right_pwm)

# =============================
# 🔌 SEND TO ARDUINO
# =============================
def send_to_arduino(left, right):
    cmd = f"L{left:03d}R{right:03d}\n"
    ser.write(cmd.encode())

# =============================
# 🔄 MAIN LOOP
# =============================
print("🚗 System started")

print("🚗 System started (Press 'q' to quit, CTRL+C for emergency stop)")

try:
    while True:
        frame = picam2.capture_array()

        # Fix color
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        else:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Perception
        lane_center, frame = detect_lanes(frame)
        objects, frame = detect_objects(frame)

        # Decision
        left_pwm, right_pwm = decide(lane_center, objects)

        # Actuation
        send_to_arduino(left_pwm, right_pwm)

        # Display
        cv2.imshow("Self Driving Car", frame)

        # 🟡 Normal quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("🟡 Quit requested")
            break

        time.sleep(0.03)

# 🔴 EMERGENCY STOP (CTRL + C)
except KeyboardInterrupt:
    print("\n🛑 EMERGENCY STOP TRIGGERED (CTRL+C)")

# =============================
# 🧹 CLEANUP (ALWAYS RUNS)
# =============================
finally:
    print("🛑 Stopping motors...")
    send_to_arduino(0, 0)
    time.sleep(0.5)

    picam2.stop()
    cv2.destroyAllWindows()
    ser.close()

    print("✅ System safely stopped")