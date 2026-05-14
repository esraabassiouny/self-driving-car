from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import numpy as np
import time
import os
import serial
from collections import deque, Counter

# -------------------
MODEL_PATH = "/home/gp/self_driving_car/models/best.onnx"

model = YOLO(MODEL_PATH)
print("✅ Model loaded")

# -------------------
picam2 = Picamera2()
picam2.configure(
    picam2.create_preview_configuration(
        main={"size": (800, 600)}
    )
)
picam2.start()

print("✅ Camera started")

# -------------------
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)

time.sleep(2)

print("✅ Serial connected")

ser.write(b"START\n")

time.sleep(1)
# -------------------
SAVE_PATH = "/home/gp/self_driving_car/output_frames"
os.makedirs(SAVE_PATH, exist_ok=True)

frame_count = 0

# ------------------- NMS -------------------
def nms(boxes, scores, iou_threshold=0.45):

    boxes = np.array(boxes)
    scores = np.array(scores)

    indices = scores.argsort()[::-1]

    keep = []

    while len(indices) > 0:

        i = indices[0]

        keep.append(i)

        if len(indices) == 1:
            break

        x1 = np.maximum(
            boxes[i][0],
            boxes[indices[1:], 0]
        )

        y1 = np.maximum(
            boxes[i][1],
            boxes[indices[1:], 1]
        )

        x2 = np.minimum(
            boxes[i][2],
            boxes[indices[1:], 2]
        )

        y2 = np.minimum(
            boxes[i][3],
            boxes[indices[1:], 3]
        )

        inter = np.maximum(0, x2 - x1) * \
                np.maximum(0, y2 - y1)

        iou = inter / (
            (boxes[i][2] - boxes[i][0]) *
            (boxes[i][3] - boxes[i][1]) +

            (boxes[indices[1:], 2] -
             boxes[indices[1:], 0]) *

            (boxes[indices[1:], 3] -
             boxes[indices[1:], 1])

            - inter + 1e-6
        )

        indices = indices[1:][iou < iou_threshold]

    return keep

# ------------------- Stability -------------------
history = deque(maxlen=10)

def get_stable_detection(history):

    flat = [item for sublist in history for item in sublist]

    if not flat:
        return None

    return Counter(flat).most_common(1)[0][0]

# ------------------- State -------------------
last_action_time = 0
ACTION_COOLDOWN = 1.0

stop_until = 0
current_action = "FORWARD"

is_stopping = False

# ------------------- Ultrasonic distances -------------------
left_dist = 999
center_dist = 999
right_dist = 999

# ============================================================
while True:

    # ========================================================
    # READ DISTANCE FROM ARDUINO
    # ========================================================
    if ser.in_waiting:

        try:
            line = ser.readline().decode().strip()

            # Example:
            # LEFT:20,CENTER:35,RIGHT:18

            parts = line.split(",")

            left_dist = float(
                parts[0].split(":")[1]
            )

            center_dist = float(
                parts[1].split(":")[1]
            )

            right_dist = float(
                parts[2].split(":")[1]
            )

        except:
            pass

    # ========================================================
    # CAMERA
    # ========================================================
    frame = picam2.capture_array()

    # Fix color
    if frame.shape[2] == 4:
        frame = cv2.cvtColor(
            frame,
            cv2.COLOR_RGBA2BGR
        )
    else:
        frame = cv2.cvtColor(
            frame,
            cv2.COLOR_RGB2BGR
        )

    # ========================================================
    # YOLO
    # ========================================================
    results = model(frame, conf=0.5)

    boxes_all = []
    scores_all = []
    class_ids_all = []

    if results[0].boxes is not None:

        for box in results[0].boxes:

            x1, y1, x2, y2 = box.xyxy[0].tolist()

            cls_id = int(box.cls[0])

            conf = float(box.conf[0])

            boxes_all.append([x1, y1, x2, y2])

            scores_all.append(conf)

            class_ids_all.append(cls_id)

    # ========================================================
    # APPLY NMS
    # ========================================================
    keep = nms(boxes_all, scores_all)

    detected_classes = []
    detected_objects = []

    for i in keep:

        x1, y1, x2, y2 = map(int, boxes_all[i])

        cls_id = class_ids_all[i]

        conf = scores_all[i]

        name = results[0].names[cls_id]

        detected_classes.append(name)

        # ==========================
        # USE ULTRASONIC DISTANCE
        # ==========================
        frame_center = (x1 + x2) // 2
        frame_width = frame.shape[1]

        # LEFT region
        if frame_center < frame_width // 3:
            dist = left_dist

        # CENTER region
        elif frame_center < 2 * frame_width // 3:
            dist = center_dist

        # RIGHT region
        else:
            dist = right_dist

        detected_objects.append((name, dist))

        # Draw
        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f"{name}, {conf:.1f}, {dist:.1f}cm",
            (x1, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1
        )

    # ========================================================
    # STABILITY
    # ========================================================
    history.append(detected_classes)

    stable_object = get_stable_detection(history)

    # ========================================================
    # CLOSEST OBJECT
    # ========================================================
    closest_obj = None

    min_dist = 999

    for name, dist in detected_objects:

        if dist < min_dist:

            min_dist = dist

            closest_obj = name

    now = time.time()

    # ========================================================
    # ACTION LOGIC
    # ========================================================
    if now < stop_until:

        action = "STOP"

    else:

        if is_stopping:
            is_stopping = False

        action = "FORWARD"

        if closest_obj:

            # STOP SIGN / RED
            if closest_obj in ["stop-sign", "red"]:

                if min_dist <= 25:

                    action = "STOP"

                    if not is_stopping:

                        stop_until = now + 3

                        is_stopping = True

                else:
                    action = "FORWARD_SLOW"

            # YELLOW
            elif closest_obj == "yellow":

                if min_dist < 25:
                    action = "FORWARD_SLOW"

            # GREEN
            elif closest_obj == "green":

                action = "FORWARD"

            # OBSTACLE
            elif closest_obj in ["lego", "toy-car"]:

                if min_dist <= 20:

                    action = "STOP"

                    if not is_stopping:

                        stop_until = now + 1.5

                        is_stopping = True

                elif min_dist <= 30:

                    action = "FORWARD_SLOW"

                else:
                    action = "FORWARD"

    # ========================================================
    # COOLDOWN
    # ========================================================
    if now - last_action_time < ACTION_COOLDOWN:

        action = current_action

    else:

        current_action = action

        last_action_time = now

    # ========================================================
    # SEND MOTOR COMMAND
    # ========================================================
    if current_action == "STOP":

        ser.write(b"L000R000\n")

    elif current_action == "FORWARD":

        ser.write(b"L150R150\n")

    elif current_action == "FORWARD_SLOW":

        ser.write(b"L090R090\n")

    elif current_action == "SLOW":

        ser.write(b"L060R060\n")

    # ========================================================
    # DEBUG
    # ========================================================
    cv2.putText(
        frame,
        f"ACTION: {current_action}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 0, 255),
        2
    )

    cv2.putText(
        frame,
        f"{closest_obj} {min_dist:.1f}cm",
        (20, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 0, 0),
        2
    )

    cv2.putText(
        frame,
        f"L:{left_dist:.0f} C:{center_dist:.0f} R:{right_dist:.0f}",
        (20, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )

    print(
        "Closest:",
        closest_obj,
        "| Dist:",
        min_dist,
        "| Action:",
        current_action
    )

    # ========================================================
    # SAVE
    # ========================================================
    if detected_classes:

        filename = f"{SAVE_PATH}/frame_{frame_count}.jpg"

        cv2.imwrite(filename, frame)

    frame_count += 1

    cv2.imshow("YOLO + Ultrasonic", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(0.03)

# ============================================================
picam2.stop()

cv2.destroyAllWindows()

ser.close()