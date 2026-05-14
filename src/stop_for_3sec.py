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
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)

print("✅ Serial connected")

# -------------------
SAVE_PATH = "/home/gp/self_driving_car/output_frames"
os.makedirs(SAVE_PATH, exist_ok=True)

frame_count = 0

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

# ------------------- Ultrasonic -------------------
ultrasonic_distance = 999

# =====================================================
while True:

    # =================================================
    # Read Arduino distance
    # =================================================
    if ser.in_waiting:

        try:
            line = ser.readline().decode().strip()

            # Expected:
            # DISTANCE: 23.5 cm
            if "DISTANCE:" in line:

                value = line.replace("DISTANCE:", "")
                value = value.replace("cm", "")
                value = value.strip()

                ultrasonic_distance = float(value)

        except:
            pass

    # =================================================
    # Camera
    # =================================================
    frame = picam2.capture_array()

    # Fix color
    if frame.shape[2] == 4:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    else:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # =================================================
    # YOLO Inference
    # =================================================
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

    # =================================================
    # NMS
    # =================================================
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

            x1 = np.maximum(boxes[i][0], boxes[indices[1:], 0])
            y1 = np.maximum(boxes[i][1], boxes[indices[1:], 1])
            x2 = np.minimum(boxes[i][2], boxes[indices[1:], 2])
            y2 = np.minimum(boxes[i][3], boxes[indices[1:], 3])

            inter = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)

            iou = inter / (
                (boxes[i][2] - boxes[i][0]) *
                (boxes[i][3] - boxes[i][1]) +

                (boxes[indices[1:], 2] - boxes[indices[1:], 0]) *
                (boxes[indices[1:], 3] - boxes[indices[1:], 1])

                - inter + 1e-6
            )

            indices = indices[1:][iou < iou_threshold]

        return keep

    keep = nms(boxes_all, scores_all)

    detected_classes = []
    detected_objects = []

    # =================================================
    # Draw detections
    # =================================================
    for i in keep:

        x1, y1, x2, y2 = map(int, boxes_all[i])

        cls_id = class_ids_all[i]
        conf = scores_all[i]

        name = results[0].names[cls_id]

        # ✅ Use ultrasonic distance
        dist = ultrasonic_distance

        detected_classes.append(name)
        detected_objects.append((name, dist))

        cv2.rectangle(frame, (x1, y1), (x2, y2),
                      (0, 255, 0), 2)

        cv2.putText(
            frame,
            f"{name}, {conf:.1f}, {dist:.1f}cm",
            (x1, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1
        )

    # =================================================
    # Stability
    # =================================================
    history.append(detected_classes)

    stable_object = get_stable_detection(history)

    # =================================================
    # Closest Object
    # =================================================
    closest_obj = None
    min_dist = ultrasonic_distance

    if detected_objects:
        closest_obj = detected_objects[0][0]

    now = time.time()

    # =================================================
    # Stop timer
    # =================================================
    if now > stop_until:

        is_stopping = False
        action = "FORWARD"

    # =================================================
    # Decision Making
    # =================================================
    if not is_stopping:

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
                else:
                    action = "FORWARD"

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

            else:
                action = "FORWARD"

        else:
            action = "FORWARD"

    # =================================================
    # Cooldown
    # =================================================
    if now - last_action_time < ACTION_COOLDOWN:

        action = current_action

    else:

        current_action = action
        last_action_time = now

    # =================================================
    # Send to Arduino
    # =================================================
    if current_action == "STOP":
        ser.write(b"L000R000\n")

    elif current_action == "FORWARD":
        ser.write(b"L150R150\n")

    elif current_action == "FORWARD_SLOW":
        ser.write(b"L090R090\n")

    elif current_action == "SLOW":
        ser.write(b"L060R060\n")

    # =================================================
    # Debug
    # =================================================
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

    print(
        "Closest:",
        closest_obj,
        "| Dist:",
        min_dist,
        "| Action:",
        current_action
    )

    # =================================================
    # Save frames
    # =================================================
    if detected_classes:

        filename = f"{SAVE_PATH}/frame_{frame_count}.jpg"

        cv2.imwrite(filename, frame)

    frame_count += 1

    cv2.imshow("YOLO Distance Control", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(0.03)

# =====================================================
picam2.stop()

cv2.destroyAllWindows()

ser.close()