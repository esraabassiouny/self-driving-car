from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import numpy as np
import time

MODEL_PATH = "/home/gp/self_driving_car/models/best.onnx"
TARGET_SIZE = (416, 416)

# =========================
# LOAD MODEL
# =========================
model = YOLO(MODEL_PATH)

# =========================
# CAMERA
# =========================
picam2 = Picamera2()

picam2.configure(
    picam2.create_preview_configuration(
        main={"size": (800, 600)}
    )
)

picam2.start()

print("✅ Camera started")
print("✅ Model loaded")
# =========================
# NMS
# =========================
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

# =========================
# DETECTION FUNCTION
# =========================
def detect_objects():

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

    # Resize
    h0, w0 = frame.shape[:2]

    r = min(
        TARGET_SIZE[0] / w0,
        TARGET_SIZE[1] / h0
    )

    new_w = int(w0 * r)
    new_h = int(h0 * r)

    frame_resized = cv2.resize(
        frame,
        (new_w, new_h)
    )

    # YOLO
    results = model(frame_resized, conf=0.5)

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

    # Apply NMS
    keep = nms(boxes_all, scores_all)

    detections = []

    for i in keep:

        x1, y1, x2, y2 = map(int, boxes_all[i])

        cls_id = class_ids_all[i]

        conf = scores_all[i]

        name = results[0].names[cls_id]

        detections.append({
            "name": name,
            "conf": conf,
            "box": (x1, y1, x2, y2)
        })

    return frame_resized, detections

# =========================
# CLEANUP
# =========================
def stop_camera():
    picam2.stop()

# =========================================================
# MAIN
# =========================================================
if __name__ == "__main__":

    try:

        while True:

            frame, detections = detect_objects()

            detected_classes = []

            for detection in detections:

                x1, y1, x2, y2 = detection["box"]

                name = detection["name"]

                conf = detection["conf"]

                detected_classes.append(name)

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
                    f"{name} {conf:.2f}",
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1
                )

            # Debug print
            if detected_classes:
                print("Detected:", detected_classes)

            cv2.imshow("YOLO Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.03)

    except KeyboardInterrupt:

        print("\nStopping...")

    finally:

        stop_camera()

        cv2.destroyAllWindows()

        print("Program terminated safely")

