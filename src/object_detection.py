from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import time
import os

# -------------------
model = YOLO("/home/gp/self_driving_car/best.onnx")
print("✅ Model loaded")

# -------------------
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

print("✅ Camera started")

SAVE_PATH = "/home/gp/self_driving_car/output_frames"
os.makedirs(SAVE_PATH, exist_ok=True)

frame_count = 0
last_classes = set()

# -------------------
while True:
    frame = picam2.capture_array()

    # 🔥 FIX COLOR (VERY IMPORTANT)
    if frame.shape[2] == 4:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    else:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Run YOLO (same as laptop)
    results = model(frame, imgsz=320, conf=0.3)

    current_classes = set()

    if results[0].boxes is not None:
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            name = results[0].names[cls_id]

            current_classes.add(name)

            # Draw
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.putText(frame, f"{name} {conf:.2f}", (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

    # -------- PRINT ONLY NEW --------
    new = current_classes - last_classes
    for obj in new:
        print(f"🚨 Detected: {obj}")

    last_classes = current_classes

    # -------- SAVE IMAGE --------
    if len(current_classes) > 0:
        filename = f"{SAVE_PATH}/frame_{frame_count}.jpg"
        cv2.imwrite(filename, frame)
        print(f"📸 Saved: {filename}")

    frame_count += 1

    # -------- SHOW --------
    cv2.imshow("YOLO Pi", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(0.03)

picam2.stop()
cv2.destroyAllWindows()