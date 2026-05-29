import cv2
import time
import os
import serial
from collections import deque, Counter
from object_detection import detect_objects, stop_camera
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
ignore_until = 0
current_action = "FORWARD"

is_stopping = False

# ------------------- Ultrasonic distances -------------------
left_dist = 999
center_dist = 999
right_dist = 999

# ============================================================
try:

    while True:

        # ========================================================
        # READ DISTANCES FROM ARDUINO
        # Expected:
        # LEFT:23,CENTER:45,RIGHT:18
        # ========================================================
        min_dist = 999

        while ser.in_waiting:

            try:
                line = ser.readline().decode().strip()

                if "DIST:" in line:

                    min_dist = float(
                        line.replace("DIST:", "").strip()
                    )

                    print("Min Distance:", min_dist)

            except Exception as e:
                print("Parse Error:", e)




        frame, detections = detect_objects()

        detected_classes = []
        detected_objects = []

        for detection in detections:

            x1, y1, x2, y2 = detection["box"]

            name = detection["name"]

            conf = detection["conf"]

            detected_classes.append(name)

            dist = min_dist

            detected_objects.append((name, dist))

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

                # =================================================
                # STOP SIGN / RED
                # =================================================
                if closest_obj in ["red"]:

                    # Ignore repeated stopping
                    if now > ignore_until:

                        if min_dist <= 30:

                            action = "STOP"

                            if not is_stopping:

                                stop_until = now + 3

                                # Ignore stop sign for 5 sec
                                ignore_until = now + 5

                                is_stopping = True

                        else:
                            action = "FORWARD_SLOW"

                    else:
                        # After stop finishes -> move again
                        action = "FORWARD"

                # =================================================
                # YELLOW
                # =================================================
                elif closest_obj == "yellow":

                    if min_dist < 25:
                        action = "FORWARD_SLOW"

                # =================================================
                # GREEN
                # =================================================
                elif closest_obj == "green":

                    action = "FORWARD"

                # =================================================
                # OBSTACLE
                # =================================================
                elif closest_obj in ["lego", "toy-car","stop-sign"]:

                    if min_dist <= 30:

                        action = "STOP"

                        #if not is_stopping:

                            #stop_until = now + 1.5

                            #is_stopping = True

                    elif min_dist <= 30:

                        action = "FORWARD_SLOW"

                    else:
                        action = "FORWARD"
                  
            else:     
                      
                if center_dist <= 20:

                    action = "STOP"

                elif center_dist <= 30:

                    action = "FORWARD"
                            

        # ========================================================
        # COOLDOWN
        # ========================================================
        #if now - last_action_time < ACTION_COOLDOWN:

         #   action = current_action

        #else:

        current_action = action

            #last_action_time = now

        # ========================================================
        # SEND MOTOR COMMAND
        # ========================================================
        if current_action == "STOP":

            ser.write(b"L000R000\n")

        elif current_action == "FORWARD":

            ser.write(b"L150R150\n")

        elif current_action == "FORWARD_SLOW":

            ser.write(b"L110R110\n")

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
            f"C:{center_dist:.0f}",
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

            #cv2.imwrite(filename, frame)

        frame_count += 1

        cv2.imshow("YOLO + Ultrasonic", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.03)
except KeyboardInterrupt:

    print("\nStopping car...")

finally:

    # Send STOP command to Arduino
    try:
        ser.write(b"L000R000\n")
        time.sleep(0.5)
    except:
        pass

    stop_camera()
    
    cv2.destroyAllWindows()

    ser.close()

    print("Program terminated safely")


