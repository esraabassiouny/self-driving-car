import serial
import time

from object_detection import detect_objects, stop_camera

# ==========================================
# SERIAL
# ==========================================
ser = serial.Serial(
    '/dev/ttyACM0',
    9600,
    timeout=0.1
)

time.sleep(2)

# ==========================================
# VARIABLES
# ==========================================
latest_distance = None

MIN_CONSECUTIVE_DETECTIONS = 3
detection_counter = {}
best_detections = {}


# ==========================================
# START SWEEP (ONCE)
# ==========================================
def start_sweep():
    ser.write(b"START_SWEEP\n")


# ==========================================
# READ DISTANCE FROM ARDUINO
# ==========================================
def read_distance():
    global latest_distance

    while ser.in_waiting:
        try:
            line = ser.readline().decode(errors="ignore").strip()

            if "MIN_DIST:" in line:
                latest_distance = float(line.split("MIN_DIST:")[1])

        except:
            pass

    return latest_distance


# ==========================================
# MAIN
# ==========================================
try:
    print("\n========================")
    print("DETECTION TEST STARTED")
    print("========================\n")

    start_sweep()   # مرة واحدة فقط

    while True:

        distance = read_distance()
        frame, detections = detect_objects()

        if distance is None:
            continue

        current_objects = set()

        for det in detections:

            name = det.get("name", "unknown")
            conf = det.get("conf", 0.0)
            min_distance = det.get("min_distance", distance)

            current_objects.add(name)

            # =========================
            # PRINT ONLY
            # =========================
            print(
                f"{name:15s} | "
                f"conf={conf:.2f} | "
                f"dist={min_distance:.1f} cm | "
                f"radar={distance:.1f} cm"
            )

            # =========================
            # STABILITY COUNTER
            # =========================
            detection_counter[name] = detection_counter.get(name, 0) + 1

            if detection_counter[name] >= MIN_CONSECUTIVE_DETECTIONS:

                if name not in best_detections:
                    best_detections[name] = {
                        "max_distance": min_distance,
                        "best_conf": conf
                    }

                    print(f"🟢 FIRST STABLE DETECTION -> {name}")

                else:
                    if min_distance > best_detections[name]["max_distance"] + 5:

                        best_detections[name]["max_distance"] = min_distance

                        if conf > best_detections[name]["best_conf"]:
                            best_detections[name]["best_conf"] = conf

                        print(f"🔵 NEW RECORD -> {name} {min_distance:.1f} cm")

        # =========================
        # RESET COUNTER
        # =========================
        for obj in list(detection_counter.keys()):
            if obj not in current_objects:
                detection_counter[obj] = 0

        time.sleep(0.05)


except KeyboardInterrupt:
    print("\nStopped by user")

finally:
    print("\n========================")
    print("FINAL RESULTS")
    print("========================")

    for name, data in best_detections.items():
        print(
            f"{name:15s} | "
            f"max_distance={data['max_distance']:.1f} cm | "
            f"best_conf={data['best_conf']:.2f}"
        )

    stop_camera()
    ser.close()

    print("\nProgram terminated safely")