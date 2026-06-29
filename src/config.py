# Configuration and tuning constants

# Debugging and logging
SAVE_VIDEO = True
SAVE_LANE = True
SAVE_ROI = False
SAVE_MASK = False
SAVE_SLIDING = False

# Serial Settings
SERIAL_PORT = '/dev/ttyACM0'
SERIAL_BAUD = 9600

# Stop signs / Traffic light thresholds
STOP_SIGN_AREA_THRESHOLD = 25000
YELLOW_SLOW_DURATION = 4.0

# Lane Change Durations (in seconds)
P1_DURATION = 1.0
FORWARD_DURATION = 0.21
PAUSE_DURATION = 0.3
PAUSE_AFTER_STEER_DURATION = 0.3
FORWARD_AFTER_STEER_DURATION = 0.06

# Lane Change Points
P1 = (120, 500)
P1_RIGHT = (680, 500)

# U-Turn Settings
UTURN_MIN_STEER_DURATION = 0.0
UTURN_TIMEOUT = 6.5

# Look Ahead and path factors (if used)
LOOK_AHEAD_FACTOR = 0.7
MERGE_DISTANCE = 150

# YOLO Detector Settings
MODEL_PATH = "/home/gp/self_driving_car/models/best.onnx"
TARGET_SIZE = (800, 600)
LANE_WIDTH = 580  # Expected distance between lines in pixels

# User Custom Auto Parking Settings (Right-Side Reverse Parking)
PARK_SEARCH_DIST_THRESHOLD = 30.0  # Distance threshold (> 30 cm)
PARK_SEARCH_DURATION = 0.3         # Space verification time (0.3 s)
PARK_ALIGN_DURATION = 0.6          # Forward positioning time past the spot (0.6 s)
PARK_BACK_STEER_DURATION = 1.2     # Reverse turn right into spot (1.2 s)
PARK_BACK_STRAIGHT_DURATION = 0.6   # Reverse straight adjustment (0.6 s)
PARK_STOP_DURATION = 0.3           # Pause duration between movements

# Motor Speeds for maneuvers (LxxxRxxx)
PARK_SEARCH_SPEED = 140
PARK_ALIGN_SPEED = 150
PARK_BACK_STEER_LEFT = -190        # Left motor reverses fast to swing tail right
PARK_BACK_STEER_RIGHT = -10        # Right motor reverses slow
PARK_BACK_STRAIGHT = -160          # Both motors reverse straight




