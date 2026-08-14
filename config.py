# --- Rover Network ---
ROVER_IP   = "192.168.50.11"
ROVER_PORT = 80
ROVER_URL  = f"http://{ROVER_IP}:{ROVER_PORT}/js"
HTTP_TIMEOUT = 0.3

# --- Camera ---
CAMERA_INDEX   = 0                  # 0 = first USB camera
FRAME_WIDTH    = 640
FRAME_HEIGHT   = 480
TARGET_FPS     = 30

# --- Green detection (HSV color space) ---
# OpenCV Hue: 0=Red, 30=Yellow, 60=Green, 90=Cyan, 120=Blue
GREEN_HSV_LOW  = (65,  60,  40)     # Minimum Hue, Saturation, Value
GREEN_HSV_HIGH = (88, 255, 255)     # Maximum Hue, Saturation, Value

MIN_GREEN_AREA = 2000 # in Pixel

ROI_TOP_RATIO    = 0.03             # Small margin at top
ROI_BOTTOM_RATIO = 0.90             # Cut off bottom 10%

# --- Zone ratios (relative to ROI height) ---
FAR_ZONE_END    = 0.20              # Top 20% of ROI = FAR zone
NEAR_ZONE_START = 0.40              # NEAR zone starts at 40%
NEAR_ZONE_END   = 0.75              # NEAR zone ends at 75%

# --- Driving speeds ---
# Wave Rover speeds: -1.0 to 1.0
SPEED_FORWARD   = 0.20
SPEED_TURN_MAX  = 0.10
SPEED_SEARCH    = 0.15

# --- Steering ---
# in this range the rover drives straight
DEAD_ZONE_RATIO = 0.167

# PD controller coefficients
KP = 0.55                          # Proportional component
KD = 0.10                          # Differential component (dampens overshoot)

# --- Stuck detection ---
# If the rover makes no forward movement for X seconds,
# the pathfinding is completely reset.
STUCK_RESET_S = 10.0                 # Seconds until pathfinding reset

# --- Bend detection & speed adjustment ---
# speed_factor goes from 1.0 (straight) down to SPEED_MIN_FACTOR (sharp bend),
# linearly between BEND_SLOW_DEG and BEND_STOP_DEG. This factor is the only
# thing that reacts to a bend/kink in the path - no separate alignment
# maneuver, just base_speed * speed_factor fed into the normal PD steering.
BEND_SLOW_DEG     = 15.0           # From this angle: start slowing down
BEND_STOP_DEG     = 32.0           # From this angle: speed_factor = 0 (PD correction alone pivots the rover)
SPEED_MIN_FACTOR  =  0.30          # Minimum speed factor when braking

# --- Rotation tracking ---
ROTATE_DEG_PER_SEC   = 50.0       # Degrees/second at turn_in_place

# ALIGNING: Never rotate more than this angle
MAX_ALIGN_ROTATION_DEG = 180.0

# SEARCHING: Rotate at most this angle per direction, then reverse
MAX_SEARCH_ROTATION_DEG = 180.0

# --- Behavior when path is lost ---
SEARCH_TIMEOUT_S   = 5.0           # After X seconds without path: start search
SEARCH_ROTATION    = 0.3           # Rotation speed while searching
SEARCH_DIRECTION   = "left"        # "left" or "right" - initial search direction

# --- Red stop marking ---
RED_STOP_WAIT_S    = 10.0            # Seconds until "signal" received (dummy)
TURN_180_SPD       = 0.20            # Rotation speed for 180° turn
RED_DETECT_Y_START = 0.55            # Red only in lower part (>= 55% frame height)
RED_COOLDOWN_S     = 15.0            # Seconds of red lock after restart

# --- Debug / Visualization ---
DEBUG_WINDOW      = False          # OpenCV window (only useful with monitor on Pi)
DEBUG_SHOW_MASK   = False          # Green mask as second OpenCV window
DEBUG_PRINT_SPEED = True           # Print speed values to console

# --- Web debug server ---
# accessible at: http://192.168.4.1:5000
DEBUG_WEB_SERVER  = True
DEBUG_SERVER_PORT = 5000
DEBUG_STREAM_FPS  = 15