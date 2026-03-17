# config.py

# ---- Motors ----
MOTOR_L_DIR  = 4
MOTOR_L_PWM  = 5
MOTOR_R_DIR  = 7
MOTOR_R_PWM  = 6
MOTOR_L_INVERT = True
MOTOR_R_INVERT = True
MOTOR_PWM_FREQ_HZ = 1000

# ---- Line sensors (LEFT -> RIGHT) ----
LINE_PINS = [12, 1, 2, 3]
LINE_INVERT = False  # keep unless your black/white polarity is wrong

# ---- Loop ----
LOOP_HZ = 50

# ---- PD following (YOUR tuned values) ----
BASE_THROTTLE = 0.65
KP = 0.35
KD = 0.008
MAX_STEER = 0.9
SLOW_K = 0.10
MIN_THROTTLE = 0.10


# ---- Event detection ----
RECENT_GOOD_LINE_MS = 350 # how long a "recent good line" counts for (used for corner/event detection)
EVENT_REARM_N = 3 #how many consecutive good line readings are needed to rearm event detection after it was triggered
EVENT_REARM_MAX = 30 # maximum value for good_rearm counter (prevents overflow if we get a very long good line stretch)
EVENT_ENTER_N = 3 # how many consecutive event readings are needed to trigger an event (intersection or corner)
EVENT_COOLDOWN_MS = 300
