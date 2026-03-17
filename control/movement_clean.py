

from utime import ticks_ms, ticks_diff, sleep_ms
from control.pd import middle_error_white_line, pd_follow
import config

def fixed_rate_tick(t_last, period_ms):
    t_now = ticks_ms()
    dt = ticks_diff(t_now, t_last)
    if dt < period_ms:
        sleep_ms(period_ms - dt)
        t_now = ticks_ms()
    return t_now


def clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


class MotionContext:

    def __init__(self):
        self.last_err = 0.0
        self.last_search_dir = +1
        self.t_last_good_line = ticks_ms()


#Steers toward the line towards the last known direction
def default_search_drive(motors, ctx, cfg):
    motors.arcade(cfg["search_throttle"], ctx.last_search_dir * cfg["search_steer"])

#Stops the robot by commanding zero throttle and zero steer
def default_stop(motors):
    motors.arcade(0.0, 0.0)



def compute_event(t_now, ctx, corner_raw, inter_raw):
    recent_good = ticks_diff(t_now, ctx.t_last_good_line) <= config.RECENT_GOOD_LINE_MS
    corner_cond = recent_good and corner_raw and (not inter_raw)
    event_cond = inter_raw or corner_cond
    return inter_raw, corner_cond, event_cond, recent_good



# ---------------------------------------------------------------------


#Border push cofiguration parameters. Adjust as needed.
border_push_cfg = {
    "period_ms": 20,

    "start_throttle": 0.5,
    "start_see_white_n": 2,

    "border_timeout_ms": 5000,
    "border_bias_k": 0.20,
    "border_bias_max": 0.60,
    "border_throttle": 0.35,
    "border_exit_n": 3,

    "acquire_timeout_ms": 3000,
    "acquire_search_steer": 0.55,
    "acquire_steer": 0.35,
    "acquire_throttle": 0.20,
    "acquire_good_n": 4,

    "gate_clear_ms": 120,
}

#Border push function
def border_push(motors, sensors, ctx, cfg = border_push_cfg):

    t_last = ticks_ms()

    phase = "start_box"
    phase_t0 = ticks_ms()

    see_white_count = 0
    border_ok = 0
    acquire_ok = 0

    if ctx is None:
        ctx = MotionContext()

    while True:
        print(phase)

        #Get sensor readings and do fixed-rate ticking
        t_now = fixed_rate_tick(t_last, cfg["period_ms"])
        t_last = t_now
        black, white, sumw, good_line, inter_raw, corner_raw = sensors.sense()

        #First phase: start_box
        if phase == "start_box":

            # Drive forward until we see white for a few consecutive readings, indicating that we've exited the start box.
            motors.arcade(cfg["start_throttle"], 0.0)

            if sumw >= 1:
                see_white_count += 1
                if see_white_count >= cfg["start_see_white_n"]:
                    phase = "border_push"
                    phase_t0 = t_now
                    see_white_count = 0
                    border_ok = 0
            else:
                see_white_count = 0

            continue

        #Second phase: border_push
        if phase == "border_push":
            if ticks_diff(t_now, phase_t0) >= cfg["border_timeout_ms"]:
                motors.arcade(0.0, 0.0)
                raise RuntimeError("border_push: BORDER_PUSH timeout")

            #Bias toward side with more white
            w = (-2 * white[0]) + (-1 * white[1]) + (1 * white[2]) + (2 * white[3])

            if w < 0:
                ctx.last_search_dir = -1
            elif w > 0:
                ctx.last_search_dir = +1

            steer = clamp(
                cfg["border_bias_k"] * w,
                -cfg["border_bias_max"],
                +cfg["border_bias_max"],
            )

            #Steer toward the line while pushing forward at a constant throttle.
            motors.arcade(cfg["border_throttle"], steer)

            #Leave border when wide patch is gone
            if sumw <= 2:
                border_ok += 1
                if border_ok >= cfg["border_exit_n"]:
                    phase = "acquire"
                    phase_t0 = t_now
                    border_ok = 0
                    acquire_ok = 0
            else:
                border_ok = 0

            continue

        
        #Third phase: acquire
        if phase == "acquire":
            if ticks_diff(t_now, phase_t0) >= cfg["acquire_timeout_ms"]:
                motors.arcade(0.0, 0.0)
                raise RuntimeError("border_push: ACQUIRE timeout")

            #If we drift back into wide white, return to border_push
            if sumw >= 3:
                phase = "border_push"
                phase_t0 = t_now
                border_ok = 0
                continue

            # No white visible: search in last known direction
            if sumw == 0:
                motors.arcade(0.0, ctx.last_search_dir * cfg["acquire_search_steer"])
                continue

            #Steer toward white using all sensors
            w = (-2 * white[0]) + (-1 * white[1]) + (1 * white[2]) + (2 * white[3])

            if w < 0:
                steer = -cfg["acquire_steer"]
                ctx.last_search_dir = -1
            elif w > 0:
                steer = +cfg["acquire_steer"]
                ctx.last_search_dir = +1
            else:
                steer = 0.0

            motors.arcade(cfg["acquire_throttle"], steer)

            if good_line:
                acquire_ok += 1
                if acquire_ok >= cfg["acquire_good_n"]:
                    phase = "gate_clear"
                    phase_t0 = t_now
            else:
                acquire_ok = 0

            continue

        #Fourth phase: gate_clear
        if phase == "gate_clear":
            motors.arcade(cfg["border_throttle"], 0.0)

            # Wait long enough to clear the gate geometry, then exit.
            if ticks_diff(t_now, phase_t0) >= cfg["gate_clear_ms"]:
                return None



# ---------------------------------------------------------------------

#Turn left/right parameters. Adjust as needed.
turn_cfg = {
    "period_ms": 20,              # control loop period (50 Hz)

    "straight_throttle": 0.35,    # forward speed during approach phase
    "turn_approach_ms": 250,      # short forward push before spinning

    "turn_spin_steer": 0.75,      # spin strength: left = -0.75, right = +0.75
    "turn_min_spin_ms": 600,      # minimum time before line reacquire is allowed
    "turn_timeout_ms": 1200,      # fail-safe timeout for spin phase

    "turn_reacquire_n": 4,        # consecutive good_line reads needed to finish turn

    "search_throttle": 0.4,
    "search_steer": 0.4

}

#Turn function
def turn(direction, motors, sensors, cfg=turn_cfg, ctx=None):

    if ctx is None:
        ctx = MotionContext()

    # Validate direction input
    if direction not in ("left", "right"):
        raise ValueError("direction must be 'left' or 'right'")

    # Determine spin direction: left turn is negative steer, right turn is positive steer
    spin_sign = -1 if direction == "left" else +1

    phase = "approach"
    t_last = ticks_ms()
    phase_t0 = t_last
    reacquire_ok = 0

    while True:
        #Get sensor readings and do fixed-rate ticking
        t_now = fixed_rate_tick(t_last, cfg["period_ms"])
        t_last = t_now
        black, white, sumw, good_line, inter_raw, corner_raw = sensors.sense()
        err = middle_error_white_line(black)

        if err is not None:
            if err > 0:
                ctx.last_search_dir = +1
            elif err < 0:
                ctx.last_search_dir = -1

        #First phase: approach
        if phase == "approach":

            #Drive straight for a fixed time before starting the turn
            motors.arcade(cfg["straight_throttle"], 0.0)
            if ticks_diff(t_now, phase_t0) >= cfg["turn_approach_ms"]:
                phase = "spin"
                phase_t0 = t_now
            continue

        #Second phase: spin
        if phase == "spin":
            
            #Spin in place toward the turn direction at a fixed steer value.
            motors.arcade(0.0, spin_sign * cfg["turn_spin_steer"])
            
            #Timeout if we take too long to see the line again, to prevent getting stuck in the spin phase.
            if ticks_diff(t_now, phase_t0) >= cfg["turn_timeout_ms"]:
                default_stop(motors)
                raise RuntimeError("turn timeout in spin")

            #Once we see the line again after the minimum spin time, transition to align phase.
            #If we're going left, we expect to see white on the middle-left sensor first. 
            #If we're going right, we expect to see white on the middle-right sensor first.
            if ticks_diff(t_now, phase_t0) >= cfg["turn_min_spin_ms"] and ((spin_sign == -1 and white[1]==1) or (spin_sign == +1 and white[2]==1)):
                phase = "align"
                phase_t0 = t_now
                reacquire_ok = 0
            continue

        if phase == "align":

            #If we lose the line entirely, return to spin phase to reacquire.
            if err is None:
                default_search_drive(motors, ctx, cfg)
                reacquire_ok = 0
                continue

            #Steer toward the line using PD control.
            thr, steer = pd_follow(err, ctx.last_err, cfg["period_ms"]/1000.0)
            motors.arcade(thr, steer)
            ctx.last_err = err

            # Once we have a good line for a few consecutive readings, we can be confident that we've completed the turn and reacquired on the new corridor, so we can exit.
            if good_line:
                reacquire_ok += 1
                if reacquire_ok >= cfg["turn_reacquire_n"]:
                    default_stop(motors)
                    return None
            else:
                reacquire_ok = 0


# ---------------------------------------------------------------------

#Straight through intersection parameters. Adjust as needed.
straight_cfg = {
    "period_ms": 20,                # 50 Hz loop

    "straight_throttle": 0.45,      # forward drive speed
    "straight_kp": 0.55,            # proportional steering correction
    "straight_max_steer": 0.25,     # clamp on steering during straight move

    "straight_min_ms": 250,         # minimum time before move can complete
    "straight_timeout_ms": 1400,    # fail-safe timeout

    "inter_exit_n": 3,              # consecutive readings outside intersection
    "straight_exit_sumw_max": 2,    # "sumw <= 2" from your original code
}

#Straight through intersection function
def straight(motors, sensors, cfg=straight_cfg, ctx=None):

    if ctx is None:
        ctx = MotionContext()

    t_last = ticks_ms()
    phase_t0 = t_last
    straight_out = 0

    while True:

        #Get sensor readings and do fixed-rate ticking
        t_now = fixed_rate_tick(t_last, cfg["period_ms"])
        t_last = t_now
        black, white, sumw, good_line, inter_raw, corner_raw = sensors.sense()
        err = middle_error_white_line(black)

        if good_line:
            ctx.t_last_good_line = t_now

        inter_cond, corner_cond, event_cond, recent_good = compute_event(
            t_now, ctx, corner_raw, inter_raw
        )

        #Total elapsed time since we started the straight primitive
        elapsed = ticks_diff(t_now, phase_t0)

        #Steer toward the line using proportional control, with a clamp on maximum steer to prevent over-correction.
        steer = 0.0
        if err is not None:
            steer = clamp(
                cfg["straight_kp"] * err,
                -cfg["straight_max_steer"],
                +cfg["straight_max_steer"]
            )

        motors.arcade(cfg["straight_throttle"], steer)

        #Debounce exit condition: require several consecutive readings indicating we've left the intersection geometry, 
        if (not inter_cond) and (sumw <= 2):
            straight_out += 1
        else:
            straight_out = 0

        #If we've been out of the intersection for enough consecutive readings and we've been moving for at least the minimum time, we can assume we've cleared the intersection and exit.
        if (elapsed >= cfg["straight_min_ms"] and straight_out >= cfg["inter_exit_n"]  and good_line):
            default_stop(motors)
            return None

        #Otherwise, timeout if we take too long
        if elapsed >= cfg["straight_timeout_ms"]:
            ctx.last_event_t = t_now
            ctx.good_rearm = 0
            default_stop(motors)
            raise RuntimeError("straight timeout")
        

# ---------------------------------------------------------------------


#180 parameters. Adjust as needed.

do_180_cfg = {
    "period_ms": 20,

    # Special dead-end nodes (20, 28)
    "dead_end_forward_throttle": 0.45,
    "dead_end_forward_ms": 1100,
    "dead_end_spin_steer": 0.80,
    "dead_end_min_spin_ms": 1500,      # deliberately larger grace time
    "dead_end_timeout_ms": 4000,

    # Generic 180 elsewhere
    "generic_reverse_throttle": -0.50,
    "generic_reverse_ms": 350,
    "generic_spin_steer": 0.80,
    "generic_min_spin_ms": 1000,
    "generic_timeout_ms": 4000,

    # Shared align phase
    "align_search_throttle": 0.18,
    "align_search_steer": 0.55,
    "align_reacquire_n": 3,
    "align_timeout_ms": 900,
}


def do_180(node, motors, sensors, cfg=do_180_cfg, ctx=None):

    """
    180 behaviour:

    - If node == 20:
        forward a bit, then spin right (+1), then align
    - If node == 28:
        forward a bit, then spin left (-1), then align
    - Else:
        reverse a bit, then spin (default right), then align
    """

    if ctx is None:
        ctx = MotionContext()

    # Choose special-case behaviour
    if node == 20:
        special_dead_end = True
        spin_sign = +1
        phase = "commit_forward"
        commit_throttle = cfg["dead_end_forward_throttle"]
        commit_ms = cfg["dead_end_forward_ms"]
        spin_steer = cfg["dead_end_spin_steer"]
        min_spin_ms = cfg["dead_end_min_spin_ms"]
        spin_timeout_ms = cfg["dead_end_timeout_ms"]

    elif node == 28:
        special_dead_end = True
        spin_sign = -1
        phase = "commit_forward"
        commit_throttle = cfg["dead_end_forward_throttle"]
        commit_ms = cfg["dead_end_forward_ms"]
        spin_steer = cfg["dead_end_spin_steer"]
        min_spin_ms = cfg["dead_end_min_spin_ms"]
        spin_timeout_ms = cfg["dead_end_timeout_ms"]

    else:
        special_dead_end = False
        spin_sign = +1   # default: just spin right for generic 180
        phase = "commit_reverse"
        commit_throttle = cfg["generic_reverse_throttle"]
        commit_ms = cfg["generic_reverse_ms"]
        spin_steer = cfg["generic_spin_steer"]
        min_spin_ms = cfg["generic_min_spin_ms"]
        spin_timeout_ms = cfg["generic_timeout_ms"]

    t_last = ticks_ms()
    phase_t0 = t_last
    reacquire_ok = 0

    while True:

        # Fixed-rate tick + sensing
        t_now = fixed_rate_tick(t_last, cfg["period_ms"])
        t_last = t_now
        black, white, sumw, good_line, inter_raw, corner_raw = sensors.sense()
        err = middle_error_white_line(black)

        if good_line:
            ctx.t_last_good_line = t_now

        if err is not None:
            if err > 0:
                ctx.last_search_dir = +1
            elif err < 0:
                ctx.last_search_dir = -1

        #Phase 1: commit_forward or commit_reverse
        if phase == "commit_forward":
            motors.arcade(commit_throttle, 0.0)

            if ticks_diff(t_now, phase_t0) >= commit_ms:
                phase = "spin"
                phase_t0 = t_now
                reacquire_ok = 0

            continue

        if phase == "commit_reverse":
            motors.arcade(commit_throttle, 0.0)

            if ticks_diff(t_now, phase_t0) >= commit_ms:
                phase = "spin"
                phase_t0 = t_now
                reacquire_ok = 0

            continue

        #Phase 2: spin
        if phase == "spin":
            motors.arcade(0.0, spin_sign * spin_steer)

            if ticks_diff(t_now, phase_t0) >= spin_timeout_ms:
                default_stop(motors)
                raise RuntimeError("do_180 timeout in spin at node {}".format(node))

            # Do not allow reacquisition too early
            if ticks_diff(t_now, phase_t0) < min_spin_ms:
                continue

            # Transition to align once a plausible line is visible
            # For dead-end cases, be stricter: require valid err.
            if special_dead_end:
                if err is not None:
                    phase = "align"
                    phase_t0 = t_now
                    reacquire_ok = 0
            else:
                # Generic case: any plausible visible line is enough to try align
                if white[1] == 1 or white[2] == 1:
                    phase = "align"
                    phase_t0 = t_now
                    reacquire_ok = 0

            continue

        # Phase 3: align
        if phase == "align":
            if ticks_diff(t_now, phase_t0) >= cfg["align_timeout_ms"]:
                default_stop(motors)
                raise RuntimeError("do_180 timeout in align at node {}".format(node))

            if err is None:
                motors.arcade(
                    cfg["align_search_throttle"],
                    spin_sign * cfg["align_search_steer"]
                )
                reacquire_ok = 0
                continue

            thr, steer = pd_follow(err, ctx.last_err, cfg["period_ms"] / 1000.0)
            motors.arcade(thr, steer)
            ctx.last_err = err

            if good_line:
                reacquire_ok += 1
                if reacquire_ok >= cfg["align_reacquire_n"]:
                    default_stop(motors)
                    return None
            else:
                reacquire_ok = 0

# ---------------------------------------------------------------------

# Grab left/right parameters. Adjust as needed.
grab_cfg = {
    "period_ms": 20,

    # forward into branch
    "grab_forward_throttle": 0.22,
    "grab_forward_ms": 100,

    # reverse out to node
    "grab_reverse_throttle": -0.22,
    "grab_reverse_exit_n": 2,
    "grab_reverse_timeout_ms": 1200,

    # optional settle after closing gripper
    "grab_close_wait_ms": 100,
}


def grab(motors, sensors, command, task_sensors, cfg=grab_cfg, ctx=None):


    """
    Behaviour:
    1. turn into the branch using normal turn(direction)
    2. drive forward with PD for a short fixed time
    3. close gripper / attempt pickup
    4. reverse out until sumw >= 3 for N consecutive readings
    5. return grab result

    Returns:
        {"status": "grab_ok", "colour": "..."}
        or
        "grab_fail"
    """

    if command == "grab_right":
        direction = "right"
    elif command == "grab_left":
        direction = "left"
    else:
        direction = None

    if ctx is None:
        ctx = MotionContext()

    if direction not in ("left", "right"):
        raise ValueError("grab(direction): direction must be 'left' or 'right'")

    # Phase 1: turn into branch
    turn(direction, motors, sensors, ctx=ctx)

    # Phase 2: drive forward briefly with PD
    t_last = ticks_ms()
    phase_t0 = t_last

    while True:
        t_now = fixed_rate_tick(t_last, cfg["period_ms"])
        t_last = t_now

        black, white, sumw, good_line, inter_raw, corner_raw = sensors.sense()
        err = middle_error_white_line(black)

        if err is None:
            default_search_drive(motors, ctx, {
                "search_throttle": cfg["grab_forward_throttle"],
                "search_steer": 0.40,
            })
        else:
            thr, steer = pd_follow(err, ctx.last_err, cfg["period_ms"] / 1000.0)
            motors.arcade(cfg["grab_forward_throttle"], steer)
            ctx.last_err = err

            if err > 0:
                ctx.last_search_dir = +1
            elif err < 0:
                ctx.last_search_dir = -1

        if ticks_diff(t_now, phase_t0) >= cfg["grab_forward_ms"]:
            default_stop(motors)
            break

    # Phase 3: close gripper / attempt pickup
    # Replace this with your real gripper code
    task_sensors.close_gripper()
    sleep_ms(cfg["grab_close_wait_ms"])

    # Phase 4: reverse out until node patch seen again
    t_last = ticks_ms()
    phase_t0 = t_last
    reverse_ok = 0

    while True:
        t_now = fixed_rate_tick(t_last, cfg["period_ms"])
        t_last = t_now

        black, white, sumw, good_line, inter_raw, corner_raw = sensors.sense()

        motors.arcade(cfg["grab_reverse_throttle"], 0.0)

        if sumw >= 3:
            reverse_ok += 1
            if reverse_ok >= cfg["grab_reverse_exit_n"]:
                default_stop(motors)
                break
        else:
            reverse_ok = 0

        if ticks_diff(t_now, phase_t0) >= cfg["grab_reverse_timeout_ms"]:
            default_stop(motors)
            return "grab_fail"

    # Phase 5: classify result
    colour = task_sensors.classify_reel()

    if colour is None:
        return "grab_fail"

    return {"status": "grab_ok", "colour": colour}

# ---------------------------------------------------------------------

#Scan left/right parameters. Adjust as needed.  
scan_cfg = {}

#Scan functions. Should eventually look into the left/right branch and return "scan_empty" or "scan_found" based on whether a reel is detected.
def scan_left():

    return "scan_empty"

def scan_right():

    return "scan_empty"

# ---------------------------------------------------------------------

#Drop parameters. Adjust as needed.  

#Drop function
def drop_reel():
    return None

# ---------------------------------------------------------------------

#Stop function. 
def stop(motors):
    while(True):
        default_stop(motors)
        sleep_ms(50)

