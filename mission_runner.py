from simplified_navigator import Navigator
from utime import ticks_ms, ticks_diff, sleep_ms
from control.movement_clean import border_push, straight, turn, do_180, grab, drop_reel, stop
from control.pd import middle_error_white_line, pd_follow
import config
from hw.line import LineSensors
from hw.motors import DCMotor, MotorPair
from control.runtime_variables import Mem
from hw.task_sensors import TaskSensors, wait_until_button_pressed, scan


def fixed_rate_tick(t_last, period_ms):
    # Keep the main loop running at a roughly fixed rate
    t_now = ticks_ms()
    dt = ticks_diff(t_now, t_last)
    if dt < period_ms:
        sleep_ms(period_ms - dt)
        t_now = ticks_ms()
    return t_now


def compute_event(t_now, mem, corner_raw, inter_cond):
    # Check if we have seen a proper line recently
    recent_good = ticks_diff(t_now, mem.t_last_good_line) <= config.RECENT_GOOD_LINE_MS

    # Only count a corner if we were recently on the line
    corner_cond = recent_good and corner_raw and (not inter_cond)

    # An event is either an intersection or a valid corner
    event_cond = inter_cond or corner_cond

    return inter_cond, corner_cond, event_cond, recent_good


def main():

    # Set up the line sensors
    sensors = LineSensors(config.LINE_PINS, invert=config.LINE_INVERT)

    # Set up left and right motors, then combine them into one motor pair
    left = DCMotor(
        config.MOTOR_L_DIR,
        config.MOTOR_L_PWM,
        pwm_freq_hz=config.MOTOR_PWM_FREQ_HZ,
        invert=config.MOTOR_L_INVERT
    )
    right = DCMotor(
        config.MOTOR_R_DIR,
        config.MOTOR_R_PWM,
        pwm_freq_hz=config.MOTOR_PWM_FREQ_HZ,
        invert=config.MOTOR_R_INVERT
    )
    motors = MotorPair(left, right)

    # Timing values for the control loop
    period_ms = int(1000 / config.LOOP_HZ)
    dt_s = period_ms / 1000.0

    # Set up navigator and memory/state object
    navigator = Navigator(expected_total_reels=12)
    mem = Mem()
    t_last = ticks_ms()

    # Set up task sensors, then wait for button press before starting
    task_sensors = TaskSensors()
    wait_until_button_pressed(task_sensors, motors)

    # Initial move to get out of the start box and onto the route
    border_push(motors, sensors, None)

    # This stores the result of the last action and is passed back to the navigator
    last_result = None

    while True:
        # Ask the navigator what command to do next
        command = navigator.next_command(last_result)
        node = navigator.current_node

        if command == "follow":
            # Main line-following mode until we detect a navigation event
            t_last = ticks_ms()
            while True:
                t_now = fixed_rate_tick(t_last, period_ms)
                t_last = t_now

                # Read line sensors
                black, white, sumw, good_line, inter_raw, corner_raw = sensors.sense()

                # Calculate line-following error
                err = middle_error_white_line(black)

                # Check whether this reading should count as an event
                inter_cond, corner_cond, event_cond, recent_good = compute_event(
                    t_now, mem, corner_raw, inter_raw
                )

                # Update last time we had a good line reading
                if good_line:
                    mem.t_last_good_line = t_now

                # Rearm event detection only if line detection has been stable enough
                if recent_good:
                    if mem.good_rearm < config.EVENT_REARM_MAX:
                        mem.good_rearm += 1
                else:
                    if mem.good_rearm > 0:
                        mem.good_rearm -= 1

                # Only allow a new event if cooldown has passed and line has rearmed
                can_detect = (
                    (ticks_diff(t_now, mem.last_event_t) >= config.EVENT_COOLDOWN_MS)
                    and (mem.good_rearm >= config.EVENT_REARM_N)
                )

                if can_detect:
                    if event_cond:
                        mem.event_in_count += 1

                        # Require repeated confirmation before accepting the event
                        if mem.event_in_count >= config.EVENT_ENTER_N:
                            mem.event_in_count = 0
                            mem.last_event_t = t_now
                            mem.good_rearm = 0
                            last_result = "event"
                            break
                    else:
                        mem.event_in_count = 0

                # Normal PD line following
                thr, steer = pd_follow(err, mem.last_err, dt_s)
                motors.arcade(thr, steer)
                mem.last_err = err

        elif command == "right":
            # Execute a right turn, then report completion
            turn("right", motors, sensors)
            last_result = "done"

        elif command == "left":
            # Execute a left turn, then report completion
            turn("left", motors, sensors)
            last_result = "done"

        elif command == "straight":
            # Drive straight through the junction
            straight(motors, sensors)
            last_result = "done"

        elif command == "180":
            # Perform a 180 turn using the current node if needed
            do_180(node, motors, sensors)
            last_result = "done"

        elif command == "scan_left":
            # Temporary logic used instead of real scanning
            # Nodes 23-28 are treated as found
            if node >= 23 and node <= 28:
                last_result = "scan_found"
            else:
                last_result = "scan_empty"

        elif command == "scan_right":
            # Temporary logic used instead of real scanning
            # Nodes 15-20 are treated as found
            if node >= 15 and node <= 20:
                last_result = "scan_found"
            else:
                last_result = "scan_empty"

        elif command == "grab_left":
            # Try to grab from the left branch
            last_result = grab(motors, sensors, command, task_sensors)

        elif command == "grab_right":
            # Try to grab from the right branch
            last_result = grab(motors, sensors, command, task_sensors)

        elif command == "drop":
            # Open the gripper to drop the reel
            task_sensors.open_gripper()
            last_result = "drop_done"

        elif command == "finished":
            # Stop the robot and exit the loop
            stop(motors)
            break

        else:
            raise ValueError("Unknown command: {}".format(command))


main()