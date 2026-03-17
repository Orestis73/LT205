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
    t_now = ticks_ms()
    dt = ticks_diff(t_now, t_last)
    if dt < period_ms:
        sleep_ms(period_ms - dt)
        t_now = ticks_ms()
    return t_now


def compute_event(t_now, mem, corner_raw, inter_cond):
    # recent good line gate
    recent_good = ticks_diff(t_now, mem.t_last_good_line) <= config.RECENT_GOOD_LINE_MS

    # corner only counts if we were on the line recently
    corner_cond = recent_good and corner_raw and (not inter_cond)

    # event condition: either effective intersection or corner
    event_cond = inter_cond or corner_cond

    return inter_cond, corner_cond, event_cond, recent_good


def main():

    #Initialize sensors + motors
    sensors = LineSensors(config.LINE_PINS, invert=config.LINE_INVERT)

    left = DCMotor(config.MOTOR_L_DIR, config.MOTOR_L_PWM, pwm_freq_hz=config.MOTOR_PWM_FREQ_HZ, invert=config.MOTOR_L_INVERT)
    right = DCMotor(config.MOTOR_R_DIR, config.MOTOR_R_PWM, pwm_freq_hz=config.MOTOR_PWM_FREQ_HZ, invert=config.MOTOR_R_INVERT)
    motors = MotorPair(left, right)

    # timing
    period_ms = int(1000 / config.LOOP_HZ)
    dt_s = period_ms / 1000.0

    # mission
    navigator = Navigator(expected_total_reels=4)
    mem = Mem()
    t_last = ticks_ms()

    task_sensors = TaskSensors()
    wait_until_button_pressed(task_sensors, motors)

    border_push(motors, sensors, None)

    last_result = None

    while True:
        command = navigator.next_command(last_result)
        node = navigator.current_node

        if command == "follow":
            t_last = ticks_ms()
            while True:
                t_now = fixed_rate_tick(t_last, period_ms)  # Adjust period as needed
                t_last = t_now
                black, white, sumw, good_line, inter_raw, corner_raw = sensors.sense()
                err = middle_error_white_line(black)
                inter_cond, corner_cond, event_cond, recent_good = compute_event(t_now, mem, corner_raw, inter_raw)

                if good_line:
                    mem.t_last_good_line = t_now

                if recent_good:
                    if mem.good_rearm <  config.EVENT_REARM_MAX:
                        mem.good_rearm += 1
                else:
                    if mem.good_rearm > 0:
                        mem.good_rearm -= 1

                
                can_detect = ((ticks_diff(t_now, mem.last_event_t) >= config.EVENT_COOLDOWN_MS) and 
                              (mem.good_rearm >= config.EVENT_REARM_N))
                

                if can_detect:
                    if event_cond:
                        mem.event_in_count += 1

                        if mem.event_in_count >= config.EVENT_ENTER_N:
                            mem.event_in_count = 0
                            mem.last_event_t = t_now
                            mem.good_rearm = 0
                            last_result = "event"
                            break
                    else:
                        mem.event_in_count = 0

                thr, steer = pd_follow(err, mem.last_err, dt_s)
                motors.arcade(thr, steer)
                mem.last_err = err

        elif command == "right":
            turn("right", motors, sensors)
            last_result = "done"

        elif command == "left":
            turn("left", motors, sensors)
            last_result = "done"

        elif command == "straight":
            straight(motors, sensors)
            last_result = "done"

        elif command == "180":
            do_180(node, motors, sensors)
            last_result = "done"

        elif command == "scan_left":
            last_result = scan(node, task_sensors, command, motors)  # "scan_empty" or "scan_found"
            last_result = "scan_empty"

        elif command == "scan_right":
            last_result = scan(node, task_sensors, command, motors)  # "scan_empty" or "scan_found"
            last_result = "scan_empty"

        elif command == "grab_left":
            last_result = grab(motors, sensors, command, task_sensors)   # {"status": "grab_ok", "colour": "blue"} or "grab_fail"

        elif command == "grab_right":
            last_result = grab(motors, sensors, command, task_sensors)   # {"status": "grab_ok", "colour": "blue"} or "grab_fail"

        elif command == "drop":
            drop_reel(task_sensors)
            last_result = "drop_done"

        elif command == "finished":
            stop()
            break

        else:
            raise ValueError("Unknown command: {}".format(command))


main()
