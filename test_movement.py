from hw.line import LineSensors
from hw.motors import DCMotor, MotorPair
import config

from control.movement_clean import (
    MotionContext,
    border_push,
    turn,
    straight,
    do_180,
    grab,
    stop,
)

# replace with your real task sensors if needed
from hw.task_sensors import TaskSensors


def build_hw():
    sensors = LineSensors(config.LINE_PINS, invert=config.LINE_INVERT)

    left = DCMotor(
        config.MOTOR_L_DIR,
        config.MOTOR_L_PWM,
        pwm_freq_hz=config.MOTOR_PWM_FREQ_HZ,
        invert=config.MOTOR_L_INVERT,
    )
    right = DCMotor(
        config.MOTOR_R_DIR,
        config.MOTOR_R_PWM,
        pwm_freq_hz=config.MOTOR_PWM_FREQ_HZ,
        invert=config.MOTOR_R_INVERT,
    )
    motors = MotorPair(left, right)

    task_sensors = TaskSensors()
    ctx = MotionContext()

    return motors, sensors, task_sensors, ctx


def wait_for_enter():
    input("Press Enter to run test...")


def main():
    motors, sensors, task_sensors, ctx = build_hw()

    while True:
        print("1  border_push")
        print("2  turn left")
        print("3  turn right")
        print("4  straight")
        print("5  do_180 node 20")
        print("6  do_180 node 28")
        print("7  do_180 generic")
        print("8  grab left")
        print("9  grab right")
        print("0  stop and exit")

        choice = input('enter a motion')

        try:
            if choice == "1":
                border_push(motors, sensors, None)
                stop(motors)
                break
            
            elif choice == "2":
                turn("left", motors, sensors)
                stop(motors)
                print("turn left done")
                break

            elif choice == "3":
                turn("right", motors, sensors)
                stop(motors)
                break

            elif choice == "4":
                straight(motors, sensors)
                stop(motors)
                print("straight done")

            elif choice == "5":
                do_180(20, motors, sensors)
                stop(motors)
                print("do_180 node 20 done")

            elif choice == "6":
                do_180(28, motors, sensors)
                stop(motors)
                print("do_180 node 28 done")

            elif choice == "7":
                do_180(999, motors, sensors)
                stop(motors)
                print("generic do_180 done")

            elif choice == "8":
                result = grab(motors, sensors, "grab_left", task_sensors)
                stop(motors)
                print("grab left result:", result)

            elif choice == "9":
                result = grab(motors, sensors, "grab_right", task_sensors)
                stop(motors)
                print("grab right result:", result)

            elif choice == "0":
                stop(motors)
                break

            else:
                print("Invalid choice")

        except Exception as e:
            stop(motors)
            print("ERROR:", e)


main()
