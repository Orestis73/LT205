from utime import ticks_ms, ticks_diff, sleep_ms
from machine import Pin


class TaskSensors:
    def __init__(self):
        # Physical pin 21 = GP16
        self.start_btn = Pin(16, Pin.IN, Pin.PULL_DOWN)

    def start_pressed(self):
        return self.start_btn.value() == 1
    
    def close_gripper(self):
        return None
    
    def classify_reel(self):
        return None
    
def wait_until_button_pressed(task_sensors, motors=None):
    print("WAITING FOR START BUTTON")
    while not task_sensors.start_pressed():
        if motors is not None:
            motors.arcade(0.0, 0.0)
        sleep_ms(20)
    print("START BUTTON PRESSED")