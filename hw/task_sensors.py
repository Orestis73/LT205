# task_sensors.py

from utime import ticks_ms, ticks_diff, sleep_ms
from machine import Pin, I2C
from libs.VL53L0X.VL53L0X import VL53L0X


def default_stop(motors):
    motors.arcade(0.0, 0.0)


class TaskSensors:
    START_BTN_PIN = 16

    # LEFT sensor: GP8 / GP9
    LEFT_I2C_ID = 0
    LEFT_SDA_PIN = 8
    LEFT_SCL_PIN = 9

    # RIGHT sensor: change these if needed
    RIGHT_I2C_ID = 0
    RIGHT_SDA_PIN = 20
    RIGHT_SCL_PIN = 21

    SCAN_DURATION_MS = 2000
    SCAN_SAMPLE_PERIOD_MS = 50

    # Right side still as previously estimated
    RIGHT_FOUND_THRESHOLD_MM = 220

    # Left side from your real measurements
    LEFT_FOUND_THRESHOLD_NODES_23_TO_28_MM = 183
    LEFT_FOUND_THRESHOLD_OTHER_MM = 126

    # Reject obvious bogus VL53L0X values
    MIN_VALID_MM = 20
    MAX_VALID_MM = 1000

    def __init__(self):
        self.start_btn = Pin(self.START_BTN_PIN, Pin.IN, Pin.PULL_DOWN)

    def start_pressed(self):
        return self.start_btn.value() == 1

    def close_gripper(self):
        return None

    def classify_reel(self):
        return None

    def _make_left_sensor(self):
        i2c_bus = I2C(
            id=self.LEFT_I2C_ID,
            sda=Pin(self.LEFT_SDA_PIN),
            scl=Pin(self.LEFT_SCL_PIN),
        )
        sensor = VL53L0X(i2c_bus)
        sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[0], 18)
        sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[1], 14)
        return sensor

    def _make_right_sensor(self):
        i2c_bus = I2C(
            id=self.RIGHT_I2C_ID,
            sda=Pin(self.RIGHT_SDA_PIN),
            scl=Pin(self.RIGHT_SCL_PIN),
        )
        sensor = VL53L0X(i2c_bus)
        sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[0], 18)
        sensor.set_Vcsel_pulse_period(sensor.vcsel_period_type[1], 14)
        return sensor

    def _scan_sensor_for_duration(self, sensor, motors=None, duration_ms=None, sample_period_ms=None):
        if duration_ms is None:
            duration_ms = self.SCAN_DURATION_MS
        if sample_period_ms is None:
            sample_period_ms = self.SCAN_SAMPLE_PERIOD_MS

        readings = []
        t0 = ticks_ms()

        if motors is not None:
            default_stop(motors)

        sensor.start()
        try:
            while ticks_diff(ticks_ms(), t0) < duration_ms:
                if motors is not None:
                    default_stop(motors)

                try:
                    d = sensor.read()

                    # Reject garbage like 8191
                    if d is not None and self.MIN_VALID_MM <= d <= self.MAX_VALID_MM:
                        readings.append(d)
                        print("Distance = {}mm".format(d))
                except Exception as e:
                    print("VL53L0X read error:", e)

                sleep_ms(sample_period_ms)
        finally:
            sensor.stop()
            if motors is not None:
                default_stop(motors)

        return readings

    @staticmethod
    def _median(values):
        if not values:
            return None
        vals = sorted(values)
        n = len(vals)
        mid = n // 2
        if n % 2 == 1:
            return vals[mid]
        return (vals[mid - 1] + vals[mid]) / 2

    def scan_right(self, motors=None):
        sensor = self._make_right_sensor()
        readings = self._scan_sensor_for_duration(sensor, motors=motors)

        if not readings:
            print("RIGHT SCAN: no valid readings")
            return "scan_empty"

        med = self._median(readings)
        print("RIGHT SCAN median =", med)

        # Keep your current right-side convention for now
        if med < self.RIGHT_FOUND_THRESHOLD_MM:
            return "scan_found"
        return "scan_empty"

    def scan_left(self, node, motors=None):
        sensor = self._make_left_sensor()
        readings = self._scan_sensor_for_duration(sensor, motors=motors)

        if not readings:
            print("LEFT SCAN: no valid readings")
            return "scan_empty"

        med = self._median(readings)
        print("LEFT SCAN median =", med)

        if 23 <= node <= 28:
            threshold = self.LEFT_FOUND_THRESHOLD_NODES_23_TO_28_MM
        else:
            threshold = self.LEFT_FOUND_THRESHOLD_OTHER_MM

        print("LEFT SCAN threshold =", threshold)

        # LEFT SENSOR LOGIC IS REVERSED COMPARED TO BEFORE:
        # reel present => bigger distance
        if med >= threshold:
            return "scan_found"
        return "scan_empty"


def scan(node, task_sensors, command, motors=None):
    print("SCAN START:", command, "node =", node)

    if motors is not None:
        default_stop(motors)

    if command == "scan_left":
        result = task_sensors.scan_left(node=node, motors=motors)
    elif command == "scan_right":
        result = task_sensors.scan_right(motors=motors)
    else:
        raise ValueError("Unknown scan command: {}".format(command))

    if motors is not None:
        default_stop(motors)

    print("SCAN RESULT:", result)
    return result


def wait_until_button_pressed(task_sensors, motors=None):
    print("WAITING FOR START BUTTON")
    while not task_sensors.start_pressed():
        if motors is not None:
            default_stop(motors)
        sleep_ms(20)
    print("START BUTTON PRESSED")