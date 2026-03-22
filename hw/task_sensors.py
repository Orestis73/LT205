from machine import Pin, PWM, I2C, ADC
from utime import ticks_ms, ticks_diff, sleep_ms

from libs.VL53L0X.VL53L0X import VL53L0X


def default_stop(motors):
    motors.arcade(0.0, 0.0)


class Grabber:
    def __init__(self, pin1, pin2):
        self.servo_pin1 = pin1
        self.servo_pin2 = pin2

        self.pwm_pin1 = PWM(Pin(self.servo_pin1), 100)
        self.pwm_pin2 = PWM(Pin(self.servo_pin2), 100)

        self.servo1_deg = 0
        self.servo2_deg = 0

    def write_servo1(self, deg):
        u16_level1 = int(9500 + deg * 48.5)
        self.pwm_pin1.duty_u16(u16_level1)
        self.servo1_deg = deg

    def write_servo2(self, deg):
        u16_level2 = int(9800 + deg * 48.5)
        self.pwm_pin2.duty_u16(u16_level2)
        self.servo2_deg = deg

    def reset(self):
        self.write_servo1(0)
        self.write_servo2(0)

    def grab(self, lift=True):
        self.write_servo1(12)
        if lift:
            self.write_servo2(self.servo2_deg - 5)

    def opn(self, lift=True):
        if lift:
            self.write_servo2(-5)
        self.write_servo1(-10)

    def lift(self, deg):
        self.write_servo2(-deg)



class TaskSensors:
    START_BTN_PIN = 16

    # LEFT VL53L0X sensor
    LEFT_I2C_ID = 0
    LEFT_SDA_PIN = 8
    LEFT_SCL_PIN = 9

    # RIGHT VL53L0X sensor
    RIGHT_I2C_ID = 0
    RIGHT_SDA_PIN = 20
    RIGHT_SCL_PIN = 21

    # Ultrasonic analog output pin
    ULTRASONIC_ADC_PIN = 26
    ULTRASONIC_VCC = 3.3
    ULTRASONIC_MAX_RANGE_CM = 500.0

    SCAN_DURATION_MS = 2000
    SCAN_SAMPLE_PERIOD_MS = 50

    RIGHT_FOUND_THRESHOLD_MM = 220
    LEFT_FOUND_THRESHOLD_NODES_23_TO_28_MM = 190
    LEFT_FOUND_THRESHOLD_OTHER_MM = 126

    MIN_VALID_MM = 20
    MAX_VALID_MM = 1000

    ULTRA_MIN_VALID_CM = 2.0
    ULTRA_MAX_VALID_CM = 500.0

    def __init__(self, grabber=None):
        self.start_btn = Pin(self.START_BTN_PIN, Pin.IN, Pin.PULL_DOWN)
        self.grabber = Grabber(15, 13)
        self.grabber.reset()
        self.ultra_adc = ADC(Pin(self.ULTRASONIC_ADC_PIN))
        self.grab_count=0

    def start_pressed(self):
        return self.start_btn.value() == 1

    def close_gripper(self, lift=True):
        if self.grabber is not None:
            self.grabber.grab(False)
            sleep_ms(700)
            self.grabber.lift(10)

    def open_gripper(self, lift=True):
        if self.grabber is not None:
            self.grabber.opn(False)
            sleep_ms(700)
            self.grabber.lift(0)
            self.grabber.reset()


    def next_grab_colour(self):
        colours = ("red", "yellow", "green", "blue")

        if self.grab_count < len(colours):
            colour = colours[self.grab_count%4]

        self.grab_count += 1
        return colour

    def reset_gripper(self):
        if self.grabber is not None:
            self.grabber.reset()

    def lift_gripper(self, deg):
        if self.grabber is not None:
            self.grabber.lift(deg)

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
                    if d is not None and self.MIN_VALID_MM <= d <= self.MAX_VALID_MM:
                        readings.append(d)
                        print("Distance = {} mm".format(d))
                except Exception as e:
                    print("VL53L0X read error:", e)

                sleep_ms(sample_period_ms)
        finally:
            sensor.stop()
            if motors is not None:
                default_stop(motors)

        return readings

    def _median(self, values):
        if not values:
            return None

        vals = sorted(values)
        n = len(vals)
        mid = n // 2

        if n % 2 == 1:
            return vals[mid]
        else:
            return (vals[mid - 1] + vals[mid]) / 2

    def scan_right(self, node, motors=None):

        if not (15 <= node <= 20):
            return "scan_empty"
        else:
            sensor = self._make_right_sensor()
            readings = self._scan_sensor_for_duration(sensor, motors=motors)

            if not readings:
                print("RIGHT SCAN: no valid readings")
                return "scan_empty"

            med = self._median(readings)
            print("RIGHT SCAN median =", med)

            if med >= self.RIGHT_FOUND_THRESHOLD_MM and med <300:
                return "scan_found"
            return "scan_empty"

    def scan_left(self, node, motors=None):

        if not (23 <= node <= 28):
            return "scan_empty"
        else:
            threshold = self.LEFT_FOUND_THRESHOLD_NODES_23_TO_28_MM

        sensor = self._make_left_sensor()
        readings = self._scan_sensor_for_duration(sensor, motors=motors)

        if not readings:
            print("LEFT SCAN: no valid readings")
            return "scan_empty"

        med = self._median(readings)

        print("LEFT SCAN median =", med)
        print("LEFT SCAN threshold =", threshold)

        if med >= threshold and med <280:
            return "scan_found"
        return "scan_empty"

    def read_ultrasonic_cm(self):
        raw_val = self.ultra_adc.read_u16()
        voltage = raw_val * (self.ULTRASONIC_VCC / 65535.0)
        distance_cm = (voltage / self.ULTRASONIC_VCC) * self.ULTRASONIC_MAX_RANGE_CM
        return distance_cm

    def read_ultrasonic_filtered_cm(self, samples=5, sample_delay_ms=20):
        vals = []

        for _ in range(samples):
            d = self.read_ultrasonic_cm()

            if self.ULTRA_MIN_VALID_CM <= d <= self.ULTRA_MAX_VALID_CM:
                vals.append(d)

            sleep_ms(sample_delay_ms)

        if not vals:
            print("ULTRASONIC: no valid readings")
            return None

        result = self._median(vals)
        print("ULTRASONIC median = {:.1f} cm from {}".format(result, vals))
        return result

    def wait_until_ultrasonic_below(
        self,
        threshold_cm,
        motors=None,
        samples=5,
        sample_delay_ms=20,
        consecutive_hits=3,
        poll_delay_ms=30,
        timeout_ms=None
    ):
        print("WAIT UNTIL ULTRASONIC <= {} cm".format(threshold_cm))

        t0 = ticks_ms()
        hits = 0

        while True:
            if motors is not None:
                default_stop(motors)

            d = self.read_ultrasonic_filtered_cm(
                samples=samples,
                sample_delay_ms=sample_delay_ms
            )

            if d is not None and d <= threshold_cm:
                hits += 1
                print("ULTRASONIC BELOW HIT {}/{} : {:.1f} cm".format(hits, consecutive_hits, d))

                if hits >= consecutive_hits:
                    if motors is not None:
                        default_stop(motors)
                    print("ULTRASONIC THRESHOLD REACHED")
                    return True
            else:
                hits = 0

            if timeout_ms is not None:
                if ticks_diff(ticks_ms(), t0) >= timeout_ms:
                    if motors is not None:
                        default_stop(motors)
                    print("ULTRASONIC WAIT TIMEOUT")
                    return False

            sleep_ms(poll_delay_ms)


def scan(node, task_sensors, command, motors=None):
    print("SCAN START:", command, "node =", node)

    if motors is not None:
        default_stop(motors)

    if command == "scan_left":
        result = task_sensors.scan_left(node=node, motors=motors)
    elif command == "scan_right":
        result = task_sensors.scan_right(node=node, motors=motors)
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