from utime import ticks_ms, ticks_diff, sleep_ms

class Mem:
    """All mutable runtime variables in one place."""

    def __init__(self):

        self.last_event_t = 0
        self.last_err = 0.0
        self.good_rearm = 0
        self.event_in_count = 0
        self.t_last_good_line = ticks_ms()

