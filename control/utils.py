from utime import ticks_ms, ticks_diff, sleep_ms

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def b4(v01):
    # list of 0/1 -> "0101"
    return "{}{}{}{}".format(v01[0], v01[1], v01[2], v01[3])

def fixed_rate_tick(t_last, period_ms):
    t_now = ticks_ms()
    dt = ticks_diff(t_now, t_last)
    if dt < period_ms:
        sleep_ms(period_ms - dt)
        t_now = ticks_ms()
    return t_now
