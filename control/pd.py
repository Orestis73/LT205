import config
from control.utils import clamp


def middle_error_white_line(black):
    # Use the two middle sensors to estimate line position error
    mL = black[1]
    mR = black[2]

    # If both middle sensors see white, treat it as centred
    if mL == 0 and mR == 0:
        return 0.0

    # Left middle sensor sees black -> line is slightly to the left
    if mL == 1 and mR == 0:
        return +1.0

    # Right middle sensor sees black -> line is slightly to the right
    if mL == 0 and mR == 1:
        return -1.0

    # Any other case is unclear / ambiguous
    return None


def pd_follow(err, last_err, dt_s):
    # Derivative term based on change in error
    derr = (err - last_err) / dt_s

    # PD steering control
    steer = config.KP * err + config.KD * derr

    # Limit steering so it stays within safe bounds
    steer = clamp(steer, -config.MAX_STEER, +config.MAX_STEER)

    # Reduce throttle when steering more sharply
    throttle = config.BASE_THROTTLE - config.SLOW_K * abs(steer)

    # Keep throttle within allowed range
    throttle = clamp(throttle, config.MIN_THROTTLE, config.BASE_THROTTLE)

    return throttle, steer