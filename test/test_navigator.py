from simplified_navigator import (
    Navigator,
    CMD_FOLLOW,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_STRAIGHT,
    CMD_180,
    CMD_SCAN_LEFT,
    CMD_SCAN_RIGHT,
    CMD_GRAB_LEFT,
    CMD_GRAB_RIGHT,
    CMD_DROP,
    CMD_FINISHED,
    RES_EVENT,
    RES_DONE,
    RES_SCAN_EMPTY,
    RES_SCAN_FOUND,
    RES_DROP_DONE,
)

# ------------------------------------------------------------
# Fake world setup
# ------------------------------------------------------------
# Put reels in these slots:
#   pd slot 2 -> blue
#   pd slot 5 -> green
#   ou slot 3 -> yellow
#   od slot 4 -> red
#
# Keys match navigator slot keys: (stack, slot)
#WORLD_REELS = { }

WORLD_REELS = {


    ("pd", 2): "blue",
    ("pd", 3): "green",
    ("pd", 4): "yellow",
    ("pd", 5): "red",
}


def fake_result_for_command(nav, cmd):
    """
    Given the current navigator state and command,
    return the fake result that the mission runner would send back.
    """
    if cmd == CMD_FOLLOW:
        return RES_EVENT

    if cmd in (CMD_LEFT, CMD_RIGHT, CMD_STRAIGHT, CMD_180):
        return RES_DONE

    if cmd in (CMD_SCAN_LEFT, CMD_SCAN_RIGHT):
        info = nav.scan_points[nav.current_node]
        key = (info["stack"], info["slot"])
        if key in WORLD_REELS:
            return RES_SCAN_FOUND
        return RES_SCAN_EMPTY

    if cmd in (CMD_GRAB_LEFT, CMD_GRAB_RIGHT):
        key = (nav.active_stack, nav.active_slot)
        if key not in WORLD_REELS:
            return {"status": "grab_fail"}

        colour = WORLD_REELS[key]
        # remove it from the fake world once grabbed
        del WORLD_REELS[key]
        return {"status": "grab_ok", "colour": colour}

    if cmd == CMD_DROP:
        return RES_DROP_DONE

    if cmd == CMD_FINISHED:
        return None

    raise ValueError("Unknown command: {}".format(cmd))


def main():
    nav = Navigator(expected_total_reels=4)

    step = 0
    last_result = None

    print("=" * 80)
    print("STARTING NAVIGATOR TEST")
    print("=" * 80)

    while True:
        cmd = nav.next_command(last_result)

        dbg = nav.debug_status()
        print(
            "[{:03d}] CMD={:<12} mode={:<12} node={:<4} heading={:<2} "
            "route_idx={:<3} active=({},{}) colour={}".format(
                step,
                cmd,
                dbg["mode"],
                str(dbg["node"]),
                dbg["heading"],
                dbg["scan_route_index"],
                dbg["active_stack"],
                dbg["active_slot"],
                dbg["carrying_colour"],
            )
        )

        if cmd == CMD_FINISHED:
            print("[{:03d}] MISSION FINISHED".format(step))
            break

        last_result = fake_result_for_command(nav, cmd)
        print("      -> RESULT =", last_result)

        step += 1

        if step > 2000:
            print("Too many steps, aborting.")
            break

    print("=" * 80)
    print("FINAL STATUS")
    print("=" * 80)
    print(nav.debug_status())


if __name__ == "__main__":
    main()