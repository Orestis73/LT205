HEAD_N = 0
HEAD_E = 1
HEAD_S = 2
HEAD_W = 3


CMD_FOLLOW = "follow"
CMD_LEFT = "left"
CMD_RIGHT = "right"
CMD_STRAIGHT = "straight"
CMD_180 = "180"
CMD_SCAN_LEFT = "scan_left"
CMD_SCAN_RIGHT = "scan_right"
CMD_GRAB_LEFT = "grab_left"
CMD_GRAB_RIGHT = "grab_right"
CMD_DROP = "drop"
CMD_FINISHED = "finished"


RES_EVENT = "event"
RES_DONE = "done"
RES_SCAN_EMPTY = "scan_empty"
RES_SCAN_FOUND = "scan_found"
RES_GRAB_OK = "grab_ok"
RES_GRAB_FAIL = "grab_fail"
RES_DROP_DONE = "drop_done"


SLOT_UNKNOWN = "unknown"
SLOT_EMPTY = "empty"
SLOT_FOUND = "found"
SLOT_DELIVERED = "delivered"


MODE_SCAN = "scan"
MODE_DELIVERY = "delivery"
MODE_RESUME = "resume"
MODE_RETURN_HOME = "return_home"
MODE_FINISHED = "finished"


RESULT_ALIASES = {
    "event_reached": RES_EVENT,
    "event": RES_EVENT,
    "intersection": RES_EVENT,
    "done": RES_DONE,
    "ok": RES_DONE,
    "scan_empty": RES_SCAN_EMPTY,
    "empty": RES_SCAN_EMPTY,
    "none": RES_SCAN_EMPTY,
    "scan_found": RES_SCAN_FOUND,
    "found": RES_SCAN_FOUND,
    "reel_found": RES_SCAN_FOUND,
    "grab_ok": RES_GRAB_OK,
    "grab_success": RES_GRAB_OK,
    "grab_fail": RES_GRAB_FAIL,
    "grab_failed": RES_GRAB_FAIL,
    "drop_done": RES_DROP_DONE,
    "dropped": RES_DROP_DONE,
}


COLOUR_NAMES = ("blue", "green", "yellow", "red")


def heading_name(h):
    return {
        HEAD_N: "N",
        HEAD_E: "E",
        HEAD_S: "S",
        HEAD_W: "W",
    }[h]



def rotate_90(heading, turn_dir):
    """
    turn_dir:
        -1 -> left
        +1 -> right
    """
    return (heading + turn_dir) % 4



def _turn_action(h_in, h_out):
    delta = (h_out - h_in) % 4
    if delta == 0:
        return CMD_STRAIGHT
    if delta == 1:
        return CMD_RIGHT
    if delta == 2:
        return CMD_180
    return CMD_LEFT


class Navigator:


    def __init__(self, expected_total_reels=4):
        self.expected_total_reels = expected_total_reels

        #Graph structure: node connected with the list of adjacent nodes. Nodes can be integers or strings (for delivery targets).
        self.graph = {
            1: [2, 39],
            2: [1, 3, "C"],
            3: [2, 4, "D"],
            4: [3, 5],
            5: [4, 6],
            6: [5, 7],
            7: [6, 8],
            8: [7, 9],
            9: [8, 10],
            10: [9, 11],
            11: [10, 12],
            12: [11, 13, 30],
            13: [12, 14, 22],
            14: [13, 15],
            15: [14, 16],
            16: [15, 17],
            17: [16, 18],
            18: [17, 19],
            19: [18, 20],
            20: [19, 21],
            21: [20],
            22: [13, 23],
            23: [22, 24],
            24: [23, 25],
            25: [24, 26],
            26: [25, 27],
            27: [26, 28],
            28: [27, 29],
            29: [28],
            30: [12, 31],
            31: [30, 32],
            32: [31, 33],
            33: [32, 34],
            34: [33, 35],
            35: [34, 36],
            36: [35, 37],
            37: [36, 38],
            38: [37, 39, "A"],
            39: [38, 1, "B"],
            "A": [38],
            "B": [39],
            "C": [2],
            "D": [3],
        }

        #Node coordinates used to determine heading direction
        self.xy = {
            1: (0, 0),
            2: (3, 0),
            3: (4, 0),
            4: (4, 1),
            5: (4, 2),
            6: (4, 3),
            7: (4, 4),
            8: (4, 5),
            9: (4, 6),
            10: (4, 7),
            11: (4, 8),
            12: (0, 8),
            13: (0, 2),
            14: (1, 2),
            15: (1, 3),
            16: (1, 4),
            17: (1, 5),
            18: (1, 6),
            19: (1, 7),
            20: (1, 8),
            21: (1, 9),
            22: (-1, 2),
            23: (-1, 3),
            24: (-1, 4),
            25: (-1, 5),
            26: (-1, 6),
            27: (-1, 7),
            28: (-1, 8),
            29: (-1, 9),
            30: (-4, 8),
            31: (-4, 7),
            32: (-4, 6),
            33: (-4, 5),
            34: (-4, 4),
            35: (-4, 3),
            36: (-4, 2),
            37: (-4, 1),
            38: (-4, 0),
            39: (-3, 0),
            "A": (-4, -1),
            "B": (-3, -1),
            "C": (3, -1),
            "D": (4, -1),
        }

        # The main scan route, which covers all the scan points in an efficient order.
        self.scan_route = [
            1,
            2, 3,
            4, 5, 6, 7, 8, 9, 10, 11,
            12, 13,
            14, 15, 16, 17, 18, 19, 20,
            19, 18, 17, 16, 15, 14,
            13,
            22, 23, 24, 25, 26, 27, 28,
            27, 26, 25, 24, 23, 22,
            13, 12,
            30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
            1,
        ]

        #
        self.scan_points = {
            4: {"stack": "pd", "slot": 1, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            5: {"stack": "pd", "slot": 2, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            6: {"stack": "pd", "slot": 3, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            7: {"stack": "pd", "slot": 4, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            8: {"stack": "pd", "slot": 5, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            9: {"stack": "pd", "slot": 6, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            15: {"stack": "pu", "slot": 1, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            16: {"stack": "pu", "slot": 2, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            17: {"stack": "pu", "slot": 3, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            18: {"stack": "pu", "slot": 4, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            19: {"stack": "pu", "slot": 5, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            20: {"stack": "pu", "slot": 6, "turn": +1, "side": "right", "scan_heading": HEAD_N},
            23: {"stack": "ou", "slot": 1, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            24: {"stack": "ou", "slot": 2, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            25: {"stack": "ou", "slot": 3, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            26: {"stack": "ou", "slot": 4, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            27: {"stack": "ou", "slot": 5, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            28: {"stack": "ou", "slot": 6, "turn": -1, "side": "left", "scan_heading": HEAD_N},
            32: {"stack": "od", "slot": 1, "turn": -1, "side": "left", "scan_heading": HEAD_S},
            33: {"stack": "od", "slot": 2, "turn": -1, "side": "left", "scan_heading": HEAD_S},
            34: {"stack": "od", "slot": 3, "turn": -1, "side": "left", "scan_heading": HEAD_S},
            35: {"stack": "od", "slot": 4, "turn": -1, "side": "left", "scan_heading": HEAD_S},
            36: {"stack": "od", "slot": 5, "turn": -1, "side": "left", "scan_heading": HEAD_S},
            37: {"stack": "od", "slot": 6, "turn": -1, "side": "left", "scan_heading": HEAD_S},
        }

        self.delivery_target = {
            "blue": "A",
            "green": "B",
            "yellow": "C",
            "red": "D",
        }

        self.scan_sequence = []
        self.scan_route_index_by_slot = {}
        self._build_scan_sequence()

        self.slot_state = {}
        for key in self.scan_sequence:
            self.slot_state[key] = SLOT_UNKNOWN

        self.reset()

    # --------------------------------------------------------------
    # Public API
    # --------------------------------------------------------------
    def reset(self):
        self.mode = MODE_SCAN
        self.current_node = self.scan_route[0]
        self.current_heading = HEAD_N
        self.at_node_event = False  # before node 1 after border-push
        self.scan_route_index = 0

        self.path_nodes = None
        self.path_index = None
        self.resume_target_slot = None
        self.resume_target_route_index = None

        self.active_stack = None
        self.active_slot = None
        self.active_turn = None
        self.carrying_colour = None

        self.awaiting = None
        self.finished_emitted = False

    def next_command(self, last_result=None):
        """
        Main interface.

        First call: nav.next_command()
        Later calls: nav.next_command(last_result)
        """
        if self.awaiting is not None:
            if last_result is None:
                raise ValueError("Navigator expected a result for command '{}'".format(self.awaiting["cmd"]))
            norm = self._normalize_result(last_result)
            self._apply_result(self.awaiting, norm)
            self.awaiting = None
        elif last_result is not None:
            # Ignore extra initial results silently.
            pass

        cmd = self._decide_next_command()
        self.awaiting = {"cmd": cmd}
        self._attach_pending_metadata(self.awaiting)
        return cmd

    def debug_status(self):
        return {
            "mode": self.mode,
            "node": self.current_node,
            "heading": heading_name(self.current_heading),
            "at_node_event": self.at_node_event,
            "scan_route_index": self.scan_route_index,
            "path_nodes": self.path_nodes,
            "path_index": self.path_index,
            "active_stack": self.active_stack,
            "active_slot": self.active_slot,
            "active_turn": self.active_turn,
            "carrying_colour": self.carrying_colour,
            "delivered_count": self.delivered_count(),
            "next_unresolved_slot": self.next_unresolved_slot(),
        }

    # --------------------------------------------------------------
    # Bookkeeping
    # --------------------------------------------------------------
    def delivered_count(self):
        total = 0
        for key in self.scan_sequence:
            if self.slot_state[key] == SLOT_DELIVERED:
                total += 1
        return total

    def all_expected_delivered(self):
        return self.delivered_count() >= self.expected_total_reels

    def next_unresolved_slot(self):
        for key in self.scan_sequence:
            if self.slot_state[key] == SLOT_UNKNOWN:
                return key
        return None

    # --------------------------------------------------------------
    # Decision logic
    # --------------------------------------------------------------
    def _decide_next_command(self):
        if self.mode == MODE_FINISHED:
            self.finished_emitted = True
            return CMD_FINISHED

        if not self.at_node_event:
            return CMD_FOLLOW

        if self.mode == MODE_DELIVERY and self._at_delivery_target():
            return CMD_DROP

        scan_cmd = self._scan_command_if_needed()
        if scan_cmd is not None:
            return scan_cmd

        grab_cmd = self._grab_command_if_needed()
        if grab_cmd is not None:
            return grab_cmd

        if self.all_expected_delivered():
            if self.current_node == 1:
                self.mode = MODE_FINISHED
                self.finished_emitted = True
                return CMD_FINISHED
            self._ensure_return_home_path()

        if self.mode == MODE_SCAN:
            return self._scan_route_move_command()
        if self.mode in (MODE_DELIVERY, MODE_RESUME, MODE_RETURN_HOME):
            return self._path_move_command()

        raise ValueError("Unknown mode '{}'".format(self.mode))

    def _scan_command_if_needed(self):
        if self.mode != MODE_SCAN:
            return None
        if self.active_slot is not None:
            return None
        info = self.scan_points.get(self.current_node)
        if info is None:
            return None
        if self.current_heading != info["scan_heading"]:
            return None
        key = (info["stack"], info["slot"])
        if self.slot_state[key] != SLOT_UNKNOWN:
            return None
        return CMD_SCAN_LEFT if info["side"] == "left" else CMD_SCAN_RIGHT

    def _grab_command_if_needed(self):
        if self.active_slot is None:
            return None
        if self.carrying_colour is not None:
            return None
        if self.mode != MODE_SCAN:
            return None
        return CMD_GRAB_LEFT if self.active_turn == -1 else CMD_GRAB_RIGHT

    def _scan_route_move_command(self):
        if self.scan_route_index >= len(self.scan_route) - 1:
            if self.current_node == 1:
                self.mode = MODE_FINISHED
                return CMD_FINISHED
            self._ensure_return_home_path()
            return self._path_move_command()

        next_node = self.scan_route[self.scan_route_index + 1]
        next_heading = self._heading_between(self.current_node, next_node)
        return _turn_action(self.current_heading, next_heading)

    def _path_move_command(self):
        if self.path_nodes is None or self.path_index is None:
            raise ValueError("No active path")

        if self.path_index >= len(self.path_nodes) - 1:
            if self.mode == MODE_RESUME:
                self.mode = MODE_SCAN
                self.scan_route_index = self.resume_target_route_index
                self.path_nodes = None
                self.path_index = None
                self.resume_target_slot = None
                self.resume_target_route_index = None
                return self._decide_next_command()
            if self.mode == MODE_RETURN_HOME:
                if self.current_node != 1:
                    raise ValueError("Return-home path ended away from node 1")
                self.mode = MODE_FINISHED
                return CMD_FINISHED
            if self.mode == MODE_DELIVERY:
                if not self._at_delivery_target():
                    raise ValueError("Delivery path ended at wrong node")
                return CMD_DROP
            raise ValueError("Path ended in unsupported mode '{}'".format(self.mode))

        next_node = self.path_nodes[self.path_index + 1]
        next_heading = self._heading_between(self.current_node, next_node)
        return _turn_action(self.current_heading, next_heading)

    # --------------------------------------------------------------
    # Result handling
    # --------------------------------------------------------------
    def _normalize_result(self, result):
        if isinstance(result, dict):
            norm = {}
            for k, v in result.items():
                norm[str(k).lower()] = v
            status = norm.get("status")
            if isinstance(status, str):
                status_key = status.strip().lower()
                norm["status"] = RESULT_ALIASES.get(status_key, status_key)
            return norm

        if isinstance(result, str):
            key = result.strip().lower()
            if ":" in key:
                left, right = key.split(":", 1)
                left = RESULT_ALIASES.get(left, left)
                return {"status": left, "colour": right}
            return {"status": RESULT_ALIASES.get(key, key)}

        raise ValueError("Unsupported result type: {}".format(type(result)))

    def _apply_result(self, pending, result):
        cmd = pending["cmd"]
        status = result.get("status")

        if cmd == CMD_FOLLOW:
            if status != RES_EVENT:
                raise ValueError("follow expected result '{}' but got '{}'".format(RES_EVENT, status))
            self.at_node_event = True
            return

        if cmd in (CMD_LEFT, CMD_RIGHT, CMD_STRAIGHT, CMD_180):
            if status != RES_DONE:
                raise ValueError("move command expected '{}' but got '{}'".format(RES_DONE, status))
            self.current_node = pending["next_node"]
            self.current_heading = pending["next_heading"]
            self.at_node_event = False

            if pending.get("kind") == "scan_route":
                self.scan_route_index = pending["next_index"]
            else:
                self.path_index = pending["next_index"]
            return

        if cmd in (CMD_SCAN_LEFT, CMD_SCAN_RIGHT):
            key = pending["slot_key"]
            if status == RES_SCAN_EMPTY:
                self.slot_state[key] = SLOT_EMPTY
                return
            if status == RES_SCAN_FOUND:
                self.slot_state[key] = SLOT_FOUND
                self.active_stack = pending["stack"]
                self.active_slot = pending["slot"]
                self.active_turn = pending["turn"]
                return
            raise ValueError("scan command got unsupported result '{}'".format(status))

        if cmd in (CMD_GRAB_LEFT, CMD_GRAB_RIGHT):
            if status == RES_GRAB_FAIL:
                # Keep the slot marked as found, but abandon carrying state.
                self.active_stack = None
                self.active_slot = None
                self.active_turn = None
                self.carrying_colour = None
                return

            if status != RES_GRAB_OK:
                raise ValueError("grab command got unsupported result '{}'".format(status))

            colour = self._extract_colour(result)
            if colour not in self.delivery_target:
                raise ValueError("grab result did not contain valid colour")

            self.carrying_colour = colour
            self.current_heading = rotate_90(self.current_heading, pending["turn_dir"])
            self.at_node_event = True
            self._build_delivery_path(colour)
            return

        if cmd == CMD_DROP:
            if status not in (RES_DROP_DONE, RES_DONE):
                raise ValueError("drop expected '{}' or '{}' but got '{}'".format(RES_DROP_DONE, RES_DONE, status))

            if self.active_stack is None or self.active_slot is None or self.carrying_colour is None:
                raise ValueError("drop completed with no active reel")

            self.slot_state[(self.active_stack, self.active_slot)] = SLOT_DELIVERED
            self.active_stack = None
            self.active_slot = None
            self.active_turn = None
            self.carrying_colour = None
            self.path_nodes = None
            self.path_index = None

            if self.all_expected_delivered():
                if self.current_node == 1:
                    self.mode = MODE_FINISHED
                else:
                    self._ensure_return_home_path()
                return

            self._build_resume_path()
            return

        if cmd == CMD_FINISHED:
            self.mode = MODE_FINISHED
            return

        raise ValueError("Unsupported pending command '{}'".format(cmd))

    def _extract_colour(self, result):
        colour = result.get("colour")
        if isinstance(colour, str):
            colour = colour.strip().lower()
            if colour in COLOUR_NAMES:
                return colour

        status = result.get("status")
        if isinstance(status, str) and status in COLOUR_NAMES:
            return status

        return None

    def _attach_pending_metadata(self, pending):
        cmd = pending["cmd"]

        if cmd in (CMD_LEFT, CMD_RIGHT, CMD_STRAIGHT, CMD_180):
            if self.mode == MODE_SCAN:
                next_index = self.scan_route_index + 1
                if next_index >= len(self.scan_route):
                    raise ValueError("scan route index overflow")
                next_node = self.scan_route[next_index]
                next_heading = self._heading_between(self.current_node, next_node)
                pending["kind"] = "scan_route"
                pending["next_index"] = next_index
                pending["next_node"] = next_node
                pending["next_heading"] = next_heading
                return

            if self.mode in (MODE_DELIVERY, MODE_RESUME, MODE_RETURN_HOME):
                if self.path_nodes is None or self.path_index is None:
                    raise ValueError("No active path for move command")
                next_index = self.path_index + 1
                if next_index >= len(self.path_nodes):
                    raise ValueError("path index overflow")
                next_node = self.path_nodes[next_index]
                next_heading = self._heading_between(self.current_node, next_node)
                pending["kind"] = "path"
                pending["next_index"] = next_index
                pending["next_node"] = next_node
                pending["next_heading"] = next_heading
                return

        if cmd in (CMD_SCAN_LEFT, CMD_SCAN_RIGHT):
            info = self.scan_points[self.current_node]
            pending["stack"] = info["stack"]
            pending["slot"] = info["slot"]
            pending["turn"] = info["turn"]
            pending["slot_key"] = (info["stack"], info["slot"])
            return

        if cmd in (CMD_GRAB_LEFT, CMD_GRAB_RIGHT):
            pending["turn_dir"] = self.active_turn
            return

    # --------------------------------------------------------------
    # Path building
    # --------------------------------------------------------------
    def _build_delivery_path(self, colour):
        target = self.delivery_target[colour]
        path = self._shortest_path(self.current_node, target)
        self.mode = MODE_DELIVERY
        self.path_nodes = path
        self.path_index = 0

    def _build_resume_path(self):
        slot_key = self.next_unresolved_slot()
        if slot_key is None:
            self._ensure_return_home_path()
            return

        route_index = self.scan_route_index_by_slot[slot_key]
        target_node = self.scan_route[route_index]
        target_heading = self.scan_points[target_node]["scan_heading"]

        path = self._pose_shortest_path(
            start_node=self.current_node,
            start_heading=self.current_heading,
            goal_node=target_node,
            goal_heading=target_heading,
        )

        self.mode = MODE_RESUME
        self.path_nodes = path
        self.path_index = 0
        self.resume_target_slot = slot_key
        self.resume_target_route_index = route_index

    def _ensure_return_home_path(self):
        path = self._shortest_path(self.current_node, 1)
        self.mode = MODE_RETURN_HOME
        self.path_nodes = path
        self.path_index = 0

    # --------------------------------------------------------------
    # Geometry helpers
    # --------------------------------------------------------------
    def _build_scan_sequence(self):
        current_heading = HEAD_N
        for i in range(len(self.scan_route) - 1):
            node = self.scan_route[i]
            nxt = self.scan_route[i + 1]
            if node in self.scan_points:
                info = self.scan_points[node]
                if current_heading == info["scan_heading"]:
                    key = (info["stack"], info["slot"])
                    self.scan_sequence.append(key)
                    self.scan_route_index_by_slot[key] = i
            current_heading = self._heading_between(node, nxt)

    def _heading_between(self, a, b):
        xa, ya = self.xy[a]
        xb, yb = self.xy[b]

        if xb == xa and yb > ya:
            return HEAD_N
        if xb > xa and yb == ya:
            return HEAD_E
        if xb == xa and yb < ya:
            return HEAD_S
        if xb < xa and yb == ya:
            return HEAD_W
        raise ValueError("Non-Manhattan edge: {} -> {}".format(a, b))

    def _shortest_path(self, start, goal):
        if start == goal:
            return [start]

        q = [start]
        q_head = 0
        prev = {start: None}

        while q_head < len(q):
            node = q[q_head]
            q_head += 1

            if node == goal:
                break

            for nxt in self.graph[node]:
                if nxt not in prev:
                    prev[nxt] = node
                    q.append(nxt)

        if goal not in prev:
            raise ValueError("No path from {} to {}".format(start, goal))

        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = prev[node]
        path.reverse()
        return path

    def _pose_shortest_path(self, start_node, start_heading, goal_node, goal_heading):
        start_state = (start_node, start_heading)
        goal_state = (goal_node, goal_heading)

        if start_state == goal_state:
            return [start_node]

        q = [start_state]
        q_head = 0
        prev = {start_state: None}

        while q_head < len(q):
            node, heading = q[q_head]
            q_head += 1

            for nxt in self.graph[node]:
                new_heading = self._heading_between(node, nxt)
                nxt_state = (nxt, new_heading)
                if nxt_state not in prev:
                    prev[nxt_state] = (node, heading)
                    if nxt_state == goal_state:
                        q_head = len(q)
                        break
                    q.append(nxt_state)

        if goal_state not in prev:
            raise ValueError(
                "No pose-path from ({}, {}) to ({}, {})".format(
                    start_node,
                    heading_name(start_heading),
                    goal_node,
                    heading_name(goal_heading),
                )
            )

        states = []
        state = goal_state
        while state is not None:
            states.append(state)
            state = prev[state]
        states.reverse()
        return [node for node, _heading in states]

    def _at_delivery_target(self):
        if self.carrying_colour is None:
            return False
        return self.current_node == self.delivery_target[self.carrying_colour]


# Backward-compatible alias.
navigation = Navigator


if __name__ == "__main__":
    nav = Navigator()
    print("Navigator ready. First command:", nav.next_command())
    print(nav.debug_status())
