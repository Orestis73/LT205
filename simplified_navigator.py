# ------------------------------
# Constant Definitions (Heading/Command/Result/Slot/Mode)
# ------------------------------
# Heading direction constants (North/East/South/West)
HEAD_N = 0
HEAD_E = 1
HEAD_S = 2
HEAD_W = 3

# Navigation command constants (core actions the robot can execute)
CMD_FOLLOW = "follow"       # Follow the line until an event (intersection/corner) is detected
CMD_LEFT = "left"           # Turn left at current node
CMD_RIGHT = "right"         # Turn right at current node
CMD_STRAIGHT = "straight"   # Move straight forward to next node
CMD_180 = "180"             # Rotate 180 degrees in place
CMD_SCAN_LEFT = "scan_left" # Scan the left side for a reel
CMD_SCAN_RIGHT = "scan_right" # Scan the right side for a reel
CMD_GRAB_LEFT = "grab_left" # Grab a reel from the left side
CMD_GRAB_RIGHT = "grab_right" # Grab a reel from the right side
CMD_DROP = "drop"           # Drop a carried reel at delivery target
CMD_FINISHED = "finished"   # Mission complete, stop all actions

# Result constants (feedback from executing commands)
RES_EVENT = "event"             # Follow command detected an intersection/corner
RES_DONE = "done"               # Move/action command completed successfully
RES_SCAN_EMPTY = "scan_empty"   # Scan found no reel
RES_SCAN_FOUND = "scan_found"   # Scan detected a reel
RES_GRAB_OK = "grab_ok"         # Grab command succeeded (reel picked up)
RES_GRAB_FAIL = "grab_fail"     # Grab command failed (no reel picked up)
RES_DROP_DONE = "drop_done"     # Drop command completed successfully

# Slot state constants (track status of reel slots)
SLOT_UNKNOWN = "unknown"    # Slot not yet scanned
SLOT_EMPTY = "empty"        # Slot scanned, no reel present
SLOT_FOUND = "found"        # Slot scanned, reel detected (not yet grabbed)
SLOT_DELIVERED = "delivered" # Reel from slot has been delivered

# Operational mode constants (navigator state machine)
MODE_SCAN = "scan"              # Scanning phase (search for reels)
MODE_DELIVERY = "delivery"      # Delivery phase (transport reel to target)
MODE_RESUME = "resume"          # Resume phase (return to scan route after delivery)
MODE_RETURN_HOME = "return_home" # Return phase (go back to start after all deliveries)
MODE_FINISHED = "finished"      # Mission complete

# Result alias mapping (normalize diverse input results to standard constants)
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

# Valid reel colour names (mapped to delivery targets)
COLOUR_NAMES = ("blue", "green", "yellow", "red")


# ------------------------------
# Helper Functions (Heading/Rotation)
# ------------------------------
def heading_name(h):
    """Convert numeric heading constant to human-readable string (N/E/S/W)"""
    return {
        HEAD_N: "N",
        HEAD_E: "E",
        HEAD_S: "S",
        HEAD_W: "W",
    }[h]


def rotate_90(heading, turn_dir):
    """
    Rotate a heading 90 degrees left/right and return new heading.
    :param heading: Current heading (HEAD_N/HEAD_E/HEAD_S/HEAD_W)
    :param turn_dir: Rotation direction (-1 = left, +1 = right)
    :return: New heading after rotation (mod 4 to wrap around)
    """
    return (heading + turn_dir) % 4


def _turn_action(h_in, h_out):
    """
    Determine the required turn command to get from input to output heading.
    :param h_in: Current heading
    :param h_out: Desired heading
    :return: Command (CMD_STRAIGHT/CMD_RIGHT/CMD_180/CMD_LEFT)
    """
    delta = (h_out - h_in) % 4
    if delta == 0:
        return CMD_STRAIGHT
    if delta == 1:
        return CMD_RIGHT
    if delta == 2:
        return CMD_180
    return CMD_LEFT


# ------------------------------
# Core Navigator Class (Mission Logic)
# ------------------------------
class Navigator:
    def __init__(self, expected_total_reels=4):
        """
        Initialize the Navigator with mission parameters and map data.
        :param expected_total_reels: Total number of reels to deliver (default: 4)
        """
        self.expected_total_reels = expected_total_reels

        # Graph structure: Node -> list of adjacent nodes (map of the robot's environment)
        # Nodes are integers (path nodes) or strings (delivery targets: A/B/C/D)
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

        # Node coordinates (x,y) for calculating heading between nodes (Manhattan grid)
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

        # Predefined scan route: ordered list of nodes to visit for reel scanning
        # Covers all scan points in an efficient path (no backtracking where avoidable)
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

        # Scan point metadata: defines reel slot properties at specific nodes
        # stack: slot group ID, slot: slot number, turn: rotation dir for grab, 
        # side: scan/grab side (left/right), scan_heading: required heading for scanning
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

        # Delivery target mapping: reel colour -> target node (A/B/C/D)
        self.delivery_target = {
            "blue": "A",
            "green": "B",
            "yellow": "C",
            "red": "D",
        }

        # Initialize scan sequence (ordered list of slot keys to scan)
        self.scan_sequence = []
        # Map slot keys to their index in the scan route (for resume pathing)
        self.scan_route_index_by_slot = {}
        self._build_scan_sequence()

        # Initialize slot state tracking (all slots start as UNKNOWN)
        self.slot_state = {}
        for key in self.scan_sequence:
            self.slot_state[key] = SLOT_UNKNOWN

        # Reset navigator state to initial conditions
        self.reset()

    # --------------------------------------------------------------
    # Public API (External Interface)
    # --------------------------------------------------------------
    def reset(self):
        """Reset all navigator state variables to initial mission conditions"""
        self.mode = MODE_SCAN                  # Start in scan mode
        self.current_node = self.scan_route[0] # Start at first node of scan route
        self.current_heading = HEAD_N          # Initial heading: North
        self.at_node_event = False             # Not at an event (intersection/corner) initially
        self.scan_route_index = 0              # Start at first position in scan route

        # Path-related state (for delivery/resume/return-home)
        self.path_nodes = None                 # List of nodes in active path
        self.path_index = None                 # Current position in active path
        self.resume_target_slot = None         # Target slot for resume mode
        self.resume_target_route_index = None  # Target scan route index for resume mode

        # Active reel state (tracks reel being grabbed/delivered)
        self.active_stack = None               # Stack ID of active reel
        self.active_slot = None                # Slot number of active reel
        self.active_turn = None                # Rotation dir for grabbing active reel
        self.carrying_colour = None            # Colour of reel being carried (None = not carrying)

        # Command awaiting result (for syncing with mission runner)
        self.awaiting = None
        self.finished_emitted = False          # Flag to avoid duplicate FINISHED commands

    def next_command(self, last_result=None):
        """
        Main public interface: get the next command for the robot to execute.
        1. Process the result of the previous command (if any)
        2. Decide the next command based on current state
        3. Track the pending command and attach metadata
        :param last_result: Result of the previously executed command (None for first call)
        :return: Next command string (e.g., CMD_FOLLOW, CMD_SCAN_LEFT)
        """
        # Process result of pending command (if awaiting a result)
        if self.awaiting is not None:
            if last_result is None:
                raise ValueError("Navigator expected a result for command '{}'".format(self.awaiting["cmd"]))
            norm = self._normalize_result(last_result)
            self._apply_result(self.awaiting, norm)
            self.awaiting = None
        elif last_result is not None:
            # Ignore extra initial results silently (edge case handling)
            pass

        # Decide next command based on current state
        cmd = self._decide_next_command()
        # Track the pending command and attach metadata
        self.awaiting = {"cmd": cmd}
        self._attach_pending_metadata(self.awaiting)
        return cmd

    def debug_status(self):
        """
        Return debug-friendly status snapshot of the navigator.
        Useful for testing/debugging to verify state machine behavior.
        :return: Dictionary of key state variables
        """
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
    # Bookkeeping (State Tracking Helpers)
    # --------------------------------------------------------------
    def delivered_count(self):
        """Count the number of reels successfully delivered (SLOT_DELIVERED)"""
        total = 0
        for key in self.scan_sequence:
            if self.slot_state[key] == SLOT_DELIVERED:
                total += 1
        return total

    def all_expected_delivered(self):
        """Check if all expected reels have been delivered"""
        return self.delivered_count() >= self.expected_total_reels

    def next_unresolved_slot(self):
        """Find the next slot that hasn't been scanned (SLOT_UNKNOWN)"""
        for key in self.scan_sequence:
            if self.slot_state[key] == SLOT_UNKNOWN:
                return key
        return None

    # --------------------------------------------------------------
    # Decision Logic (Core State Machine)
    # --------------------------------------------------------------
    def _decide_next_command(self):
        """
        Core decision logic: determine next command based on current mode/state.
        Follows priority order:
        1. Finish mission if in FINISHED mode
        2. Follow line if not at an event node
        3. Drop reel if at delivery target (DELIVERY mode)
        4. Scan if at a scan point and slot is unknown
        5. Grab if reel found and not carrying anything
        6. Return home if all reels delivered
        7. Follow scan route (SCAN mode) or active path (delivery/resume/return)
        """
        # Case 1: Mission already finished
        if self.mode == MODE_FINISHED:
            self.finished_emitted = True
            return CMD_FINISHED

        # Case 2: Not at an event node → follow line until event detected
        if not self.at_node_event:
            return CMD_FOLLOW

        # Case 3: At delivery target (DELIVERY mode) → drop reel
        if self.mode == MODE_DELIVERY and self._at_delivery_target():
            return CMD_DROP

        # Case 4: Need to scan a slot → return scan command
        scan_cmd = self._scan_command_if_needed()
        if scan_cmd is not None:
            return scan_cmd

        # Case 5: Need to grab a found reel → return grab command
        grab_cmd = self._grab_command_if_needed()
        if grab_cmd is not None:
            return grab_cmd

        # Case 6: All reels delivered → return home to node 1
        if self.all_expected_delivered():
            if self.current_node == 1:
                self.mode = MODE_FINISHED
                self.finished_emitted = True
                return CMD_FINISHED
            self._ensure_return_home_path()

        # Case 7: Follow scan route (SCAN mode) or active path (other modes)
        if self.mode == MODE_SCAN:
            return self._scan_route_move_command()
        if self.mode in (MODE_DELIVERY, MODE_RESUME, MODE_RETURN_HOME):
            return self._path_move_command()

        # Fallback: unknown mode (error)
        raise ValueError("Unknown mode '{}'".format(self.mode))

    def _scan_command_if_needed(self):
        """
        Check if a scan command is needed:
        - Must be in SCAN mode
        - No active reel (active_slot is None)
        - Current node is a scan point
        - Current heading matches required scan heading
        - Slot state is UNKNOWN
        :return: Scan command (CMD_SCAN_LEFT/RIGHT) or None
        """
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
        """
        Check if a grab command is needed:
        - Active reel detected (active_slot not None)
        - Not carrying a reel (carrying_colour is None)
        - In SCAN mode
        :return: Grab command (CMD_GRAB_LEFT/RIGHT) or None
        """
        if self.active_slot is None:
            return None
        if self.carrying_colour is not None:
            return None
        if self.mode != MODE_SCAN:
            return None
        return CMD_GRAB_LEFT if self.active_turn == -1 else CMD_GRAB_RIGHT

    def _scan_route_move_command(self):
        """
        Determine next move command to follow the predefined scan route.
        :return: Move command (CMD_LEFT/RIGHT/STRAIGHT/180) or CMD_FINISHED (route complete)
        """
        # Check if scan route is complete
        if self.scan_route_index >= len(self.scan_route) - 1:
            if self.current_node == 1:
                self.mode = MODE_FINISHED
                return CMD_FINISHED
            # Route complete but not at home → return home
            self._ensure_return_home_path()
            return self._path_move_command()

        # Calculate next node/heading and corresponding turn command
        next_node = self.scan_route[self.scan_route_index + 1]
        next_heading = self._heading_between(self.current_node, next_node)
        return _turn_action(self.current_heading, next_heading)

    def _path_move_command(self):
        """
        Determine next move command to follow an active path (delivery/resume/return-home).
        :return: Move command (CMD_LEFT/RIGHT/STRAIGHT/180) or CMD_FINISHED/DROP (path complete)
        """
        # Validate active path exists
        if self.path_nodes is None or self.path_index is None:
            raise ValueError("No active path")

        # Check if path is complete
        if self.path_index >= len(self.path_nodes) - 1:
            # Resume mode → switch back to scan mode
            if self.mode == MODE_RESUME:
                self.mode = MODE_SCAN
                self.scan_route_index = self.resume_target_route_index
                self.path_nodes = None
                self.path_index = None
                self.resume_target_slot = None
                self.resume_target_route_index = None
                return self._decide_next_command()
            # Return-home mode → finish mission if at node 1
            if self.mode == MODE_RETURN_HOME:
                if self.current_node != 1:
                    raise ValueError("Return-home path ended away from node 1")
                self.mode = MODE_FINISHED
                return CMD_FINISHED
            # Delivery mode → drop reel at target
            if self.mode == MODE_DELIVERY:
                if not self._at_delivery_target():
                    raise ValueError("Delivery path ended at wrong node")
                return CMD_DROP
            # Unsupported mode (error)
            raise ValueError("Path ended in unsupported mode '{}'".format(self.mode))

        # Calculate next node/heading and corresponding turn command
        next_node = self.path_nodes[self.path_index + 1]
        next_heading = self._heading_between(self.current_node, next_node)
        return _turn_action(self.current_heading, next_heading)

    # --------------------------------------------------------------
    # Result Handling (Process Command Feedback)
    # --------------------------------------------------------------
    def _normalize_result(self, result):
        """
        Normalize diverse command results to standard constants (via RESULT_ALIASES).
        Handles string/dict results and standardizes key names/values.
        :param result: Raw result from mission runner (string/dict)
        :return: Normalized result dict with "status" key
        """
        # Handle dict results (e.g., grab commands with colour)
        if isinstance(result, dict):
            norm = {}
            for k, v in result.items():
                norm[str(k).lower()] = v
            status = norm.get("status")
            if isinstance(status, str):
                status_key = status.strip().lower()
                norm["status"] = RESULT_ALIASES.get(status_key, status_key)
            return norm

        # Handle string results (e.g., "done", "scan_empty")
        if isinstance(result, str):
            key = result.strip().lower()
            if ":" in key:
                left, right = key.split(":", 1)
                left = RESULT_ALIASES.get(left, left)
                return {"status": left, "colour": right}
            return {"status": RESULT_ALIASES.get(key, key)}

        # Unsupported result type (error)
        raise ValueError("Unsupported result type: {}".format(type(result)))

    def _apply_result(self, pending, result):
        """
        Apply the normalized result of a command to update navigator state.
        Each command type has specific state updates (e.g., follow → mark event, scan → update slot state).
        :param pending: Pending command metadata (from _attach_pending_metadata)
        :param result: Normalized result dict
        """
        cmd = pending["cmd"]
        status = result.get("status")

        # Handle FOLLOW command result (event detected)
        if cmd == CMD_FOLLOW:
            if status != RES_EVENT:
                raise ValueError("follow expected result '{}' but got '{}'".format(RES_EVENT, status))
            self.at_node_event = True
            return

        # Handle MOVE commands (LEFT/RIGHT/STRAIGHT/180)
        if cmd in (CMD_LEFT, CMD_RIGHT, CMD_STRAIGHT, CMD_180):
            if status != RES_DONE:
                raise ValueError("move command expected '{}' but got '{}'".format(RES_DONE, status))
            # Update current node/heading and clear event flag
            self.current_node = pending["next_node"]
            self.current_heading = pending["next_heading"]
            self.at_node_event = False

            # Update scan route index or path index (based on command metadata)
            if pending.get("kind") == "scan_route":
                self.scan_route_index = pending["next_index"]
            else:
                self.path_index = pending["next_index"]
            return

        # Handle SCAN commands (LEFT/RIGHT)
        if cmd in (CMD_SCAN_LEFT, CMD_SCAN_RIGHT):
            key = pending["slot_key"]
            # Scan found no reel → mark slot as empty
            if status == RES_SCAN_EMPTY:
                self.slot_state[key] = SLOT_EMPTY
                return
            # Scan found a reel → mark slot as found and set active reel state
            if status == RES_SCAN_FOUND:
                self.slot_state[key] = SLOT_FOUND
                self.active_stack = pending["stack"]
                self.active_slot = pending["slot"]
                self.active_turn = pending["turn"]
                return
            # Unsupported scan result (error)
            raise ValueError("scan command got unsupported result '{}'".format(status))

        # Handle GRAB commands (LEFT/RIGHT)
        if cmd in (CMD_GRAB_LEFT, CMD_GRAB_RIGHT):
            # Grab failed → reset active reel state
            if status == RES_GRAB_FAIL:
                self.active_stack = None
                self.active_slot = None
                self.active_turn = None
                self.carrying_colour = None
                return

            # Validate grab success
            if status != RES_GRAB_OK:
                raise ValueError("grab command got unsupported result '{}'".format(status))

            # Extract reel colour and validate delivery target
            colour = self._extract_colour(result)
            if colour not in self.delivery_target:
                raise ValueError("grab result did not contain valid colour")

            # Update state for successful grab: set carrying colour, rotate heading, build delivery path
            self.carrying_colour = colour
            self.current_heading = rotate_90(self.current_heading, pending["turn_dir"])
            self.at_node_event = True
            self._build_delivery_path(colour)
            return

        # Handle DROP command
        if cmd == CMD_DROP:
            # Validate drop success
            if status not in (RES_DROP_DONE, RES_DONE):
                raise ValueError("drop expected '{}' or '{}' but got '{}'".format(RES_DROP_DONE, RES_DONE, status))

            # Validate active reel state (should be carrying a reel)
            if self.active_stack is None or self.active_slot is None or self.carrying_colour is None:
                raise ValueError("drop completed with no active reel")

            # Mark slot as delivered and reset active reel state
            self.slot_state[(self.active_stack, self.active_slot)] = SLOT_DELIVERED
            self.active_stack = None
            self.active_slot = None
            self.active_turn = None
            self.carrying_colour = None
            self.path_nodes = None
            self.path_index = None

            # All reels delivered → return home
            if self.all_expected_delivered():
                if self.current_node == 1:
                    self.mode = MODE_FINISHED
                else:
                    self._ensure_return_home_path()
                return

            # More reels to deliver → build resume path to next scan slot
            self._build_resume_path()
            return

        # Handle FINISHED command
        if cmd == CMD_FINISHED:
            self.mode = MODE_FINISHED
            return

        # Unsupported command (error)
        raise ValueError("Unsupported pending command '{}'".format(cmd))

    def _extract_colour(self, result):
        """
        Extract valid reel colour from normalized result (handles multiple formats).
        :param result: Normalized result dict
        :return: Valid colour string (blue/green/yellow/red) or None
        """
        # Check "colour" key first
        colour = result.get("colour")
        if isinstance(colour, str):
            colour = colour.strip().lower()
            if colour in COLOUR_NAMES:
                return colour

        # Fallback: check "status" key (edge case where colour is in status)
        status = result.get("status")
        if isinstance(status, str) and status in COLOUR_NAMES:
            return status

        # No valid colour found
        return None

    def _attach_pending_metadata(self, pending):
        """
        Attach metadata to pending commands (for result processing).
        Metadata includes next node/heading (move commands), slot info (scan commands), etc.
        :param pending: Pending command dict ({"cmd": ...})
        """
        cmd = pending["cmd"]

        # Attach metadata for MOVE commands
        if cmd in (CMD_LEFT, CMD_RIGHT, CMD_STRAIGHT, CMD_180):
            # Scan route move → attach scan route metadata
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

            # Path move (delivery/resume/return-home) → attach path metadata
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

        # Attach metadata for SCAN commands
        if cmd in (CMD_SCAN_LEFT, CMD_SCAN_RIGHT):
            info = self.scan_points[self.current_node]
            pending["stack"] = info["stack"]
            pending["slot"] = info["slot"]
            pending["turn"] = info["turn"]
            pending["slot_key"] = (info["stack"], info["slot"])
            return

        # Attach metadata for GRAB commands
        if cmd in (CMD_GRAB_LEFT, CMD_GRAB_RIGHT):
            pending["turn_dir"] = self.active_turn
            return

    # --------------------------------------------------------------
    # Path Building (Shortest Path Calculation)
    # --------------------------------------------------------------
    def _build_delivery_path(self, colour):
        """
        Build shortest path from current node to delivery target for the given colour.
        Switches mode to DELIVERY and initializes path state.
        :param colour: Reel colour (blue/green/yellow/red)
        """
        target = self.delivery_target[colour]
        path = self._shortest_path(self.current_node, target)
        self.mode = MODE_DELIVERY
        self.path_nodes = path
        self.path_index = 0

    def _build_resume_path(self):
        """
        Build shortest path to resume scanning (next unresolved slot).
        Switches mode to RESUME and initializes path state (includes heading alignment).
        """
        # Find next unresolved slot (or return home if all scanned)
        slot_key = self.next_unresolved_slot()
        if slot_key is None:
            self._ensure_return_home_path()
            return

        # Get scan route index and target node/heading for the slot
        route_index = self.scan_route_index_by_slot[slot_key]
        target_node = self.scan_route[route_index]
        target_heading = self.scan_points[target_node]["scan_heading"]

        # Build path with heading alignment (pose-based shortest path)
        path = self._pose_shortest_path(
            start_node=self.current_node,
            start_heading=self.current_heading,
            goal_node=target_node,
            goal_heading=target_heading,
        )

        # Set resume mode and path state
        self.mode = MODE_RESUME
        self.path_nodes = path
        self.path_index = 0
        self.resume_target_slot = slot_key
        self.resume_target_route_index = route_index

    def _ensure_return_home_path(self):
        """
        Build shortest path from current node back to home (node 1).
        Switches mode to RETURN_HOME and initializes path state.
        """
        path = self._shortest_path(self.current_node, 1)
        self.mode = MODE_RETURN_HOME
        self.path_nodes = path
        self.path_index = 0

    # --------------------------------------------------------------
    # Geometry Helpers (Scan Sequence/Heading/Path Calculation)
    # --------------------------------------------------------------
    def _build_scan_sequence(self):
        """
        Build ordered list of scan slots (scan_sequence) and map slots to scan route indices.
        Only includes slots where the robot's heading matches the required scan heading.
        """
        current_heading = HEAD_N
        for i in range(len(self.scan_route) - 1):
            node = self.scan_route[i]
            nxt = self.scan_route[i + 1]
            # Check if current node is a scan point with matching heading
            if node in self.scan_points:
                info = self.scan_points[node]
                if current_heading == info["scan_heading"]:
                    key = (info["stack"], info["slot"])
                    self.scan_sequence.append(key)
                    self.scan_route_index_by_slot[key] = i
            # Update heading for next node in scan route
            current_heading = self._heading_between(node, nxt)

    def _heading_between(self, a, b):
        """
        Calculate heading from node a to node b (Manhattan grid only).
        :param a: Start node (int/str)
        :param b: End node (int/str)
        :return: Heading (HEAD_N/HEAD_E/HEAD_S/HEAD_W)
        """
        xa, ya = self.xy[a]
        xb, yb = self.xy[b]

        # North (y increases)
        if xb == xa and yb > ya:
            return HEAD_N
        # East (x increases)
        if xb > xa and yb == ya:
            return HEAD_E
        # South (y decreases)
        if xb == xa and yb < ya:
            return HEAD_S
        # West (x decreases)
        if xb < xa and yb == ya:
            return HEAD_W
        # Non-Manhattan edge (error)
        raise ValueError("Non-Manhattan edge: {} -> {}".format(a, b))

    def _shortest_path(self, start, goal):
        """
        Calculate shortest path between start and goal nodes using BFS (Breadth-First Search).
        :param start: Start node (int/str)
        :param goal: Goal node (int/str)
        :return: List of nodes in shortest path (start → goal)
        """
        # Trivial case: start == goal
        if start == goal:
            return [start]

        # BFS initialization
        q = [start]
        q_head = 0
        prev = {start: None}

        # BFS loop
        while q_head < len(q):
            node = q[q_head]
            q_head += 1

            # Goal found → exit loop
            if node == goal:
                break

            # Explore adjacent nodes
            for nxt in self.graph[node]:
                if nxt not in prev:
                    prev[nxt] = node
                    q.append(nxt)

        # No path found (error)
        if goal not in prev:
            raise ValueError("No path from {} to {}".format(start, goal))

        # Reconstruct path (reverse from goal to start)
        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = prev[node]
        path.reverse()
        return path

    def _pose_shortest_path(self, start_node, start_heading, goal_node, goal_heading):
        """
        Calculate shortest path that aligns heading at the goal node (pose-based BFS).
        State = (node, heading) → ensures robot faces correct direction at goal.
        :param start_node: Start node
        :param start_heading: Start heading
        :param goal_node: Goal node
        :param goal_heading: Required heading at goal node
        :return: List of nodes in pose-aligned shortest path
        """
        start_state = (start_node, start_heading)
        goal_state = (goal_node, goal_heading)

        # Trivial case: start state == goal state
        if start_state == goal_state:
            return [start_node]

        # BFS initialization (state = (node, heading))
        q = [start_state]
        q_head = 0
        prev = {start_state: None}

        # BFS loop
        while q_head < len(q):
            node, heading = q[q_head]
            q_head += 1

            # Explore adjacent nodes and calculate new heading
            for nxt in self.graph[node]:
                new_heading = self._heading_between(node, nxt)
                nxt_state = (nxt, new_heading)
                if nxt_state not in prev:
                    prev[nxt_state] = (node, heading)
                    # Goal state found → exit loop early
                    if nxt_state == goal_state:
                        q_head = len(q)
                        break
                    q.append(nxt_state)

        # No pose path found (error)
        if goal_state not in prev:
            raise ValueError(
                "No pose-path from ({}, {}) to ({}, {})".format(
                    start_node,
                    heading_name(start_heading),
                    goal_node,
                    heading_name(goal_heading),
                )
            )

        # Reconstruct path from states (extract nodes only)
        states = []
        state = goal_state
        while state is not None:
            states.append(state)
            state = prev[state]
        states.reverse()
        return [node for node, _heading in states]

    def _at_delivery_target(self):
        """
        Check if current node is the delivery target for the carried reel.
        :return: True if at target, False otherwise
        """
        if self.carrying_colour is None:
            return False
        return self.current_node == self.delivery_target[self.carrying_colour]


# Backward-compatible alias (for legacy code support)
navigation = Navigator


# Test entry point (run standalone to verify initialization)
if __name__ == "__main__":
    nav = Navigator()
    print("Navigator ready. First command:", nav.next_command())
    print(nav.debug_status())