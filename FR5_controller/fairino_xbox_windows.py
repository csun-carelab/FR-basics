#!/usr/bin/env python3
"""
Fairino FR5 Xbox Controller - Windows Edition
"""
import csv
import os
import sys
import threading
import time

try:
    from inputs import get_gamepad, devices
except ImportError:
    print("Error: 'inputs' library not found. Run: pip install inputs")
    sys.exit(1)

try:
    from fairino.Robot import RPC
except ImportError:
    try:
        from Robot import RPC
    except ImportError:
        print("Error: Fairino SDK not found. Please install the Windows SDK.")
        sys.exit(1)

# ── CONFIGURATION ────────────────────────────────────────────────────────────
ROBOT_IP = "192.168.58.2"
SPEED_DEFAULT = 30
SPEED_MIN = 10
SPEED_MAX = 100
SPEED_STEP = 5
DEADZONE = 0.15
TRIGGER_THRESHOLD = 0.1
RECORD_HZ = 10
TRAJECTORY_FILE = "trajectory_log.csv"


class RobotController:
    def __init__(self):
        self._lock = threading.Lock()
        print(f"Connecting to Fairino at {ROBOT_IP}...")
        try:
            self.rb = RPC(ROBOT_IP)
            print("Connected to SDK.")
        except Exception as e:
            print(f"Connection Error: {e}")
            sys.exit(1)

        self.mode = "CARTESIAN"
        self.speed = SPEED_DEFAULT
        self.selected_joint = 1
        self.recording = False
        self._last_speed_adj = 0.0
        self._record_stop = threading.Event()
        self._record_thread = None
        self.running = True
        self.lb_held = False
        self.rb_held = False

        # Initial soft limits (FR5 factory ±175 deg)
        try:
            with self._lock:
                self.rb.SetLimitPositive([175.0] * 6)
                self.rb.SetLimitNegative([-175.0] * 6)
        except Exception:
            print("Note: Could not set limits (Robot may be offline)")

        # ── Gripper Initialization ───────────────────────────────────────────────
        self._init_gripper()

    def _init_gripper(self):
        """Initializes the gripper based on the working example."""
        print("-" * 20)
        print("Attempting simplified gripper initialization (Manu=4, Type=0)...")
        try:
            # From the working example, the sequence is: Config -> Activate.
            # The "Reset" call (ActGripper with 0) seems to be the issue.
            self._call("SetGripperConfig", 4, 0)
            time.sleep(0.5)
            self._call("ActGripper", 1, 1)
            time.sleep(2) # Give the gripper time to activate
            print("  [INFO] Simplified gripper initialization sequence sent.")
            print("-" * 20)
        except Exception as e:
            print(f"  [ERROR] Gripper initialization failed: {e}")

    def _call(self, func, *args):
        with self._lock:
            try:
                result = getattr(self.rb, func)(*args)
                if isinstance(result, int):
                    result = (result, None)
                return result
            except Exception:
                return (-1, None)

    # ── Gripper ──────────────────────────────────────────────────────────────

    def _open_gripper(self):
        print("Gripper: OPEN")
        self._call("MoveGripper", 1, 100, 50, 50, 30000, 0, 0, 0, 0, 0)

    def _close_gripper(self):
        print("Gripper: CLOSE")
        self._call("MoveGripper", 1, 25, 50, 50, 30000, 0, 0, 0, 0, 0)

    # ── Trajectory Recording ─────────────────────────────────────────────────

    def _record_loop(self):
        header = ["timestamp",
                  "j1", "j2", "j3", "j4", "j5", "j6",
                  "x", "y", "z", "rx", "ry", "rz"]
        file_exists = os.path.isfile(TRAJECTORY_FILE) and os.path.getsize(TRAJECTORY_FILE) > 0
        with open(TRAJECTORY_FILE, "a", newline="") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(header)
            while not self._record_stop.is_set():
                ts = time.time()
                result_j = self._call("GetActualJointPosDegree")
                result_t = self._call("GetActualTCPPose")
                try:
                    joints = result_j[1]
                    tcp = result_t[1]
                    if joints and tcp:
                        row = [ts] + list(joints) + list(tcp)
                        writer.writerow(row)
                        f.flush()
                except (TypeError, IndexError):
                    pass
                self._record_stop.wait(1.0 / RECORD_HZ)
        print("[Recording stopped]")

    def start_recording(self):
        if self.recording:
            return
        self.recording = True
        self._record_stop.clear()
        self._record_thread = threading.Thread(target=self._record_loop, daemon=True)
        self._record_thread.start()
        print(f"[Recording STARTED -> {TRAJECTORY_FILE}]")

    def stop_recording(self):
        if not self.recording:
            return
        self._record_stop.set()
        if self._record_thread:
            self._record_thread.join(timeout=2)
        self.recording = False

    # ── Controls Display ─────────────────────────────────────────────────────

    def _print_controls(self):
        print("-" * 55)
        print(f"  Mode: {self.mode}  |  Speed: {self.speed}%  |  Joint: J{self.selected_joint}")
        print("  LS = X/Y  |  RS(ud) = Z")
        print("  LB/RB = Pitch-/+ (CARTESIAN) | Prev/Next Joint (JOINT)")
        print("  LT = Jaw-  |  RT = Jaw+")
        print("  A = Open Gripper  |  B = Close Gripper")
        print("  X = Speed-  |  Y = Speed+  |  RS(lr) = Speed adj")
        print("  Back = Record  |  Start = Mode Toggle")
        print("  D-Pad Up = EMERGENCY STOP")
        print("-" * 55)

    # ── Main Loop ────────────────────────────────────────────────────────────

    def run(self):
        print("Searching for Xbox Controller...")
        while self.running and not devices.gamepads:
            time.sleep(1)

        if not devices.gamepads:
            print("No gamepad found.")
            return

        print(f"[OK] Gamepad detected: {devices.gamepads[0]}")
        self._print_controls()

        axes = {"LX": 0.0, "LY": 0.0, "RX": 0.0, "RY": 0.0, "LT": 0.0, "RT": 0.0}

        while self.running:
            try:
                events = get_gamepad()
                for event in events:
                    # ── Analog axes ──
                    if event.code == "ABS_X":
                        axes["LX"] = event.state / 32768.0
                    elif event.code == "ABS_Y":
                        axes["LY"] = event.state / 32768.0
                    elif event.code == "ABS_RX":
                        axes["RX"] = event.state / 32768.0
                    elif event.code == "ABS_RY":
                        axes["RY"] = event.state / 32768.0
                    elif event.code == "ABS_Z":
                        axes["LT"] = event.state / 255.0
                    elif event.code == "ABS_RZ":
                        axes["RT"] = event.state / 255.0

                    # ── D-Pad emergency stop ──
                    elif event.code == "ABS_HAT0Y" and event.state == -1:
                        self._call("ImmStopJOG")
                        print("** EMERGENCY STOP **")

                    # ── LB / RB hold state (pitch control in CARTESIAN) ──
                    if event.code == "BTN_TL":               # LB
                        self.lb_held = (event.state == 1)
                        if event.state == 1 and self.mode == "JOINT":
                            self.selected_joint = max(1, self.selected_joint - 1)
                            print(f"Joint: J{self.selected_joint}")
                    elif event.code == "BTN_TR":             # RB
                        self.rb_held = (event.state == 1)
                        if event.state == 1 and self.mode == "JOINT":
                            self.selected_joint = min(6, self.selected_joint + 1)
                            print(f"Joint: J{self.selected_joint}")

                    # ── Button presses (state == 1) ──
                    elif event.state == 1:
                        if event.code == "BTN_SELECT":       # Back → Record
                            if self.recording:
                                self.stop_recording()
                            else:
                                self.start_recording()
                        elif event.code == "BTN_START":      # Start → Mode Toggle
                            self._call("ImmStopJOG")
                            self.mode = "JOINT" if self.mode == "CARTESIAN" else "CARTESIAN"
                            print(f"Mode: {self.mode}")
                        elif event.code == "BTN_SOUTH":      # A
                            self._open_gripper()
                        elif event.code == "BTN_EAST":       # B
                            self._close_gripper()
                        elif event.code == "BTN_WEST":       # X
                            self.speed = max(SPEED_MIN, self.speed - SPEED_STEP)
                            print(f"Speed: {self.speed}%")
                        elif event.code == "BTN_NORTH":      # Y
                            self.speed = min(SPEED_MAX, self.speed + SPEED_STEP)
                            print(f"Speed: {self.speed}%")

                # ── RS X-axis → Speed control ──
                now = time.time()
                if abs(axes["RX"]) > DEADZONE and (now - self._last_speed_adj) > 0.2:
                    if axes["RX"] > 0:
                        self.speed = min(SPEED_MAX, self.speed + SPEED_STEP)
                    else:
                        self.speed = max(SPEED_MIN, self.speed - SPEED_STEP)
                    print(f"Speed: {self.speed}%")
                    self._last_speed_adj = now

                # ── Motion Logic ──
                moved = False
                if self.mode == "CARTESIAN":
                    if abs(axes["LY"]) > DEADZONE:
                        d = 1 if axes["LY"] > 0 else 0
                        print(f"Moving X {'+'if d else'-'}")
                        self._call("StartJOG", 2, 1, d, 100.0, self.speed, 100.0)
                        moved = True
                    elif abs(axes["LX"]) > DEADZONE:
                        d = 1 if axes["LX"] > 0 else 0
                        print(f"Moving Y {'+'if d else'-'}")
                        self._call("StartJOG", 2, 2, d, 100.0, self.speed, 100.0)
                        moved = True
                    elif abs(axes["RY"]) > DEADZONE:
                        d = 0 if axes["RY"] > 0 else 1
                        print(f"Moving Z {'+'if d else'-'}")
                        self._call("StartJOG", 2, 3, d, 100.0, self.speed, 100.0)
                        moved = True
                    elif self.lb_held:
                        print("Rotating Pitch-")
                        self._call("StartJOG", 2, 5, 0, 100.0, self.speed, 100.0)
                        moved = True
                    elif self.rb_held:
                        print("Rotating Pitch+")
                        self._call("StartJOG", 2, 5, 1, 100.0, self.speed, 100.0)
                        moved = True
                    elif axes["LT"] > TRIGGER_THRESHOLD:
                        print("Jaw-")
                        self._call("StartJOG", 2, 6, 0, 100.0, self.speed, 100.0)
                        moved = True
                    elif axes["RT"] > TRIGGER_THRESHOLD:
                        print("Jaw+")
                        self._call("StartJOG", 2, 6, 1, 100.0, self.speed, 100.0)
                        moved = True

                    if not moved:
                        self._call("StopJOG", 3)
                else:  # JOINT mode
                    if abs(axes["LY"]) > DEADZONE:
                        d = 0 if axes["LY"] > 0 else 1
                        print(f"Jogging J{self.selected_joint} {'+'if d else'-'}")
                        self._call("StartJOG", 0, self.selected_joint, d, 100.0, self.speed, 100.0)
                        moved = True
                    if not moved:
                        self._call("StopJOG", 1)

                time.sleep(0.01)
            except Exception as e:
                print(f"Error: {e}")
                self._call("ImmStopJOG")
                print("Waiting for controller reconnection...")
                time.sleep(2)


if __name__ == "__main__":
    ctrl = RobotController()
    try:
        ctrl.run()
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.stop_recording()
        ctrl._call("ImmStopJOG")
        print("Shutdown.")
