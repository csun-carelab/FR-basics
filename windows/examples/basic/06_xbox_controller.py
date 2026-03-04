"""
06_xbox_controller.py — Xbox controller teleoperation for FR5.
A = Open gripper fully | B = Close gripper fully

Requirements:
    pip install inputs
    Xbox controller connected via USB or Bluetooth
"""

import os
import sys
import csv
import threading
import time

# Add python-sdk to path
SDK_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'python-sdk')
sys.path.insert(0, os.path.abspath(SDK_DIR))

try:
    from inputs import get_gamepad, devices
except ImportError:
    print("'inputs' library not found. Run: pip install inputs")
    sys.exit(1)

import Robot

# ── CONFIGURATION ─────────────────────────────────────────────────────────────
ROBOT_IP       = "192.168.58.2"
SPEED_DEFAULT  = 30
SPEED_MIN      = 10
SPEED_MAX      = 100
SPEED_STEP     = 5
DEADZONE       = 0.15
TRIGGER_THRESH = 0.1
RECORD_HZ      = 10
HOME_JOINTS     = [0., -60., -100., -110., 90., 0.]
RECORDINGS_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Trajectory_Recordings")
os.makedirs(RECORDINGS_DIR, exist_ok=True)
TRAJECTORY_FILE = os.path.join(RECORDINGS_DIR, "trajectory_log.csv")
TOOL_NUM        = 10   # must match WebApp tool selection (Tool10 = gripper)
USER_NUM       = 0

# Workspace limits (mm)
X_MIN, X_MAX = 150, 650
Y_MIN, Y_MAX = -350, 350
Z_MIN, Z_MAX = 250, 750

# ── STATE ──────────────────────────────────────────────────────────────────────
_lock       = threading.Lock()
_axes_lock  = threading.Lock()
axes        = {"LX": 0.0, "LY": 0.0, "RY": 0.0, "LT": 0.0, "RT": 0.0}
mode        = "CARTESIAN"
speed       = SPEED_DEFAULT
sel_joint   = 1
gripper_pos = 100
lb_held     = False
rb_held     = False
home_req    = False
running     = True

recording       = False
_record_stop    = threading.Event()
_record_thread  = None

# ── CONNECT ────────────────────────────────────────────────────────────────────
print(f"Connecting to Fairino at {ROBOT_IP}...")
try:
    robot = Robot.RPC(ROBOT_IP)
    print("Connected.")
except Exception:
    print("Could not connect to robot.")
    sys.exit(1)

# ── HELPERS ────────────────────────────────────────────────────────────────────
def call(func, *args, **kwargs):
    with _lock:
        try:
            result = getattr(robot, func)(*args, **kwargs)
            if isinstance(result, int):
                result = (result, None)
            return result
        except Exception:
            return (-1, None)

# ── INIT ───────────────────────────────────────────────────────────────────────
try:
    with _lock:
        robot.ResetAllError()
except Exception:
    pass
time.sleep(0.5)

try:
    with _lock:
        robot.SetLimitPositive([175.0] * 6)
        robot.SetLimitNegative([-175.0] * 6)
except Exception:
    pass

print("Initializing gripper...")
call("SetGripperConfig", 4, 0)
time.sleep(0.5)
call("ActGripper", 1, 1)
time.sleep(2)
print("Gripper ready.")

# ── GRIPPER ────────────────────────────────────────────────────────────────────
def open_gripper():
    global gripper_pos
    gripper_pos = 100
    print("Gripper: Open")
    # SDK: MoveGripper(index, pos, vel, force, maxtime, block, type, rotNum, rotVel, rotTorque)
    call("MoveGripper", 1, 100, 100, 50, 5000, 0, 0, 0, 0, 0)

def close_gripper():
    global gripper_pos
    gripper_pos = 0
    print("Gripper: Closed")
    call("MoveGripper", 1, 0, 100, 50, 5000, 0, 0, 0, 0, 0)

# ── RECORDING ──────────────────────────────────────────────────────────────────
def record_loop():
    header = ["timestamp", "j1", "j2", "j3", "j4", "j5", "j6",
              "x", "y", "z", "rx", "ry", "rz"]
    file_exists = os.path.isfile(TRAJECTORY_FILE) and os.path.getsize(TRAJECTORY_FILE) > 0
    with open(TRAJECTORY_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(header)
        while not _record_stop.is_set():
            ts = time.time()
            result_j = call("GetActualJointPosDegree", 0)
            result_t = call("GetActualTCPPose", 0)
            try:
                joints = result_j[1]
                tcp = result_t[1]
                if joints and tcp:
                    writer.writerow([ts] + list(joints) + list(tcp))
                    f.flush()
            except (TypeError, IndexError):
                pass
            _record_stop.wait(1.0 / RECORD_HZ)
    print("Recording stopped.")

def start_recording():
    global recording, _record_thread
    if recording:
        return
    recording = True
    _record_stop.clear()
    _record_thread = threading.Thread(target=record_loop, daemon=True)
    _record_thread.start()
    print(f"Recording started -> {TRAJECTORY_FILE}")

def stop_recording():
    global recording
    if not recording:
        return
    _record_stop.set()
    if _record_thread:
        _record_thread.join(timeout=2)
    recording = False

# ── CONTROLS DISPLAY ───────────────────────────────────────────────────────────
def print_controls():
    print("-" * 55)
    print(f"  Mode: {mode}  |  Speed: {speed}%  |  Joint: J{sel_joint}")
    print("  LS = X/Y  |  RS(ud) = Z")
    print("  LB/RB = Pitch-/+ (CARTESIAN) | Prev/Next Joint (JOINT)")
    print("  LT = Yaw-  |  RT = Yaw+")
    print("  A = Open Gripper (full)  |  B = Close Gripper (full)")
    print("  D-Pad Left = Go Home")
    print("  X = Speed-  |  Y = Speed+")
    print("  Back = Record  |  Start = Mode Toggle")
    print("  D-Pad Up = EMERGENCY STOP")
    print("-" * 55)

# ── GAMEPAD THREAD ─────────────────────────────────────────────────────────────
def process_event(event):
    global mode, speed, sel_joint, lb_held, rb_held, home_req

    with _axes_lock:
        if event.code == "ABS_X":
            axes["LX"] = event.state / 32768.0
            return
        elif event.code == "ABS_Y":
            axes["LY"] = event.state / 32768.0
            return
        elif event.code == "ABS_RY":
            axes["RY"] = event.state / 32768.0
            return
        elif event.code == "ABS_Z":
            axes["LT"] = event.state / 255.0
            return
        elif event.code == "ABS_RZ":
            axes["RT"] = event.state / 255.0
            return

    if event.code == "ABS_HAT0Y" and event.state == -1:        # D-Pad up = E-stop
        call("ImmStopJOG")
        print("** EMERGENCY STOP **")
    elif event.code == "ABS_HAT0X" and event.state == -1:      # D-Pad left = Home
        home_req = True
        print("Home requested...")

    if event.code == "BTN_TL":                                  # LB
        lb_held = (event.state == 1)
        if event.state == 1 and mode == "JOINT":
            sel_joint = max(1, sel_joint - 1)
            print(f"Joint: J{sel_joint}")
    elif event.code == "BTN_TR":                                # RB
        rb_held = (event.state == 1)
        if event.state == 1 and mode == "JOINT":
            sel_joint = min(6, sel_joint + 1)
            print(f"Joint: J{sel_joint}")
    elif event.code == "BTN_SOUTH" and event.state == 1:        # A = full open
        threading.Thread(target=open_gripper, daemon=True).start()
    elif event.code == "BTN_EAST" and event.state == 1:         # B = full close
        threading.Thread(target=close_gripper, daemon=True).start()
    elif event.state == 1:
        if event.code == "BTN_SELECT":                          # Back = Record
            if recording:
                stop_recording()
            else:
                start_recording()
        elif event.code == "BTN_START":                         # Start = Mode toggle
            call("ImmStopJOG")
            mode = "JOINT" if mode == "CARTESIAN" else "CARTESIAN"
            print(f"Mode: {mode}")
        elif event.code == "BTN_WEST":                          # X = Speed-
            speed = max(SPEED_MIN, speed - SPEED_STEP)
            print(f"Speed: {speed}%")
        elif event.code == "BTN_NORTH":                         # Y = Speed+
            speed = min(SPEED_MAX, speed + SPEED_STEP)
            print(f"Speed: {speed}%")

def gamepad_loop():
    while running:
        try:
            for event in get_gamepad():
                process_event(event)
        except Exception:
            if running:
                time.sleep(1)

# ── MAIN ───────────────────────────────────────────────────────────────────────
print("Searching for Xbox Controller...")
while not devices.gamepads:
    time.sleep(1)
print(f"Gamepad detected: {devices.gamepads[0]}")
print_controls()

threading.Thread(target=gamepad_loop, daemon=True).start()

try:
    while running:
        with _axes_lock:
            ax = dict(axes)

        if home_req:
            home_req = False
            print("Going to HOME position...")
            call("ImmStopJOG")
            for _ in range(50):
                time.sleep(0.1)
                ret = call("GetRobotMotionDone")
                if ret and ret[0] == 0 and ret[1] == 1:
                    break
            call("ResetAllError")
            time.sleep(0.5)
            fk = call("GetForwardKin", HOME_JOINTS)
            if fk[0] == 0:
                ee = fk[1]
                if X_MIN < ee[0] < X_MAX and Y_MIN < ee[1] < Y_MAX and Z_MIN < ee[2] < Z_MAX:
                    ret = call("MoveJ", joint_pos=HOME_JOINTS, tool=TOOL_NUM, user=USER_NUM)
                    print("HOME reached." if ret[0] == 0 else f"Go home failed (code {ret[0]})")
                else:
                    print("HOME position outside workspace limits.")
            else:
                print("Cannot validate home position.")
            continue

        moved = False
        if mode == "CARTESIAN":
            if abs(ax["LY"]) > DEADZONE:
                call("StartJOG", 2, 1, 1 if ax["LY"] > 0 else 0, 100.0, speed, 100.0)
                moved = True
            elif abs(ax["LX"]) > DEADZONE:
                call("StartJOG", 2, 2, 0 if ax["LX"] > 0 else 1, 100.0, speed, 100.0)
                moved = True
            elif abs(ax["RY"]) > DEADZONE:
                call("StartJOG", 2, 3, 1 if ax["RY"] > 0 else 0, 100.0, speed, 100.0)
                moved = True
            elif lb_held:
                call("StartJOG", 2, 5, 0, 100.0, speed, 100.0)
                moved = True
            elif rb_held:
                call("StartJOG", 2, 5, 1, 100.0, speed, 100.0)
                moved = True
            elif ax["LT"] > TRIGGER_THRESH:
                call("StartJOG", 2, 6, 0, 100.0, speed, 100.0)
                moved = True
            elif ax["RT"] > TRIGGER_THRESH:
                call("StartJOG", 2, 6, 1, 100.0, speed, 100.0)
                moved = True
            if not moved:
                call("StopJOG", 3)
        else:  # JOINT
            if abs(ax["LY"]) > DEADZONE:
                call("StartJOG", 0, sel_joint, 0 if ax["LY"] > 0 else 1, 100.0, speed, 100.0)
                moved = True
            if not moved:
                call("StopJOG", 1)

        time.sleep(0.01)

except KeyboardInterrupt:
    pass
finally:
    running = False
    stop_recording()
    call("ImmStopJOG")
    print("Shutdown.")
