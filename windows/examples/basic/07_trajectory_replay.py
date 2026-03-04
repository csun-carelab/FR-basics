"""
07_trajectory_replay.py — Record and replay FR5 joint trajectories.

Usage:
    Replay:  python 07_trajectory_replay.py
    timestamp,j1,j2,j3,j4,j5,j6,x,y,z,rx,ry,rz

Controls during replay:
    Up arrow   = speed +5%
    Down arrow = speed -5%
    q          = stop

Requirements:
    Real FR5 robot at 192.168.58.2
    fairino Python SDK installed
    Teach pendant in AUTO mode, robot enabled
"""

import argparse
import csv
import msvcrt
import os
import sys
import threading
import time

# Add python-sdk to path
SDK_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'python-sdk')
sys.path.insert(0, os.path.abspath(SDK_DIR))

import Robot

# ── CONFIGURATION ─────────────────────────────────────────────────────────────
ROBOT_IP        = "192.168.58.2"
SPEED_DEFAULT   = 30
SPEED_MIN       = 5
SPEED_MAX       = 100
SPEED_STEP      = 5
TOOL_NUM        = 10   # must match WebApp tool selection (Tool10 = gripper)
USER_NUM        = 0
HOME_JOINTS     = [0., -60., -100., -110., 90., 0.]
RECORDINGS_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Trajectory_Recordings")
TRAJECTORY_FILE = os.path.join(RECORDINGS_DIR, "trajectory_log.csv")

# Workspace limits (mm)
X_MIN, X_MAX = 150, 650
Y_MIN, Y_MAX = -350, 350
Z_MIN, Z_MAX = 250, 750

# ── CSV LOADING ────────────────────────────────────────────────────────────────
def load_csv(file):
    waypoints = []
    with open(file, "r", newline="") as f:
        for row in csv.DictReader(f):
            waypoints.append({
                "timestamp": float(row["timestamp"]),
                "joints":    [float(row[f"j{i}"]) for i in range(1, 7)],
            })
    return waypoints


def downsample(waypoints, min_delta_deg=0.3):
    if not waypoints:
        return []
    result = [waypoints[0]]
    for wp in waypoints[1:-1]:
        if max(abs(wp["joints"][j] - result[-1]["joints"][j]) for j in range(6)) >= min_delta_deg:
            result.append(wp)
    if len(waypoints) > 1:
        result.append(waypoints[-1])
    return result

# ── MOTION ─────────────────────────────────────────────────────────────────────
def within_limits(tcp):
    return X_MIN < tcp[0] < X_MAX and Y_MIN < tcp[1] < Y_MAX and Z_MIN < tcp[2] < Z_MAX


def safe_movej(rb, joints, speed):
    """FK check then MoveJ — pattern from final_exam.py MoveJoints()."""
    err, tcp = rb.GetForwardKin(joints)
    if err != 0:
        print(f"FK failed (code {err})")
        return err
    if not within_limits(tcp):
        print(f"Target out of workspace: x={tcp[0]:.1f} y={tcp[1]:.1f} z={tcp[2]:.1f}")
        return -1
    # SDK: MoveJ(joint_pos, tool, user, vel=...) — pass speed so pendant override is not needed
    ret = rb.MoveJ(joint_pos=joints, tool=TOOL_NUM, user=USER_NUM, vel=float(speed))
    return ret[0] if isinstance(ret, tuple) else ret

# ── KEYBOARD THREAD ────────────────────────────────────────────────────────────
# Background thread so keys are captured even during blocking safe_movej calls.
_kb_lock  = threading.Lock()
_kb_speed = SPEED_DEFAULT
_kb_quit  = False


def keyboard_thread():
    global _kb_speed, _kb_quit
    while True:
        key = msvcrt.getch()
        if key in (b"\xe0", b"\x00"):          # Windows extended key prefix
            key2 = msvcrt.getch()
            with _kb_lock:
                if key2 in (b"H", b"I"):       # Up arrow
                    prev = _kb_speed
                    _kb_speed = min(SPEED_MAX, _kb_speed + SPEED_STEP)
                    if _kb_speed != prev:
                        print(f"\nSpeed increased: {prev}% -> {_kb_speed}%")
                    else:
                        print(f"\nSpeed at maximum: {_kb_speed}%")
                elif key2 in (b"P", b"Q"):     # Down arrow
                    prev = _kb_speed
                    _kb_speed = max(SPEED_MIN, _kb_speed - SPEED_STEP)
                    if _kb_speed != prev:
                        print(f"\nSpeed decreased: {prev}% -> {_kb_speed}%")
                    else:
                        print(f"\nSpeed at minimum: {_kb_speed}%")
        elif key == b"\x1b":                   # ESC — ANSI terminals (VS Code, ConEmu)
            if msvcrt.kbhit():
                key2 = msvcrt.getch()
                if key2 == b"[" and msvcrt.kbhit():
                    key3 = msvcrt.getch()
                    with _kb_lock:
                        if key3 == b"A":       # ANSI up arrow \x1b[A
                            prev = _kb_speed
                            _kb_speed = min(SPEED_MAX, _kb_speed + SPEED_STEP)
                            if _kb_speed != prev:
                                print(f"\nSpeed increased: {prev}% -> {_kb_speed}%")
                            else:
                                print(f"\nSpeed at maximum: {_kb_speed}%")
                        elif key3 == b"B":     # ANSI down arrow \x1b[B
                            prev = _kb_speed
                            _kb_speed = max(SPEED_MIN, _kb_speed - SPEED_STEP)
                            if _kb_speed != prev:
                                print(f"\nSpeed decreased: {prev}% -> {_kb_speed}%")
                            else:
                                print(f"\nSpeed at minimum: {_kb_speed}%")
        elif key in (b"q", b"Q"):
            with _kb_lock:
                _kb_quit = True
            break

# ── MAIN ───────────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="Fairino FR5 Trajectory Replayer")
parser.add_argument("--file", default=TRAJECTORY_FILE, help="CSV trajectory file")
args = parser.parse_args()

waypoints = downsample(load_csv(args.file))
print(f"Loaded {len(waypoints)} waypoints from {args.file}")
if not waypoints:
    print("No waypoints found.")
    sys.exit(0)

print(f"Connecting to {ROBOT_IP}...")
robot = Robot.RPC(ROBOT_IP)
print("Connected.")
print("NOTE: Teach pendant must be in AUTO mode and robot must be enabled.")

print(f"\nMoving to HOME {HOME_JOINTS}")
print("Press ENTER to continue or Ctrl-C to abort.")
input()

ret = safe_movej(robot, HOME_JOINTS, SPEED_DEFAULT)
if ret != 0:
    print(f"Failed to reach HOME (code {ret}). Aborting.")
    sys.exit(1)
time.sleep(2)

print(f"\nReplaying {len(waypoints)} waypoints.")
print("  Up arrow = speed+  |  Down arrow = speed-  |  q = stop")
print("Press ENTER to start or Ctrl-C to abort.\n")
input()

# Start background keyboard listener — captures keys during blocking MoveJ calls
threading.Thread(target=keyboard_thread, daemon=True).start()

commanded   = 0
completed   = 0
first_error = None

try:
    for i, cur in enumerate(waypoints):
        prev = waypoints[i - 1] if i > 0 else cur
        commanded += 1

        with _kb_lock:
            cur_speed = _kb_speed
            stop      = _kb_quit
        if stop:
            print("User stopped.")
            break

        ret = safe_movej(robot, cur["joints"], cur_speed)
        if ret != 0:
            first_error = (i, ret)
            print(f"Aborted at waypoint {i} (code {ret}).")
            break
        completed += 1

        # Wait out inter-waypoint gap while checking quit flag
        dt = max(0.0, cur["timestamp"] - prev["timestamp"])
        deadline = time.time() + dt
        while time.time() < deadline:
            with _kb_lock:
                if _kb_quit:
                    break
            time.sleep(0.01)
        with _kb_lock:
            if _kb_quit:
                print("User stopped.")
                break

except KeyboardInterrupt:
    print("Interrupted.")

finally:
    robot.StopMotion()
    if first_error is None:
        print(f"Done. {completed}/{commanded} waypoints completed.")
    else:
        print(f"Done. {completed}/{commanded} waypoints completed, "
              f"failed at waypoint {first_error[0]} (code {first_error[1]}).")
