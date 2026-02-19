#!/usr/bin/env python3
"""
Fairino FR5 Trajectory Replayer
"""

import sys
import time
import csv
import msvcrt
import argparse

try:
    from fairino.Robot import RPC
except ImportError:
    try:
        from Robot import RPC
    except ImportError:
        print("Error: Fairino SDK not found.")
        sys.exit(1)

# ── CONFIGURATION ─────────────────────────────────────────────────────────────
ROBOT_IP = "192.168.58.2"
SPEED_MIN = 5
SPEED_MAX = 100
SPEED_STEP = 5


def load_csv(file):
    waypoints = []
    with open(file, "r") as f:
        for row in csv.DictReader(f):
            waypoints.append({
                "timestamp": float(row["timestamp"]),
                "joints": [float(row[f"j{i}"]) for i in range(1, 7)],
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


def call(rb, func, *args):
    try:
        ret = getattr(rb, func)(*args)
        return (ret, None) if isinstance(ret, int) else ret
    except Exception as e:
        print(f"  [SDK] {func}: {e}")
        return (-1, None)


def main():
    parser = argparse.ArgumentParser(description="Fairino FR5 Trajectory Replayer")
    parser.add_argument("--file",  default="trajectory_log.csv", help="CSV trajectory file")
    parser.add_argument("--speed", type=int, default=30, help="Starting speed %% (default: 30)")
    args = parser.parse_args()

    speed = max(SPEED_MIN, min(SPEED_MAX, args.speed))

    # Load trajectory
    waypoints = downsample(load_csv(args.file))
    print(f"Loaded {len(waypoints)} waypoints from {args.file}")

    # Connect
    print(f"Connecting to {ROBOT_IP}...")
    rb = RPC(ROBOT_IP)
    print("Connected.")

    # Move to start
    print(f"\nReady to move to start position. Press ENTER to continue or Ctrl-C to abort.")
    while msvcrt.getch() not in (b'\r', b'\n'):
        pass
    call(rb, "SetSpeed", speed)
    call(rb, "MoveJ", waypoints[0]["joints"], 0, 0)
    time.sleep(2)

    # Replay
    print(f"\nReplaying — speed: {speed}%")
    print("  Up/Down arrows = speed +/-    |    q = stop\n")

    def poll_keys(rb, speed, dt):
        """Sleep for dt seconds while polling for keypresses. Returns updated speed or None to stop."""
        deadline = time.time() + dt
        while time.time() < deadline:
            if msvcrt.kbhit():
                key = msvcrt.getch()
                if key in (b'\xe0', b'\x00'):   # arrow key prefix (both variants)
                    key2 = msvcrt.getch()
                    if key2 == b'H':             # up arrow → speed up
                        speed = min(SPEED_MAX, speed + SPEED_STEP)
                        call(rb, "SetSpeed", speed)
                        print(f"Speed: {speed}%")
                    elif key2 == b'P':           # down arrow → speed down
                        speed = max(SPEED_MIN, speed - SPEED_STEP)
                        call(rb, "SetSpeed", speed)
                        print(f"Speed: {speed}%")
                elif key in (b'q', b'Q'):
                    return None                  # signal stop
            time.sleep(0.01)
        return speed

    try:
        for i in range(1, len(waypoints)):
            prev = waypoints[i - 1]
            cur  = waypoints[i]

            # Send waypoint (non-blocking)
            call(rb, "MoveJ", cur["joints"], 0, 0, [0.0,0.0,0.0,0.0,0.0,0.0], speed, 0.0, speed)

            # Wait the recorded interval (scaled by base speed), polling keys
            dt = cur["timestamp"] - prev["timestamp"]
            result = poll_keys(rb, speed, dt)
            if result is None:
                print("Stopping.")
                break
            speed = result

    except KeyboardInterrupt:
        pass
    finally:
        call(rb, "ImmStopJOG")
        print("Done.")


if __name__ == "__main__":
    main()
