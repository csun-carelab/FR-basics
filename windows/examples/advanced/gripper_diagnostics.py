#!/usr/bin/env python3
"""
Fairino FR5 - DH Gripper Diagnostic and Open/Close Test.

Runs diagnostics first, then tries multiple initialization strategies
before attempting to open and close the gripper.
"""

import os
import sys
import time

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "..", "..", "python-sdk"))
if SDK_DIR not in sys.path:
    sys.path.insert(0, SDK_DIR)

try:
    from fairino.Robot import RPC
except ImportError:
    try:
        from Robot import RPC
    except ImportError:
        print("[FATAL] Fairino SDK not found.")
        sys.exit(1)

ROBOT_IP = "192.168.58.2"


def call(rb, func, *args):
    """Call an SDK function and return (error_code, result)."""
    try:
        result = getattr(rb, func)(*args)
        if isinstance(result, int):
            return result, None
        if isinstance(result, (list, tuple)):
            return result[0], result[1:]
        return result, None
    except Exception as exc:
        print(f"  [EXCEPTION] {func}: {exc}")
        return -1, None


def section(title):
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print(f"{'=' * 60}")


section("CONNECTING")
print(f"Robot IP: {ROBOT_IP}")
try:
    rb = RPC(ROBOT_IP)
    print("[OK] Connected to Fairino controller.")
except Exception as exc:
    print(f"[FATAL] Connection failed: {exc}")
    sys.exit(1)

# Phase 1: Diagnostics
section("PHASE 1 - DIAGNOSTICS")

print("\n[1] GetGripperConfig()...")
err, data = call(rb, "GetGripperConfig")
print(f"    error={err}  data={data}")
if data:
    labels = ["company", "device", "softversion", "bus"]
    for label, val in zip(labels, data):
        print(f"    {label} = {val}")

print("\n[2] GetGripperActivateStatus()...")
try:
    result = rb.GetGripperActivateStatus()
    print(f"    raw result = {result}")
except Exception as exc:
    print(f"    [EXCEPTION] {exc}")

print("\n[3] GetGripperCurPosition()...")
try:
    result = rb.GetGripperCurPosition()
    print(f"    raw result = {result}")
except Exception as exc:
    print(f"    [EXCEPTION] {exc}")

print("\n[4] GetGripperMotionDone()...")
err, data = call(rb, "GetGripperMotionDone")
print(f"    error={err}  data={data}")

print("\n[5] GetAxleCommunicationParam()...")
try:
    result = rb.GetAxleCommunicationParam()
    print(f"    raw result = {result}")
    if isinstance(result, (list, tuple)) and result[0] == 0:
        param_labels = [
            "baudRate_code",
            "dataBit",
            "stopBit",
            "verify",
            "timeout_ms",
            "timeoutTimes",
            "period_ms",
        ]
        baud_map = {1: 9600, 2: 14400, 3: 19200, 4: 38400, 5: 56000, 6: 67600, 7: 115200, 8: 128000}
        for i, label in enumerate(param_labels):
            val = result[i + 1]
            extra = ""
            if label == "baudRate_code" and val in baud_map:
                extra = f"  ({baud_map[val]} baud)"
            print(f"    {label} = {val}{extra}")
except Exception as exc:
    print(f"    [EXCEPTION] {exc}")

print("\n[6] GetGripperVoltage()...")
try:
    result = rb.GetGripperVoltage()
    print(f"    raw result = {result}")
except Exception as exc:
    print(f"    [EXCEPTION] {exc}")

print("\n[7] GetGripperTemp()...")
try:
    result = rb.GetGripperTemp()
    print(f"    raw result = {result}")
except Exception as exc:
    print(f"    [EXCEPTION] {exc}")

# Phase 2: RS485 setup
section("PHASE 2 - SET RS485 COMMS (115200 baud)")
print("Setting: baudRate=7(115200), dataBit=8, stopBit=1, verify=0(None),")
print("         timeout=500ms, timeoutTimes=3, period=50ms")
err, _ = call(rb, "SetAxleCommunicationParam", 7, 8, 1, 0, 500, 3, 50)
print(f"    error={err}")
time.sleep(0.5)

# Phase 3: Initialization strategies
section("PHASE 3 - GRIPPER INITIALIZATION")

strategies = [
    {
        "name": "Strategy A: Dahuan company=4, device=0 (PGI-140 / generic DH)",
        "company": 4,
        "device": 0,
    },
    {
        "name": "Strategy B: Robotiq company=1, device=0 (matching pendant manu=97->remapped)",
        "company": 1,
        "device": 0,
    },
]

success = False

for i, strat in enumerate(strategies):
    print(f"\n--- {strat['name']} ---")

    print(f"  SetGripperConfig({strat['company']}, {strat['device']})...")
    err, _ = call(rb, "SetGripperConfig", strat["company"], strat["device"])
    print(f"    error={err}")
    time.sleep(1)

    print("  ActGripper(1, 0)  [reset]...")
    err, _ = call(rb, "ActGripper", 1, 0)
    print(f"    error={err}")
    time.sleep(1)

    print("  ActGripper(1, 1)  [activate]...")
    err, _ = call(rb, "ActGripper", 1, 1)
    print(f"    error={err}")
    time.sleep(3)

    print("  GetGripperActivateStatus()...")
    try:
        result = rb.GetGripperActivateStatus()
        print(f"    result = {result}")
    except Exception as exc:
        print(f"    [EXCEPTION] {exc}")

    print("  MoveGripper(1, 50, 50, 50, 10000, 0, 0, 0, 0, 0)  [test move to 50%]...")
    err, _ = call(rb, "MoveGripper", 1, 50, 50, 50, 10000, 0, 0, 0, 0, 0)
    print(f"    error={err}")

    if err == 0:
        print(f"\n  >>> Strategy {chr(65 + i)} SUCCEEDED! <<<")
        success = True
        break

    print(f"  Strategy {chr(65 + i)} failed (error={err}), trying next...")
    time.sleep(1)

if not success:
    print("\n--- Strategy C: Dahuan company=4, device=0, NO reset (simplified) ---")
    print("  SetGripperConfig(4, 0)...")
    err, _ = call(rb, "SetGripperConfig", 4, 0)
    print(f"    error={err}")
    time.sleep(0.5)

    print("  ActGripper(1, 1)  [activate only, no reset]...")
    err, _ = call(rb, "ActGripper", 1, 1)
    print(f"    error={err}")
    time.sleep(3)

    print("  MoveGripper(1, 50, 50, 50, 10000, 0, 0, 0, 0, 0)  [test move]...")
    err, _ = call(rb, "MoveGripper", 1, 50, 50, 50, 10000, 0, 0, 0, 0, 0)
    print(f"    error={err}")
    if err == 0:
        print("\n  >>> Strategy C SUCCEEDED! <<<")
        success = True

# Phase 4: Open/close test
section("PHASE 4 - OPEN / CLOSE TEST")

if not success:
    print("[WARN] No initialization strategy returned error=0.")
    print("       Attempting open/close anyway (gripper may still respond)...\n")

input("Press ENTER to OPEN the gripper (pos=100)...")
print("Opening gripper...")
err, _ = call(rb, "MoveGripper", 1, 100, 50, 50, 30000, 0, 0, 0, 0, 0)
print(f"  MoveGripper -> error={err}")
time.sleep(2)

try:
    result = rb.GetGripperCurPosition()
    print(f"  Current position: {result}")
except Exception:
    pass

input("\nPress ENTER to CLOSE the gripper (pos=0)...")
print("Closing gripper...")
err, _ = call(rb, "MoveGripper", 1, 0, 50, 50, 30000, 0, 0, 0, 0, 0)
print(f"  MoveGripper -> error={err}")
time.sleep(2)

try:
    result = rb.GetGripperCurPosition()
    print(f"  Current position: {result}")
except Exception:
    pass

input("\nPress ENTER to OPEN again (pos=100)...")
print("Opening gripper...")
err, _ = call(rb, "MoveGripper", 1, 100, 50, 50, 30000, 0, 0, 0, 0, 0)
print(f"  MoveGripper -> error={err}")

section("DONE")
print("Review the output above to diagnose the gripper.")
print()
print("Key things to check:")
print("  - Phase 1 [5]: Is baudRate = 7 (115200)? If not, RS485 mismatch.")
print("  - Phase 1 [2]: Is gripper_active bit0 = 1? If 0, gripper never activated.")
print("  - Phase 1 [6]: Does voltage read > 0? If 0, gripper may not have power.")
print("  - Phase 3: Which strategy error codes did you get?")
print("  - Phase 4: Did the gripper physically move?")
print()
print("If ALL strategies fail with timeout, the likely causes are:")
print("  1. Wrong device code (DH-3 may not be device=0)")
print("  2. RS485 wiring / slave ID mismatch")
print("  3. Gripper needs activation from teach pendant first")
