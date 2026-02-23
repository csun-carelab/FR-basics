# Fairino FR5 — Xbox Controller (Windows)

Teleoperate the Fairino FR5 robot using an Xbox controller on Windows. Supports Cartesian and joint-space jogging, gripper control, speed adjustment, and trajectory recording.

---

## Prerequisites

- Windows PC on the same network as the FR5 (default IP: `192.168.58.2`)
- Fairino Windows SDK installed (`fairino` package importable)
- Python 3.x (native Windows)
- `inputs` library: `pip install inputs`
- Xbox controller connected via USB or Bluetooth

---

## Usage

```bash
python fairino_xbox_windows.py
```

No command-line arguments. Configuration is at the top of the script.

---

## Configuration Constants

| Constant | Default | Description |
|----------|---------|-------------|
| `ROBOT_IP` | `192.168.58.2` | FR5 network address |
| `SPEED_DEFAULT` | `30` | Initial speed (%) |
| `SPEED_MIN` / `SPEED_MAX` | `10` / `100` | Speed clamp range |
| `SPEED_STEP` | `5` | Speed increment per button press |
| `DEADZONE` | `0.15` | Analog stick dead zone (normalized) |
| `TRIGGER_THRESHOLD` | `0.1` | Minimum trigger value to register motion |
| `RECORD_HZ` | `10` | Trajectory recording frequency (Hz) |
| `TRAJECTORY_FILE` | `trajectory_log.csv` | Output file for recorded trajectories |

---

## Controller Map

### Modes

| Input | Action |
|-------|--------|
| **Start** | Toggle between **CARTESIAN** and **JOINT** mode |

---

### Cartesian Mode (default)

| Input | Motion |
|-------|--------|
| Left Stick Up/Down | +/− X axis |
| Left Stick Left/Right | +/− Y axis |
| Right Stick Up/Down | +/− Z axis |
| **LB** (hold) | Pitch− (rotation axis 5) |
| **RB** (hold) | Pitch+ (rotation axis 5) |
| Left Trigger (LT) | Jaw− (rotation axis 6) |
| Right Trigger (RT) | Jaw+ (rotation axis 6) |

Release all sticks/triggers/buttons to stop motion (`StopJOG`).

---

### Joint Mode

| Input | Action |
|-------|--------|
| Left Stick Up/Down | Jog selected joint +/− |
| **LB** | Select previous joint (J1–J6) |
| **RB** | Select next joint (J1–J6) |

Release stick to stop joint motion.

---

### Gripper

| Button | Action |
|--------|--------|
| **A** | Open gripper (position 100) |
| **B** | Close gripper (position 25) |

Gripper is initialized automatically on startup (`SetGripperConfig(4,0)` → `ActGripper(1,1)`).

---

### Speed Control

| Input | Action |
|-------|--------|
| **X** | Speed − 5% |
| **Y** | Speed + 5% |
| Right Stick Left/Right | Speed −/+ 5% (0.2 s debounce) |

---

### Recording & Safety

| Input | Action |
|-------|--------|
| **Back** | Toggle trajectory recording on/off |
| **D-Pad Up** | **EMERGENCY STOP** (`ImmStopJOG`) |
| **Ctrl-C** | Graceful shutdown |

---

## Trajectory Recording

- Pressing **Back** starts recording to `trajectory_log.csv` (appends if file exists).
- Data is logged at 10 Hz with columns: `timestamp, j1, j2, j3, j4, j5, j6, x, y, z, rx, ry, rz`
- Press **Back** again to stop recording.
- The CSV can be replayed with `fairino_trajectory_replay.py`.

---

## Startup Sequence

1. Script connects to FR5 at `ROBOT_IP`.
2. Sets joint soft limits to ±175° on all 6 axes.
3. Initializes gripper (`SetGripperConfig` → `ActGripper`), allow ~2 s.
4. Waits for Xbox controller detection (polls every 1 s).
5. Prints control layout and enters the main event loop.

---

## Safety Notes

- **D-Pad Up = Emergency Stop** — cuts motion immediately via `ImmStopJOG`.
- Keep speed at or below 30% during initial testing.
- On controller disconnect, motion is halted and the script waits 2 s for reconnection.
- On any unhandled exception, `ImmStopJOG` is called automatically.
- Recording appends to the CSV — delete or rename `trajectory_log.csv` before a new session if you want a clean file.
