# Fairino FR5 — Windows Teleoperation

Xbox controller teleoperation and trajectory replay for the Fairino FR5 on Windows.

## Contents

| File | Description |
|------|-------------|
| `fairino_xbox_windows.py` | Xbox controller jogging + trajectory recording |
| `fairino_trajectory_replay.py` | Replay a recorded CSV trajectory |
| `run_fairino_xbox.bat` | Double-click launcher for the Xbox script |
| `fairino_xbox_windows.md` | Full Xbox controller usage guide |
| `fairino_trajectory_replay.md` | Full trajectory replay usage guide |
| `requirements.txt` | Python dependencies |

---

## Setup

### 1. Set up the Fairino Windows SDK

The SDK is included in this repo under `fairino-python-sdk-main/windows/fairino/`.
It is a pure Python package — no compilation needed.

**Option A — copy into your working directory** (simplest):
```
xcopy /E /I fairino-python-sdk-main\windows\fairino windows_teleop\fairino
```

**Option B — add to PYTHONPATH permanently**:
```
set PYTHONPATH=%PYTHONPATH%;C:\path\to\FR-basics\fairino-python-sdk-main\windows
```

> `setup.py` is included only if you want to compile a Cython `.pyd` extension
> (requires Cython + MSVC). Not needed for normal use.

### 2. Install Python dependencies

```
pip install -r windows_teleop/requirements.txt
```

This installs the `inputs` library for Xbox controller support.

### 3. Network

Connect your PC to the same network as the FR5. Default robot IP: `192.168.58.2`.

---

## Quick Start

### Xbox Controller Teleoperation

```
python windows_teleop\fairino_xbox_windows.py
```

Or double-click `run_fairino_xbox.bat`.

See `fairino_xbox_windows.md` for the full controller map.

### Trajectory Replay

```
python windows_teleop\fairino_trajectory_replay.py --file trajectory_log.csv --speed 30
```

See `fairino_trajectory_replay.md` for options and CSV format.
