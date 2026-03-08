# Trust Before Touch

A **Physical Challenge-Response System (PCS)** for [SO-101](https://github.com/TheRobotStudio/SO-101) robot arms that verifies mechanical trustworthiness before granting collaborative handoff control. Inspired by cryptographic challenge-response protocols, one arm must *prove* it can follow randomized motion commands before another arm will hand over an object.

```
  Leader                Prover               Verifier
    │                     │                     │
    │  1. CLAIM           │                     │
    │◄────────────────────┤                     │
    │                     │                     │
    │  2. CHALLENGE       │                     │
    ├────────────────────►│                     │
    │                     │                     │
    │  3. RESPONSE        │  telemetry stream   │
    │                     ├────────────────────►│
    │                     │                     │
    │  4. VERIFY          │                     │
    │◄────────────────────────────────────────── │
    │                     │                     │
    │  GRANT / DENY       │                     │
    ├────────────────────►│                     │
```

---

## How It Works

The protocol runs in **four phases** across three independent robot arms:

| Phase | What Happens |
|-------|-------------|
| **CLAIM** | The Prover announces its current end-effector pose and whether it's holding an object |
| **CHALLENGE** | The Leader generates a randomized motion task for the Prover to execute |
| **RESPONSE** | The Prover executes the challenge while streaming joint telemetry and camera frames |
| **VERIFY** | The Verifier scores the response on trajectory accuracy, timing, and visual confirmation |

A **handoff is granted** when the composite trust score meets the threshold (default: **0.70**).

---

## Challenge Types

The system randomly selects from three challenge types, each testing different capabilities:

### Micro-Trajectory
Trace a sequence of random waypoints through joint space. Tests smooth, accurate multi-joint coordination.
- 3 waypoints, each up to +/-15 degrees from current pose
- 1.5 seconds per waypoint segment

### Tap Rhythm
Open and close the gripper in a specific rhythmic pattern. Tests precise timing control.
- 3-6 taps at a randomized tempo (40-80 BPM)

### Pose and Present
Move to a named end-effector pose and hold steady. Tests stability and positioning accuracy.
- 5 target poses: `reach_forward`, `reach_left`, `reach_right`, `reach_high`, `handoff_ready`
- 2-second hold duration

---

## Scoring

Responses are evaluated with three weighted sub-scores:

| Component | Weight | Measures |
|-----------|--------|----------|
| **Trajectory** | 50% | Joint position accuracy vs. expected waypoints (normalized RMSE with exponential decay) |
| **Timing** | 30% | Whether waypoints/taps/holds were reached within acceptable time windows |
| **Visual** | 20% | Object presence detection via wrist camera |

All weights and thresholds are configurable in [`config.py`](config.py).

---

## Quick Start

### Prerequisites

- Python 3.7+
- Three SO-101 robot arms (or use `--mock` for simulation)

### Installation

```bash
git clone https://github.com/rvazdev-ex/trust_before_touch.git
cd trust_before_touch
pip install -r requirements.txt
```

### Run in Simulation (No Hardware)

```bash
python main.py --mock
```

Run multiple rounds with verbose output:

```bash
python main.py --mock --rounds 5 --verbose
```

### Run with Hardware

Connect three SO-101 arms via USB and run:

```bash
python main.py \
    --leader-port /dev/ttyACM0 \
    --prover-port /dev/ttyACM1 \
    --verifier-port /dev/ttyACM2 \
    --verifier-camera 0
```

---

## CLI Reference

```
usage: main.py [-h] [--mock] [--rounds N] [--threshold T]
               [--leader-port PORT] [--prover-port PORT]
               [--verifier-port PORT] [--verifier-camera IDX]
               [--no-verifier-camera] [-v]

options:
  --mock                  Run in simulation mode (no hardware required)
  --rounds N              Number of PCS rounds to execute (default: 1)
  --threshold T           Trust score threshold for handoff (default: 0.70)
  --leader-port PORT      USB serial port for Leader arm
  --prover-port PORT      USB serial port for Prover arm
  --verifier-port PORT    USB serial port for Verifier arm
  --verifier-camera IDX   OpenCV camera index for Verifier wrist cam
  --no-verifier-camera    Disable wrist camera on Verifier
  -v, --verbose           Enable debug logging
```

---

## Project Structure

```
trust_before_touch/
├── main.py                 # CLI entry point and multi-round orchestration
├── config.py               # All tunable parameters and hardware config
├── requirements.txt        # Python dependencies
├── LICENSE                 # MIT License
└── pcs/
    ├── __init__.py
    ├── protocol.py         # 4-phase PCS coordinator and arm control loops
    ├── challenge_gen.py    # Randomized challenge generation
    ├── arm_interface.py    # Hardware abstraction (real + mock arms)
    └── verifier.py         # Scoring engine and trust evaluation
```

---

## Architecture

### Three-Arm Roles

| Role | Responsibility | Hardware |
|------|---------------|----------|
| **Leader** | Generates challenges based on Prover's claim | SO-101 via USB serial |
| **Prover** | Executes challenges and streams telemetry | SO-101 via USB serial |
| **Verifier** | Observes and scores the Prover's response | SO-101 via USB serial + wrist camera |

### Concurrency Model

Each arm runs its own background control loop at **50 Hz** on a dedicated thread. The protocol coordinator synchronizes the phases using events and shared state. All timing, scoring, and hardware parameters are centralized in `config.py`.

### Mock Mode

The `MockArmInterface` provides full software simulation:

- First-order joint dynamics with a 0.15s time constant
- Simulated servo lag and encoder quantization noise
- Synthetic camera frames with green-blob object detection
- Deterministic enough for testing, noisy enough to be realistic

---

## Configuration

Key parameters in [`config.py`](config.py):

```python
# Timing
CONTROL_HZ = 50              # Joint control loop frequency
CHALLENGE_TIMEOUT_S = 12.0   # Max time for Prover response
INTERPHASE_PAUSE_S = 0.5     # Pause between phases

# Scoring weights
W_TRAJECTORY = 0.50          # Trajectory shape accuracy
W_TIMING = 0.30              # Timing adherence
W_VISUAL = 0.20              # Visual object presence
TRUST_THRESHOLD = 0.70       # Minimum score for handoff

# Challenge parameters
MICRO_TRAJ_WAYPOINTS = 3
MICRO_TRAJ_AMPLITUDE_DEG = 15.0
TAP_BPM_RANGE = (40, 80)
POSE_HOLD_S = 2.0
```

---

## Dependencies

| Package | Purpose |
|---------|---------|
| [lerobot](https://github.com/huggingface/lerobot) >= 0.1.0 | SO-101 arm control via LeRobot |
| [numpy](https://numpy.org/) >= 1.24.0 | Numerical computation for scoring |
| [opencv-python](https://opencv.org/) >= 4.8.0 | Wrist camera frame capture and processing |

---

## License

[MIT](LICENSE)
