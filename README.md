# trust_before_touch

**A Physical Challenge–Response System (PCS) for SO-101 robot arms.**

Before one robot arm hands off control (or an object) to another, it must first *earn trust* — by completing a randomised physical challenge while a third arm watches and scores the response.

---

## Overview

`trust_before_touch` implements a cryptography-inspired challenge–response protocol, but in the physical world. Three SO-101 robot arms collaborate:

| Role | Responsibility |
|------|---------------|
| **Leader** | Generates a randomised motion challenge |
| **Prover** | Executes the challenge to prove identity/capability |
| **Verifier** | Observes the response and computes a trust score |

A handoff is only **granted** when the Prover's composite trust score meets or exceeds the configured threshold (`0.70` by default).

---

## Protocol Phases

```
┌──────────┐    ┌──────────┐    ┌──────────────┐
│  Prover  │    │  Leader  │    │   Verifier   │
└────┬─────┘    └────┬─────┘    └──────┬───────┘
     │               │                 │
     │  1. CLAIM     │                 │
     │ ─────────────>│                 │
     │               │                 │
     │  2. CHALLENGE │                 │
     │ <─────────────│                 │
     │               │                 │
     │  3. RESPONSE  │  (Verifier watches)
     │ ─────────────────────────────> │
     │               │                 │
     │               │  4. VERIFY      │
     │               │ <───────────────│
     │               │  GRANTED / DENIED
```

1. **CLAIM** — Prover announces its current end-effector pose and whether it's holding an object.
2. **CHALLENGE** — Leader inspects the claim and generates a randomised motion task.
3. **RESPONSE** — Prover executes the challenge; joint states and camera frames are recorded. The Verifier simultaneously watches with its wrist camera.
4. **VERIFY** — Verifier scores the response across three dimensions and emits a pass/fail verdict.

---

## Challenge Types

### MicroTrajectory
The Prover must trace a short joint-space path through N random waypoints within per-segment time windows.

### TapRhythm
The Prover must open and close its gripper at a secret BPM for a specified number of beats.

### PoseAndPresent
The Prover must move to a named end-effector pose and hold it steady for a configurable duration.

---

## Trust Scoring

The Verifier produces a **composite score** from three weighted sub-scores:

| Sub-score | Weight | Measures |
|-----------|--------|---------|
| `trajectory` | 0.50 | How closely the Prover traced the reference path (normalised RMSE → exponential decay) |
| `timing` | 0.30 | Whether waypoints / taps were hit within their time windows |
| `visual` | 0.20 | Whether an object remained visually present throughout (green-channel blob detector) |

```
composite = 0.50 × trajectory + 0.30 × timing + 0.20 × visual
handoff   = composite ≥ 0.70
```

---

## Architecture

```
trust_before_touch/
├── main.py            # Entry point — argument parsing, config building, session loop
├── config.py          # All tunable constants (ports, timing, thresholds, joint limits)
├── requirements.txt
└── pcs/
    ├── arm_interface.py  # ArmInterface ABC + LeRobotArmInterface + MockArmInterface
    ├── challenge_gen.py  # Challenge dataclasses + randomised generators
    ├── protocol.py       # 4-phase PCS coordinator + per-arm background control loops
    └── verifier.py       # VerificationEngine + per-type scoring functions
```

Key design decisions:
- **Hardware-agnostic** — `MockArmInterface` simulates first-order servo dynamics (no hardware needed for development).
- **Concurrent control** — each arm runs its read loop in a daemon thread; the protocol coordinator lives on the main thread and synchronises via events.
- **Pluggable challenges** — add a new `Challenge` subclass, a generator, and scoring functions; the rest of the system picks it up automatically.

---

## Requirements

- Python ≥ 3.10
- [LeRobot](https://github.com/huggingface/lerobot) ≥ 0.1.0 (SO-101 driver)
- NumPy ≥ 1.24
- OpenCV ≥ 4.8 (wrist camera; optional in mock mode)

```bash
pip install -r requirements.txt
```

---

## Quick Start

### Simulation mode (no hardware required)

```bash
# Single round
python main.py --mock

# Five rounds with verbose logging
python main.py --mock --rounds 5 --verbose

# Lower the trust threshold so rounds pass more easily
python main.py --mock --rounds 3 --threshold 0.50
```

Sample output:
```
10:42:01  INFO     main  Starting PCS  mode=SIMULATION  rounds=3  threshold=0.70
...
══════════════════════════════════════════════════
  PCS SESSION SUMMARY  (3 rounds)
══════════════════════════════════════════════════
  Granted : 2
  Denied  : 1

  [round-001]  MICRO_TRAJECTORY      score=0.812  GRANTED ✓
  [round-002]  TAP_RHYTHM            score=0.643  DENIED  ✗
  [round-003]  POSE_AND_PRESENT      score=0.774  GRANTED ✓
══════════════════════════════════════════════════
```

### Real hardware

Edit the port assignments in `config.py` or pass them as flags:

```bash
python main.py \
    --leader-port   /dev/ttyACM0 \
    --prover-port   /dev/ttyACM1 \
    --verifier-port /dev/ttyACM2 \
    --verifier-camera 0
```

Disable the wrist camera if it is not fitted:

```bash
python main.py --leader-port /dev/ttyACM0 \
               --prover-port /dev/ttyACM1 \
               --verifier-port /dev/ttyACM2 \
               --no-verifier-camera
```

---

## CLI Reference

```
usage: main.py [-h] [--mock] [--rounds N] [--threshold T]
               [--leader-port PORT] [--prover-port PORT]
               [--verifier-port PORT] [--verifier-camera IDX]
               [--no-verifier-camera] [--verbose]

options:
  --mock                Run in simulation mode (no hardware required)
  --rounds N            Number of PCS rounds to run (default: 1)
  --threshold T         Trust score threshold for handoff (default: 0.70)
  --leader-port PORT    Serial port for the Leader arm (default: /dev/ttyACM0)
  --prover-port PORT    Serial port for the Prover arm (default: /dev/ttyACM1)
  --verifier-port PORT  Serial port for the Verifier arm (default: /dev/ttyACM2)
  --verifier-camera IDX OpenCV camera index for the Verifier wrist cam (default: 0)
  --no-verifier-camera  Disable wrist camera on the Verifier arm
  -v, --verbose         Enable DEBUG-level logging
```

---

## Configuration

All constants live in `config.py` and can be overridden at runtime via CLI flags or by editing the file directly.

| Constant | Default | Description |
|----------|---------|-------------|
| `CONTROL_HZ` | `50` | Joint read/command rate (Hz) |
| `CHALLENGE_TIMEOUT_S` | `12.0` | Hard deadline for Prover response (s) |
| `TRUST_THRESHOLD` | `0.70` | Minimum composite score to grant handoff |
| `W_TRAJECTORY` | `0.50` | Weight of trajectory sub-score |
| `W_TIMING` | `0.30` | Weight of timing sub-score |
| `W_VISUAL` | `0.20` | Weight of visual sub-score |
| `MICRO_TRAJ_WAYPOINTS` | `3` | Waypoints per MicroTrajectory challenge |
| `MICRO_TRAJ_AMPLITUDE_DEG` | `15.0` | Max joint deviation per waypoint (°) |
| `TAP_BPM_RANGE` | `(40, 80)` | BPM range for TapRhythm challenges |
| `TAP_COUNT_RANGE` | `(3, 6)` | Beat count range for TapRhythm challenges |
| `POSE_HOLD_S` | `2.0` | Hold duration for PoseAndPresent (s) |

---

## Programmatic API

```python
from config import MOCK_CONFIG
from pcs.protocol import PhysicalChallengeResponseSystem

pcs = PhysicalChallengeResponseSystem(MOCK_CONFIG)
pcs.start()

result = pcs.run_round(round_id="my-round-001")

print(result.trust.report())
# Trust score: 0.812  [GRANTED ✓]
#   trajectory=0.901 (w=0.5)
#   timing    =0.750 (w=0.3)
#   visual    =0.800 (w=0.2)
#   threshold =0.7

pcs.stop()
```

---

## License

MIT — see [LICENSE](LICENSE).
