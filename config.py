"""
PCS configuration: ports, timing constants, trust thresholds.
Edit ArmConfig entries to match your actual USB port assignments.
"""
from dataclasses import dataclass, field

# ── Control timing ────────────────────────────────────────────────────────────
CONTROL_HZ: int = 50              # Joint command / read rate
CHALLENGE_TIMEOUT_S: float = 12.0 # Hard deadline for Prover response
INTERPHASE_PAUSE_S: float = 0.5   # Brief pause between protocol phases

# ── Scoring ───────────────────────────────────────────────────────────────────
TRUST_THRESHOLD: float = 0.70     # ≥ this → handoff GRANTED
W_TRAJECTORY: float = 0.50        # Weight: trajectory shape accuracy
W_TIMING: float = 0.30            # Weight: timing adherence
W_VISUAL: float = 0.20            # Weight: visual object presence

# ── Challenge generation ──────────────────────────────────────────────────────
MICRO_TRAJ_WAYPOINTS: int = 3     # Waypoints in a MicroTrajectory challenge
MICRO_TRAJ_AMPLITUDE_DEG: float = 15.0   # Max joint deviation from current pose
MICRO_TRAJ_WAYPOINT_DURATION_S: float = 1.5  # Time per waypoint segment

TAP_COUNT_RANGE: tuple = (3, 6)   # Min/max taps in a TapRhythm challenge
TAP_BPM_RANGE: tuple = (40, 80)   # BPM range for tap rhythm

POSE_HOLD_S: float = 2.0          # Hold time in PoseAndPresent challenge

# ── Joint definitions ─────────────────────────────────────────────────────────
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
NUM_JOINTS = len(JOINT_NAMES)

# Per-joint safe operating range [deg].  Gripper is 0–100 (open %).
JOINT_LIMITS_DEG = [
    (-90.0,  90.0),   # shoulder_pan
    (-90.0,  90.0),   # shoulder_lift
    (-90.0,  90.0),   # elbow_flex
    (-90.0,  90.0),   # wrist_flex
    (-180.0, 180.0),  # wrist_roll
    (  0.0, 100.0),   # gripper
]

# Safe home / rest pose (degrees)
HOME_POSE_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 50.0]


@dataclass
class ArmConfig:
    role: str           # "leader" | "prover" | "verifier"
    port: str           # e.g. "/dev/ttyACM0"
    id: str             # Stable LeRobot id used for calibration lookup
    has_camera: bool = False
    camera_index: int = 0   # OpenCV camera index for wrist cam
    mock: bool = False       # True → run without real hardware (simulation)


@dataclass
class PCSConfig:
    leader: ArmConfig
    prover: ArmConfig
    verifier: ArmConfig
    trust_threshold: float = TRUST_THRESHOLD
    control_hz: int = CONTROL_HZ
    challenge_timeout_s: float = CHALLENGE_TIMEOUT_S


# ── Default config (override ports for your setup) ───────────────────────────
DEFAULT_CONFIG = PCSConfig(
    leader=ArmConfig(
        role="leader",
        port="/dev/ttyACM0",
        id="pcs_leader",
        mock=False,
    ),
    prover=ArmConfig(
        role="prover",
        port="/dev/ttyACM1",
        id="pcs_prover",
        has_camera=False,
        mock=False,
    ),
    verifier=ArmConfig(
        role="verifier",
        port="/dev/ttyACM2",
        id="pcs_verifier",
        has_camera=True,   # wrist camera
        camera_index=0,
        mock=False,
    ),
)

MOCK_CONFIG = PCSConfig(
    leader=ArmConfig(role="leader", port="mock", id="pcs_leader", mock=True),
    prover=ArmConfig(role="prover", port="mock", id="pcs_prover", mock=True),
    verifier=ArmConfig(role="verifier", port="mock", id="pcs_verifier", has_camera=True, mock=True),
)
