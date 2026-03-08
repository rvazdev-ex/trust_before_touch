"""
challenge_gen.py – Randomised motion challenges for Phase 2 of PCS.

Three challenge types:
  MicroTrajectory   – N waypoints through joint-space the Prover must trace.
  TapRhythm         – Gripper open/close at a secret BPM pattern.
  PoseAndPresent    – Move to a named end-effector pose and hold steady.

Each Challenge carries its full specification so the Verifier can score the
Prover's response without additional context.
"""
from __future__ import annotations

import random
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional

import numpy as np

from config import (
    HOME_POSE_DEG,
    JOINT_LIMITS_DEG,
    JOINT_NAMES,
    MICRO_TRAJ_AMPLITUDE_DEG,
    MICRO_TRAJ_WAYPOINT_DURATION_S,
    MICRO_TRAJ_WAYPOINTS,
    NUM_JOINTS,
    POSE_HOLD_S,
    TAP_BPM_RANGE,
    TAP_COUNT_RANGE,
)


# ── Challenge type enum ───────────────────────────────────────────────────────

class ChallengeType(Enum):
    MICRO_TRAJECTORY = auto()
    TAP_RHYTHM = auto()
    POSE_AND_PRESENT = auto()


# ── Data containers ───────────────────────────────────────────────────────────

@dataclass
class Challenge:
    """Base container; every challenge carries a unique ID and creation time."""
    challenge_type: ChallengeType
    challenge_id: str = field(default_factory=lambda: f"ch-{int(time.time()*1000) % 100000:05d}")
    created_at: float = field(default_factory=time.monotonic)

    # Total expected execution time (seconds), computed by each subtype.
    expected_duration_s: float = 0.0


@dataclass
class MicroTrajectoryChallenge(Challenge):
    """
    Prover must trace a short joint-space path through N waypoints.

    waypoints: list of NUM_JOINTS-length arrays (degrees)
    segment_duration_s: time budget per waypoint-to-waypoint segment
    """
    waypoints: List[np.ndarray] = field(default_factory=list)
    segment_duration_s: float = MICRO_TRAJ_WAYPOINT_DURATION_S

    def __post_init__(self):
        self.challenge_type = ChallengeType.MICRO_TRAJECTORY
        self.expected_duration_s = len(self.waypoints) * self.segment_duration_s


@dataclass
class TapRhythmChallenge(Challenge):
    """
    Prover must tap the gripper open/close at a specified BPM for N beats.

    beat_times_s: list of timestamps (relative to challenge start) at which
                  the gripper should be fully closed (tap moment).
    open_deg:  gripper value for "open" (default 80 %)
    closed_deg: gripper value for "closed" / tap (default 10 %)
    """
    beat_times_s: List[float] = field(default_factory=list)
    open_deg: float = 80.0
    closed_deg: float = 10.0
    bpm: float = 60.0
    n_beats: int = 4

    def __post_init__(self):
        self.challenge_type = ChallengeType.TAP_RHYTHM
        # Add a full beat of silence after the last tap
        self.expected_duration_s = (
            self.beat_times_s[-1] + (60.0 / self.bpm) if self.beat_times_s else 0.0
        )


@dataclass
class PoseAndPresentChallenge(Challenge):
    """
    Prover moves to a named pose and holds it steady for ``hold_s`` seconds.

    target_pose: NUM_JOINTS-length array (degrees)
    hold_s: how long to hold the pose
    """
    target_pose: np.ndarray = field(default_factory=lambda: np.array(HOME_POSE_DEG))
    pose_name: str = "unknown"
    hold_s: float = POSE_HOLD_S
    move_duration_s: float = 2.0

    def __post_init__(self):
        self.challenge_type = ChallengeType.POSE_AND_PRESENT
        self.expected_duration_s = self.move_duration_s + self.hold_s


# ── Named poses for PoseAndPresent ────────────────────────────────────────────

NAMED_POSES: dict[str, np.ndarray] = {
    "reach_forward": np.array([0.0,   30.0, -45.0,  30.0,   0.0,  50.0]),
    "reach_left":    np.array([45.0,  20.0, -30.0,  20.0,  10.0,  50.0]),
    "reach_right":   np.array([-45.0, 20.0, -30.0,  20.0, -10.0,  50.0]),
    "reach_high":    np.array([0.0,  -45.0, -20.0, -20.0,   0.0,  50.0]),
    "handoff_ready": np.array([0.0,   10.0, -60.0,  50.0,   0.0,  70.0]),
}


# ── Generators ────────────────────────────────────────────────────────────────

def _random_joint_offset(amplitude: float, joint_idx: int) -> float:
    """Random displacement ±amplitude, clamped so the result stays in limits."""
    lo, hi = JOINT_LIMITS_DEG[joint_idx]
    return float(np.clip(
        np.random.uniform(-amplitude, amplitude), lo, hi
    ))


def generate_micro_trajectory(
    current_pose: np.ndarray,
    n_waypoints: int = MICRO_TRAJ_WAYPOINTS,
    amplitude_deg: float = MICRO_TRAJ_AMPLITUDE_DEG,
    segment_duration_s: float = MICRO_TRAJ_WAYPOINT_DURATION_S,
    rng: Optional[np.random.Generator] = None,
) -> MicroTrajectoryChallenge:
    """
    Generate a random N-waypoint micro-trajectory near ``current_pose``.
    Waypoints are bounded to ±amplitude_deg from the start pose, with
    joint-limit clamping, and the last waypoint returns close to the start.
    """
    if rng is None:
        rng = np.random.default_rng()

    # Joints to perturb: all except gripper (last) to keep object stable
    movable = list(range(NUM_JOINTS - 1))  # 0..4
    waypoints: List[np.ndarray] = []

    for w in range(n_waypoints):
        wp = current_pose.copy()
        for j in movable:
            lo, hi = JOINT_LIMITS_DEG[j]
            offset = float(rng.uniform(-amplitude_deg, amplitude_deg))
            wp[j] = float(np.clip(current_pose[j] + offset, lo, hi))

        # Last waypoint: nudge back toward start so Prover ends near origin
        if w == n_waypoints - 1:
            wp = current_pose.copy() + rng.uniform(-3.0, 3.0, size=NUM_JOINTS)
            wp = np.clip(
                wp,
                [lo for lo, hi in JOINT_LIMITS_DEG],
                [hi for lo, hi in JOINT_LIMITS_DEG],
            )

        waypoints.append(wp)

    return MicroTrajectoryChallenge(
        challenge_type=ChallengeType.MICRO_TRAJECTORY,
        waypoints=waypoints,
        segment_duration_s=segment_duration_s,
    )


def generate_tap_rhythm(
    bpm: Optional[float] = None,
    n_beats: Optional[int] = None,
    rng: Optional[np.random.Generator] = None,
) -> TapRhythmChallenge:
    """
    Generate a tap-rhythm challenge: N gripper taps at a randomised BPM.
    Beat times are quantised to the nearest 10ms for realism.
    """
    if rng is None:
        rng = np.random.default_rng()

    lo_bpm, hi_bpm = TAP_BPM_RANGE
    lo_n, hi_n = TAP_COUNT_RANGE

    if bpm is None:
        bpm = float(rng.integers(lo_bpm, hi_bpm + 1))
    if n_beats is None:
        n_beats = int(rng.integers(lo_n, hi_n + 1))

    beat_interval_s = 60.0 / bpm
    # Start after a short lead-in so the Prover has time to brace
    lead_in_s = beat_interval_s
    beat_times = [round(lead_in_s + i * beat_interval_s, 3) for i in range(n_beats)]

    return TapRhythmChallenge(
        challenge_type=ChallengeType.TAP_RHYTHM,
        beat_times_s=beat_times,
        bpm=bpm,
        n_beats=n_beats,
        open_deg=80.0,
        closed_deg=10.0,
    )


def generate_pose_and_present(
    pose_name: Optional[str] = None,
    rng: Optional[np.random.Generator] = None,
) -> PoseAndPresentChallenge:
    """Pick a random named pose (or the specified one) and ask Prover to hold it."""
    if rng is None:
        rng = np.random.default_rng()

    names = list(NAMED_POSES.keys())
    if pose_name is None or pose_name not in NAMED_POSES:
        pose_name = str(rng.choice(names))

    return PoseAndPresentChallenge(
        challenge_type=ChallengeType.POSE_AND_PRESENT,
        target_pose=NAMED_POSES[pose_name].copy(),
        pose_name=pose_name,
        hold_s=POSE_HOLD_S,
        move_duration_s=2.0,
    )


# ── Top-level randomiser ──────────────────────────────────────────────────────

def generate_challenge(
    current_pose: np.ndarray,
    challenge_type: Optional[ChallengeType] = None,
    seed: Optional[int] = None,
) -> Challenge:
    """
    Generate a random (or specified) challenge given the Prover's current pose.

    Args:
        current_pose:   Prover's current joint positions (degrees, shape NUM_JOINTS).
        challenge_type: Force a specific type, or None for uniform random.
        seed:           RNG seed for reproducibility (useful in tests).

    Returns:
        A fully-specified Challenge instance ready to be sent to the Prover.
    """
    rng = np.random.default_rng(seed)

    if challenge_type is None:
        challenge_type = rng.choice(list(ChallengeType))  # type: ignore[arg-type]

    if challenge_type == ChallengeType.MICRO_TRAJECTORY:
        return generate_micro_trajectory(current_pose, rng=rng)
    if challenge_type == ChallengeType.TAP_RHYTHM:
        return generate_tap_rhythm(rng=rng)
    if challenge_type == ChallengeType.POSE_AND_PRESENT:
        return generate_pose_and_present(rng=rng)

    raise ValueError(f"Unknown challenge type: {challenge_type}")


def describe_challenge(ch: Challenge) -> str:
    """Human-readable one-liner for logging."""
    if isinstance(ch, MicroTrajectoryChallenge):
        return (
            f"MicroTrajectory [{ch.challenge_id}]: "
            f"{len(ch.waypoints)} waypoints × {ch.segment_duration_s:.1f}s "
            f"= {ch.expected_duration_s:.1f}s total"
        )
    if isinstance(ch, TapRhythmChallenge):
        return (
            f"TapRhythm [{ch.challenge_id}]: "
            f"{ch.n_beats} beats @ {ch.bpm:.0f} BPM "
            f"= {ch.expected_duration_s:.1f}s total"
        )
    if isinstance(ch, PoseAndPresentChallenge):
        return (
            f"PoseAndPresent [{ch.challenge_id}]: "
            f"pose={ch.pose_name!r}, hold={ch.hold_s:.1f}s, "
            f"total={ch.expected_duration_s:.1f}s"
        )
    return f"Challenge [{ch.challenge_id}]: {ch.challenge_type.name}"
