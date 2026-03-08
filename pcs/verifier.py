"""
verifier.py – Scoring engine for Phase 4 of PCS.

Produces a TrustScore in [0, 1] from three sub-scores:
  trajectory_score  – how closely the Prover traced the reference path
  timing_score      – how well the Prover hit timing windows
  visual_score      – whether the object was visually present throughout

The weighted combination must exceed TRUST_THRESHOLD for a handoff to be granted.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from config import (
    JOINT_LIMITS_DEG,
    NUM_JOINTS,
    TRUST_THRESHOLD,
    W_TIMING,
    W_TRAJECTORY,
    W_VISUAL,
)
from pcs.arm_interface import CameraFrame, JointState
from pcs.challenge_gen import (
    Challenge,
    ChallengeType,
    MicroTrajectoryChallenge,
    PoseAndPresentChallenge,
    TapRhythmChallenge,
)

log = logging.getLogger(__name__)


# ── Result containers ─────────────────────────────────────────────────────────

@dataclass
class TrustScore:
    trajectory_score: float  # [0, 1]
    timing_score: float      # [0, 1]
    visual_score: float      # [0, 1]
    composite: float         # weighted sum → final verdict
    granted: bool            # True if composite ≥ TRUST_THRESHOLD
    notes: str = ""

    def report(self) -> str:
        verdict = "GRANTED ✓" if self.granted else "DENIED ✗"
        return (
            f"Trust score: {self.composite:.3f}  [{verdict}]\n"
            f"  trajectory={self.trajectory_score:.3f} (w={W_TRAJECTORY})\n"
            f"  timing    ={self.timing_score:.3f} (w={W_TIMING})\n"
            f"  visual    ={self.visual_score:.3f} (w={W_VISUAL})\n"
            f"  threshold ={TRUST_THRESHOLD}\n"
            + (f"  notes: {self.notes}" if self.notes else "")
        )


# ── Trajectory scoring ────────────────────────────────────────────────────────

def _normalise_range(joint_idx: int) -> float:
    """Full range of motion for a joint (used to normalise errors to [0,1])."""
    lo, hi = JOINT_LIMITS_DEG[joint_idx]
    return float(hi - lo) or 1.0


def _resample(positions: np.ndarray, n_target: int) -> np.ndarray:
    """
    Linearly resample a (T, NUM_JOINTS) trajectory to n_target time steps.
    """
    t_src = np.linspace(0.0, 1.0, len(positions))
    t_dst = np.linspace(0.0, 1.0, n_target)
    return np.stack(
        [np.interp(t_dst, t_src, positions[:, j]) for j in range(positions.shape[1])],
        axis=1,
    )


def score_trajectory(
    reference_waypoints: List[np.ndarray],
    actual_states: List[JointState],
    segment_duration_s: float,
) -> float:
    """
    Measure how closely the Prover's recorded joint stream matches the
    reference waypoints.

    Strategy: resample both to the same length, compute mean normalised RMSE
    across all joints, then convert to a 0-1 score with exponential decay.
    """
    if not actual_states or not reference_waypoints:
        return 0.0

    # Build reference dense trajectory (waypoints linearly interpolated)
    from pcs.arm_interface import interpolate_trajectory, HOME_POSE_DEG
    start = actual_states[0].positions
    ref_setpoints, _ = interpolate_trajectory(
        start,
        reference_waypoints,
        segment_duration_s=segment_duration_s,
        hz=len(actual_states),  # match recorded density for resampling
    )
    ref_arr = np.stack(ref_setpoints)  # (T_ref, NUM_JOINTS)
    act_arr = np.stack([s.positions for s in actual_states])  # (T_act, NUM_JOINTS)

    n = min(len(ref_arr), len(act_arr))
    if n < 2:
        return 0.0

    ref_r = _resample(ref_arr, n)
    act_r = _resample(act_arr, n)

    # Per-joint normalised error
    errors = []
    for j in range(NUM_JOINTS):
        rng = _normalise_range(j)
        rmse = float(np.sqrt(np.mean((ref_r[:, j] - act_r[:, j]) ** 2)))
        errors.append(rmse / rng)

    mean_norm_err = float(np.mean(errors))
    # Exponential mapping: err=0 → score=1, err=0.1 → ~0.37, err=0.3 → ~0.05
    score = float(np.exp(-10.0 * mean_norm_err))
    return float(np.clip(score, 0.0, 1.0))


# ── Timing scoring ────────────────────────────────────────────────────────────

def score_timing_micro_traj(
    challenge: MicroTrajectoryChallenge,
    actual_states: List[JointState],
    tolerance_s: float = 0.25,
) -> float:
    """
    For MicroTrajectory: check whether each waypoint was approximately reached
    within ±tolerance_s of its expected arrival time.
    """
    if not actual_states or not challenge.waypoints:
        return 0.0

    t0 = actual_states[0].timestamp
    n_hits = 0
    expected_times = [
        (i + 1) * challenge.segment_duration_s for i in range(len(challenge.waypoints))
    ]

    for wp, t_exp in zip(challenge.waypoints, expected_times):
        # Find the recorded state closest to expected arrival time
        times = np.array([s.timestamp - t0 for s in actual_states])
        closest_idx = int(np.argmin(np.abs(times - t_exp)))
        t_actual = times[closest_idx]

        if abs(t_actual - t_exp) <= tolerance_s:
            # Also check joint proximity to waypoint (within 5 deg RMSE)
            err = float(np.sqrt(np.mean(
                (actual_states[closest_idx].positions - wp) ** 2
            )))
            if err < 5.0:
                n_hits += 1

    return n_hits / len(challenge.waypoints)


def score_timing_tap_rhythm(
    challenge: TapRhythmChallenge,
    actual_states: List[JointState],
    tolerance_s: float = 0.15,
) -> float:
    """
    For TapRhythm: detect local minima in gripper position and check they
    align with expected beat times.
    """
    if not actual_states:
        return 0.0

    t0 = actual_states[0].timestamp
    times = np.array([s.timestamp - t0 for s in actual_states])
    gripper = np.array([s.positions[-1] for s in actual_states])  # last joint

    # Find local minima (taps = gripper closed = low value)
    tap_times: List[float] = []
    for i in range(1, len(gripper) - 1):
        if gripper[i] < gripper[i - 1] and gripper[i] < gripper[i + 1]:
            if gripper[i] < (challenge.open_deg + challenge.closed_deg) / 2:
                tap_times.append(float(times[i]))

    if not tap_times:
        return 0.0

    # Match detected taps to expected beats (greedy nearest-neighbour)
    matched = 0
    used = set()
    for t_exp in challenge.beat_times_s:
        for k, t_det in enumerate(tap_times):
            if k not in used and abs(t_det - t_exp) <= tolerance_s:
                matched += 1
                used.add(k)
                break

    return matched / len(challenge.beat_times_s)


def score_timing_pose(
    challenge: PoseAndPresentChallenge,
    actual_states: List[JointState],
    pos_tolerance_deg: float = 8.0,
) -> float:
    """
    For PoseAndPresent: what fraction of the hold window was the Prover
    within pos_tolerance_deg RMSE of the target pose?
    """
    if not actual_states:
        return 0.0

    t0 = actual_states[0].timestamp
    hold_start = challenge.move_duration_s
    hold_end = hold_start + challenge.hold_s

    hold_states = [
        s for s in actual_states
        if hold_start <= (s.timestamp - t0) <= hold_end
    ]
    if not hold_states:
        return 0.0

    in_pose = 0
    for s in hold_states:
        rmse = float(np.sqrt(np.mean((s.positions - challenge.target_pose) ** 2)))
        if rmse < pos_tolerance_deg:
            in_pose += 1

    return in_pose / len(hold_states)


def score_timing(challenge: Challenge, actual_states: List[JointState]) -> float:
    """Dispatch to the right timing scorer for the challenge type."""
    if isinstance(challenge, MicroTrajectoryChallenge):
        return score_timing_micro_traj(challenge, actual_states)
    if isinstance(challenge, TapRhythmChallenge):
        return score_timing_tap_rhythm(challenge, actual_states)
    if isinstance(challenge, PoseAndPresentChallenge):
        return score_timing_pose(challenge, actual_states)
    return 0.5  # Unknown type: neutral score




def _watermark_micromotion_score(frames: List[CameraFrame]) -> float:
    """
    Detect intentional servo-linked micro-movements from a red watermark marker.

    In mock mode, the camera synthesizer embeds a tiny red patch that oscillates
    with realistic phase drift. We track its centroid across frames and score
    whether displacement is non-zero and temporally varying.
    """
    if len(frames) < 4:
        return 0.0

    centers: List[Tuple[float, float]] = []
    for cf in frames:
        img = cf.image
        if img.ndim != 3 or img.shape[2] < 3:
            continue
        red = img[:, :, 2].astype(np.float32)
        blue = img[:, :, 0].astype(np.float32)
        green = img[:, :, 1].astype(np.float32)
        mask = (red > 180) & (blue < 80) & (green < 80)
        if not np.any(mask):
            continue
        ys, xs = np.where(mask)
        centers.append((float(xs.mean()), float(ys.mean())))

    if len(centers) < 4:
        return 0.0

    pts = np.array(centers, dtype=float)
    deltas = np.diff(pts, axis=0)
    speeds = np.linalg.norm(deltas, axis=1)
    if len(speeds) == 0:
        return 0.0

    move_strength = float(np.clip(speeds.mean() / 2.0, 0.0, 1.0))
    temporal_variation = float(np.clip(speeds.std() / 0.8, 0.0, 1.0))
    return float(0.65 * move_strength + 0.35 * temporal_variation)
# ── Visual scoring ────────────────────────────────────────────────────────────

def score_visual(
    frames_before: List[CameraFrame],
    frames_during: List[CameraFrame],
    frames_after: List[CameraFrame],
) -> float:
    """
    Estimate whether the Prover held an object throughout the challenge by
    comparing green-channel energy (proxy for the mock green blob) or
    general image variance.

    Returns 1.0 if object appears present, 0.0 if it appears absent/dropped.
    Gracefully handles missing frames (returns neutral 0.8).
    """
    all_frames = frames_before + frames_during + frames_after
    if not all_frames:
        return 0.8  # No camera → neutral benefit of the doubt

    # Compute per-frame object proxy: sum of green channel pixels above threshold
    scores: List[float] = []
    for cf in all_frames:
        img = cf.image.astype(np.float32)
        if img.ndim == 3 and img.shape[2] >= 3:
            # BGR: channel 1 = green
            green = img[:, :, 1]
            # Count bright green pixels (simple blob detector)
            mask = (green > 150) & (img[:, :, 0] < 100) & (img[:, :, 2] < 100)
            scores.append(float(mask.sum()))
        else:
            scores.append(float(img.mean()))

    if not scores:
        return 0.8

    arr = np.array(scores, dtype=float)
    if arr.max() < 1e-6:
        return 0.5  # No signal at all

    # Normalise
    arr_norm = arr / arr.max()

    # A good response: object present (high score) consistently
    mean_presence = float(arr_norm.mean())
    # Penalise variance (object shouldn't disappear and reappear)
    stability = 1.0 - float(arr_norm.std())
    stability = max(0.0, stability)

    visual_presence = 0.6 * mean_presence + 0.4 * stability

    # Physical watermark check: intentional micro-motions from verifier camera.
    watermark = _watermark_micromotion_score(all_frames)

    # Blend classic presence scoring with watermark dynamics.
    visual = 0.75 * visual_presence + 0.25 * watermark
    return float(np.clip(visual, 0.0, 1.0))


# ── Composite scoring ─────────────────────────────────────────────────────────

class VerificationEngine:
    """
    Aggregates sub-scores into a final TrustScore.

    Usage:
        engine = VerificationEngine()
        trust = engine.evaluate(challenge, actual_states, frames_before, frames_during)
    """

    def __init__(self, threshold: float = TRUST_THRESHOLD):
        self.threshold = threshold

    def evaluate(
        self,
        challenge: Challenge,
        actual_states: List[JointState],
        frames_before: Optional[List[CameraFrame]] = None,
        frames_during: Optional[List[CameraFrame]] = None,
        frames_after: Optional[List[CameraFrame]] = None,
    ) -> TrustScore:
        frames_before = frames_before or []
        frames_during = frames_during or []
        frames_after = frames_after or []

        # ── Trajectory score ──
        if isinstance(challenge, MicroTrajectoryChallenge):
            traj = score_trajectory(
                challenge.waypoints, actual_states, challenge.segment_duration_s
            )
        elif isinstance(challenge, PoseAndPresentChallenge):
            # Treat the single target pose as a 1-waypoint trajectory
            traj = score_trajectory(
                [challenge.target_pose], actual_states, challenge.move_duration_s
            )
        else:
            # TapRhythm: trajectory score is gripper position accuracy during taps
            traj = _score_tap_positions(challenge, actual_states)  # type: ignore[arg-type]

        # ── Timing score ──
        timing = score_timing(challenge, actual_states)

        # ── Visual score ──
        visual = score_visual(frames_before, frames_during, frames_after)

        composite = (
            W_TRAJECTORY * traj
            + W_TIMING * timing
            + W_VISUAL * visual
        )
        composite = float(np.clip(composite, 0.0, 1.0))
        granted = composite >= self.threshold

        ts = TrustScore(
            trajectory_score=round(traj, 4),
            timing_score=round(timing, 4),
            visual_score=round(visual, 4),
            composite=round(composite, 4),
            granted=granted,
        )
        log.info(ts.report())
        return ts


def _score_tap_positions(
    challenge: TapRhythmChallenge,
    actual_states: List[JointState],
) -> float:
    """
    For TapRhythm, the trajectory score measures how close the gripper gets
    to the target closed position during taps (min gripper value proximity).
    """
    if not actual_states:
        return 0.0
    gripper_vals = np.array([s.positions[-1] for s in actual_states])
    min_gripper = float(gripper_vals.min())
    target_closed = challenge.closed_deg
    target_open = challenge.open_deg
    rng = target_open - target_closed
    err = abs(min_gripper - target_closed) / (rng or 1.0)
    return float(np.clip(1.0 - err, 0.0, 1.0))
