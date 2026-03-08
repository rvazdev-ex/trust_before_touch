"""
protocol.py – 4-phase Physical Challenge–Response System coordinator.

Phases:
  1. CLAIM       – Prover broadcasts current end-effector pose + object state.
  2. CHALLENGE   – Leader generates a randomised motion task.
  3. RESPONSE    – Prover executes it; joint states + camera frames streamed.
  4. VERIFY      – Verifier scores the response and grants or denies handoff.

All 3 arms run their control loops concurrently (one thread each).
The protocol coordinator lives on the main thread and synchronises phases
via threading.Event gates.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Tuple

import numpy as np

from config import (
    CHALLENGE_TIMEOUT_S,
    CONTROL_HZ,
    INTERPHASE_PAUSE_S,
    TRUST_THRESHOLD,
    ArmConfig,
    PCSConfig,
)
from pcs.arm_interface import ArmInterface, CameraFrame, JointState, make_arm
from pcs.challenge_gen import (
    Challenge,
    MicroTrajectoryChallenge,
    PoseAndPresentChallenge,
    TapRhythmChallenge,
    describe_challenge,
    generate_challenge,
)
from pcs.verifier import TrustScore, VerificationEngine

log = logging.getLogger(__name__)


# ── Protocol state machine ────────────────────────────────────────────────────

class Phase(Enum):
    IDLE = auto()
    CLAIM = auto()
    CHALLENGE = auto()
    RESPONSE = auto()
    VERIFY = auto()
    DONE = auto()


@dataclass
class ClaimPacket:
    """What the Prover announces at the start of a round."""
    prover_pose: np.ndarray           # Joint positions (degrees)
    object_present: bool              # Is the Prover currently holding an object?
    timestamp: float = field(default_factory=time.monotonic)


@dataclass
class ResponsePacket:
    """Everything recorded during the Prover's challenge execution."""
    challenge_id: str
    joint_states: List[JointState]
    camera_frames: List[CameraFrame]
    start_time: float
    end_time: float

    @property
    def duration_s(self) -> float:
        return self.end_time - self.start_time


@dataclass
class ProtocolResult:
    """Full audit record of a single PCS round."""
    round_id: str
    claim: ClaimPacket
    challenge: Challenge
    response: ResponsePacket
    trust: TrustScore


# ── Per-arm background control loop ──────────────────────────────────────────

class ArmLoopThread(threading.Thread):
    """
    Runs a lightweight idle control loop for one arm.

    In IDLE state it just reads state at CONTROL_HZ and caches it.
    During RESPONSE it hands control to the protocol coordinator.

    Using a daemon thread ensures clean exit when the main thread ends.
    """

    def __init__(self, arm: ArmInterface, hz: int = CONTROL_HZ):
        super().__init__(daemon=True, name=f"arm-loop-{arm.role}")
        self.arm = arm
        self.hz = hz
        self._stop_event = threading.Event()
        self._latest_state: Optional[JointState] = None
        self._lock = threading.Lock()

    def run(self) -> None:
        dt = 1.0 / self.hz
        log.debug("[%s] Control loop started at %d Hz", self.arm.role, self.hz)
        while not self._stop_event.is_set():
            t0 = time.monotonic()
            try:
                state = self.arm.read_state()
                with self._lock:
                    self._latest_state = state
            except Exception as exc:
                log.warning("[%s] Read error: %s", self.arm.role, exc)
            elapsed = time.monotonic() - t0
            sleep = dt - elapsed
            if sleep > 0:
                time.sleep(sleep)

    def latest_state(self) -> Optional[JointState]:
        with self._lock:
            return self._latest_state

    def stop(self) -> None:
        self._stop_event.set()


# ── Prover response executor ──────────────────────────────────────────────────

def _execute_micro_trajectory(
    arm: ArmInterface,
    challenge: MicroTrajectoryChallenge,
    timeout_s: float,
) -> Tuple[List[JointState], List[CameraFrame]]:
    """Execute a MicroTrajectoryChallenge and return telemetry."""
    log.info("[prover] Executing micro-trajectory: %d waypoints", len(challenge.waypoints))
    states, frames = arm.execute_trajectory(
        waypoints=challenge.waypoints,
        segment_duration_s=challenge.segment_duration_s,
        record=True,
    )
    return states, frames


def _execute_tap_rhythm(
    arm: ArmInterface,
    challenge: TapRhythmChallenge,
    timeout_s: float,
) -> Tuple[List[JointState], List[CameraFrame]]:
    """
    Execute a TapRhythm by building a synthetic gripper trajectory.

    Between beats: gripper open (challenge.open_deg).
    At each beat: gripper slams to closed (challenge.closed_deg) for 80ms,
    then returns to open.
    """
    log.info(
        "[prover] Executing tap rhythm: %d beats @ %.0f BPM",
        challenge.n_beats, challenge.bpm,
    )
    dt = 1.0 / CONTROL_HZ
    t_end = challenge.expected_duration_s
    n_steps = int(round(t_end * CONTROL_HZ))
    beat_set = set(challenge.beat_times_s)

    # Build a waypoint list for the gripper (all other joints stay put)
    current = arm.read_state().positions.copy()

    recorded_states: List[JointState] = []
    recorded_frames: List[CameraFrame] = []
    tap_duration_s = 0.08  # seconds to hold "closed"

    t0 = time.monotonic()
    beat_iter = iter(challenge.beat_times_s)
    next_beat = next(beat_iter, None)
    tapping_until: Optional[float] = None

    for _ in range(n_steps):
        t_now = time.monotonic() - t0
        cmd = current.copy()

        if tapping_until is not None and t_now < tapping_until:
            cmd[-1] = challenge.closed_deg  # Stay closed during tap
        else:
            tapping_until = None
            cmd[-1] = challenge.open_deg    # Return to open

        # Trigger tap?
        if next_beat is not None and t_now >= next_beat:
            cmd[-1] = challenge.closed_deg
            tapping_until = t_now + tap_duration_s
            log.debug("[prover] Tap at t=%.3f (expected %.3f)", t_now, next_beat)
            next_beat = next(beat_iter, None)

        arm.command_joints(cmd)
        state = arm.read_state()
        recorded_states.append(state)
        if arm.config.has_camera:
            frame = arm.read_camera()
            if frame:
                recorded_frames.append(frame)

        elapsed = time.monotonic() - t0
        sleep = ((_ + 1) * dt) - elapsed
        if sleep > 0:
            time.sleep(sleep)

        if time.monotonic() - t0 > timeout_s:
            log.warning("[prover] Tap rhythm timed out")
            break

    return recorded_states, recorded_frames


def _execute_pose_and_present(
    arm: ArmInterface,
    challenge: PoseAndPresentChallenge,
    timeout_s: float,
) -> Tuple[List[JointState], List[CameraFrame]]:
    """Move to target pose, then hold it while streaming telemetry."""
    log.info("[prover] Executing pose-and-present: pose=%r", challenge.pose_name)

    # Phase A: move to target
    states_move, frames_move = arm.execute_trajectory(
        waypoints=[challenge.target_pose],
        segment_duration_s=challenge.move_duration_s,
        record=True,
    )

    # Phase B: hold pose
    hold_steps = int(round(challenge.hold_s * CONTROL_HZ))
    dt = 1.0 / CONTROL_HZ
    states_hold: List[JointState] = []
    frames_hold: List[CameraFrame] = []

    t0 = time.monotonic()
    for _ in range(hold_steps):
        arm.command_joints(challenge.target_pose)
        state = arm.read_state()
        states_hold.append(state)
        if arm.config.has_camera:
            frame = arm.read_camera()
            if frame:
                frames_hold.append(frame)
        elapsed = time.monotonic() - t0
        sleep = ((_ + 1) * dt) - elapsed
        if sleep > 0:
            time.sleep(sleep)

    return states_move + states_hold, frames_move + frames_hold


def execute_response(
    arm: ArmInterface,
    challenge: Challenge,
    timeout_s: float = CHALLENGE_TIMEOUT_S,
) -> ResponsePacket:
    """Dispatch challenge execution to the right executor and wrap in a packet."""
    t_start = time.monotonic()

    if isinstance(challenge, MicroTrajectoryChallenge):
        states, frames = _execute_micro_trajectory(arm, challenge, timeout_s)
    elif isinstance(challenge, TapRhythmChallenge):
        states, frames = _execute_tap_rhythm(arm, challenge, timeout_s)
    elif isinstance(challenge, PoseAndPresentChallenge):
        states, frames = _execute_pose_and_present(arm, challenge, timeout_s)
    else:
        raise ValueError(f"Unknown challenge type: {challenge.challenge_type}")

    return ResponsePacket(
        challenge_id=challenge.challenge_id,
        joint_states=states,
        camera_frames=frames,
        start_time=t_start,
        end_time=time.monotonic(),
    )


# ── Main PCS coordinator ──────────────────────────────────────────────────────

class PhysicalChallengeResponseSystem:
    """
    Orchestrates all 3 arms and runs the 4-phase PCS protocol.

    Typical usage:
        pcs = PhysicalChallengeResponseSystem(config)
        pcs.start()
        result = pcs.run_round()
        pcs.stop()
    """

    def __init__(self, pcs_config: PCSConfig):
        self.cfg = pcs_config
        self._verifier = VerificationEngine(threshold=pcs_config.trust_threshold)
        self._phase = Phase.IDLE
        self._history: List[ProtocolResult] = []

        # Instantiate arms
        self.leader: ArmInterface = make_arm(pcs_config.leader)
        self.prover: ArmInterface = make_arm(pcs_config.prover)
        self.verifier_arm: ArmInterface = make_arm(pcs_config.verifier)

        # Background control loops
        self._loops: List[ArmLoopThread] = []

    # -- Lifecycle ----------------------------------------------------------------

    def start(self) -> None:
        """Connect all arms and start background control loops."""
        log.info("=== PCS start: connecting arms ===")
        for arm in (self.leader, self.prover, self.verifier_arm):
            arm.connect()
            loop = ArmLoopThread(arm, hz=self.cfg.control_hz)
            loop.start()
            self._loops.append(loop)
            log.info("  %s loop started", arm.role)
        log.info("All arms connected and running.")

    def stop(self) -> None:
        """Stop loops and disconnect all arms."""
        log.info("=== PCS stop ===")
        for loop in self._loops:
            loop.stop()
        for loop in self._loops:
            loop.join(timeout=2.0)
        for arm in (self.leader, self.prover, self.verifier_arm):
            try:
                arm.go_home(duration_s=1.5)
                arm.disconnect()
            except Exception as exc:
                log.warning("Error during disconnect of %s: %s", arm.role, exc)
        self._loops.clear()
        log.info("All arms disconnected.")

    # -- Phase implementations ----------------------------------------------------

    def _phase_claim(self) -> ClaimPacket:
        """
        Phase 1 – CLAIM.
        Prover reads its current state and announces pose + object presence.
        Verifier reads a baseline camera frame for later comparison.
        """
        self._phase = Phase.CLAIM
        log.info("─── Phase 1: CLAIM ───")

        prover_state = self.prover.read_state()
        log.info("[claim] Prover pose: %s", dict(zip(
            ["sho_pan", "sho_lft", "elb", "wri_fl", "wri_rl", "grip"],
            [f"{v:.1f}" for v in prover_state.positions],
        )))

        # Infer object presence from gripper (open > 60 % → no object held)
        gripper_val = float(prover_state.positions[-1])
        object_present = gripper_val < 60.0
        log.info("[claim] Object present: %s (gripper=%.1f°)", object_present, gripper_val)

        time.sleep(INTERPHASE_PAUSE_S)
        return ClaimPacket(
            prover_pose=prover_state.positions.copy(),
            object_present=object_present,
        )

    def _phase_challenge(self, claim: ClaimPacket) -> Challenge:
        """
        Phase 2 – CHALLENGE.
        Leader inspects the claim and generates a randomised motion task.
        """
        self._phase = Phase.CHALLENGE
        log.info("─── Phase 2: CHALLENGE ───")

        challenge = generate_challenge(current_pose=claim.prover_pose)
        log.info("[challenge] %s", describe_challenge(challenge))

        time.sleep(INTERPHASE_PAUSE_S)
        return challenge

    def _phase_response(
        self, challenge: Challenge
    ) -> Tuple[ResponsePacket, List[CameraFrame]]:
        """
        Phase 3 – RESPONSE.
        Prover executes the challenge.
        Verifier simultaneously watches and records its wrist camera.
        """
        self._phase = Phase.RESPONSE
        log.info("─── Phase 3: RESPONSE ───")

        # Start verifier observation thread
        verifier_frames: List[CameraFrame] = []
        stop_verifier_obs = threading.Event()

        def _verifier_observe() -> None:
            while not stop_verifier_obs.is_set():
                if self.verifier_arm.config.has_camera:
                    try:
                        frame = self.verifier_arm.read_camera()
                        if frame:
                            verifier_frames.append(frame)
                    except Exception as exc:
                        log.debug("[verifier-obs] camera read error: %s", exc)
                time.sleep(1.0 / self.cfg.control_hz)

        obs_thread = threading.Thread(
            target=_verifier_observe, daemon=True, name="verifier-obs"
        )
        obs_thread.start()
        log.info("[response] Prover executing challenge '%s'…", challenge.challenge_id)

        response = execute_response(
            self.prover,
            challenge,
            timeout_s=self.cfg.challenge_timeout_s,
        )

        stop_verifier_obs.set()
        obs_thread.join(timeout=1.0)

        log.info(
            "[response] Done. %.2fs elapsed, %d joint snapshots, %d camera frames",
            response.duration_s,
            len(response.joint_states),
            len(response.camera_frames),
        )
        time.sleep(INTERPHASE_PAUSE_S)
        return response, verifier_frames

    def _phase_verify(
        self,
        challenge: Challenge,
        response: ResponsePacket,
        verifier_frames: List[CameraFrame],
    ) -> TrustScore:
        """
        Phase 4 – VERIFY.
        Verifier scores the response; Leader decides on handoff.
        """
        self._phase = Phase.VERIFY
        log.info("─── Phase 4: VERIFY ───")

        # Split verifier frames into before / during / after
        t_start = response.start_time
        t_end = response.end_time
        frames_before = [f for f in verifier_frames if f.timestamp < t_start]
        frames_during = [f for f in verifier_frames if t_start <= f.timestamp <= t_end]
        frames_after = [f for f in verifier_frames if f.timestamp > t_end]

        trust = self._verifier.evaluate(
            challenge=challenge,
            actual_states=response.joint_states,
            frames_before=frames_before,
            frames_during=frames_during,
            frames_after=frames_after,
        )

        if trust.granted:
            log.info("[verify] ✓ HANDOFF GRANTED  (score=%.3f ≥ %.2f)",
                     trust.composite, self.cfg.trust_threshold)
        else:
            log.info("[verify] ✗ HANDOFF DENIED   (score=%.3f < %.2f)",
                     trust.composite, self.cfg.trust_threshold)

        return trust

    # -- Top-level round ----------------------------------------------------------

    def run_round(self, round_id: Optional[str] = None) -> ProtocolResult:
        """
        Execute a single PCS round (all 4 phases) and return the full result.
        Blocks until verification is complete.
        """
        if round_id is None:
            round_id = f"round-{int(time.time()) % 100000:05d}"

        log.info("══════════════════════════════════════")
        log.info(" PCS ROUND %s", round_id)
        log.info("══════════════════════════════════════")

        claim = self._phase_claim()
        challenge = self._phase_challenge(claim)
        response, verifier_frames = self._phase_response(challenge)
        trust = self._phase_verify(challenge, response, verifier_frames)

        result = ProtocolResult(
            round_id=round_id,
            claim=claim,
            challenge=challenge,
            response=response,
            trust=trust,
        )
        self._history.append(result)
        self._phase = Phase.DONE
        return result

    # -- Convenience --------------------------------------------------------------

    @property
    def history(self) -> List[ProtocolResult]:
        return list(self._history)

    def current_phase(self) -> Phase:
        return self._phase

    def arm_states(self) -> dict:
        """Snapshot of all three arms' latest joint states (from their loops)."""
        out = {}
        for loop in self._loops:
            state = loop.latest_state()
            out[loop.arm.role] = state
        return out
