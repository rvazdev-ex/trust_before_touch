"""
arm_interface.py – Per-arm control wrapper around LeRobot SO-101.

Two concrete implementations:
  - LeRobotArmInterface: talks to real hardware via lerobot
  - MockArmInterface: pure-Python simulation for testing without hardware

Both expose the same API so protocol.py never needs to know which is running.
"""
from __future__ import annotations

import abc
import logging
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from config import (
    CONTROL_HZ,
    HOME_POSE_DEG,
    JOINT_LIMITS_DEG,
    JOINT_NAMES,
    NUM_JOINTS,
    ArmConfig,
)

log = logging.getLogger(__name__)


PCS_TO_LEROBOT = {
    "sho_pan": "shoulder_pan",
    "sho_lft": "shoulder_lift",
    "elb": "elbow_flex",
    "wri_fl": "wrist_flex",
    "wri_rl": "wrist_roll",
    "grip": "gripper",
}


# ── Data structures ───────────────────────────────────────────────────────────

@dataclass
class JointState:
    """Snapshot of all joint positions at a single point in time."""
    positions: np.ndarray   # shape (NUM_JOINTS,), degrees
    timestamp: float        # time.monotonic()

    def as_dict(self) -> Dict[str, float]:
        return dict(zip(JOINT_NAMES, self.positions.tolist()))


@dataclass
class CameraFrame:
    """Single RGB frame from a wrist or overhead camera."""
    image: np.ndarray   # uint8, shape (H, W, 3), BGR
    timestamp: float


# ── Joint helpers ─────────────────────────────────────────────────────────────

def clip_to_limits(positions: np.ndarray) -> np.ndarray:
    """Clamp joint values to their safe operating range."""
    out = positions.copy()
    for i, (lo, hi) in enumerate(JOINT_LIMITS_DEG):
        out[i] = np.clip(out[i], lo, hi)
    return out


def interpolate_trajectory(
    start: np.ndarray,
    waypoints: List[np.ndarray],
    segment_duration_s: float,
    hz: int,
) -> Tuple[List[np.ndarray], List[float]]:
    """
    Build a dense list of joint-position setpoints by linearly interpolating
    through ``waypoints`` starting at ``start``.

    Returns (setpoints, relative_timestamps_s).
    """
    setpoints: List[np.ndarray] = []
    timestamps: List[float] = []
    dt = 1.0 / hz
    t = 0.0

    all_pts = [start] + waypoints
    for seg_idx in range(len(all_pts) - 1):
        p0 = all_pts[seg_idx]
        p1 = all_pts[seg_idx + 1]
        n_steps = max(1, int(round(segment_duration_s * hz)))
        for step in range(n_steps):
            alpha = step / n_steps
            setpoints.append(clip_to_limits(p0 + alpha * (p1 - p0)))
            timestamps.append(t)
            t += dt

    # Append final waypoint
    setpoints.append(clip_to_limits(all_pts[-1]))
    timestamps.append(t)
    return setpoints, timestamps


# ── Abstract base ─────────────────────────────────────────────────────────────

class ArmInterface(abc.ABC):
    """Common interface for both real and mock arms."""

    def __init__(self, config: ArmConfig):
        self.config = config
        self.role = config.role
        self._connected = False

    # -- Lifecycle --

    @abc.abstractmethod
    def connect(self) -> None: ...

    @abc.abstractmethod
    def disconnect(self) -> None: ...

    # -- Sensing --

    @abc.abstractmethod
    def read_state(self) -> JointState:
        """Return current joint positions."""
        ...

    @abc.abstractmethod
    def read_camera(self) -> Optional[CameraFrame]:
        """Return latest camera frame, or None if no camera configured."""
        ...

    # -- Actuation --

    @abc.abstractmethod
    def command_joints(self, target: np.ndarray) -> None:
        """Send a single joint-position setpoint (non-blocking)."""
        ...

    # -- Compound motion: blocking trajectory execution --

    def execute_trajectory(
        self,
        waypoints: List[np.ndarray],
        segment_duration_s: float,
        record: bool = True,
    ) -> Tuple[List[JointState], List[CameraFrame]]:
        """
        Execute a multi-waypoint trajectory, optionally recording telemetry.

        Returns (recorded_joint_states, recorded_frames).
        """
        current = self.read_state().positions
        setpoints, rel_ts = interpolate_trajectory(
            current, waypoints, segment_duration_s, hz=CONTROL_HZ
        )

        dt = 1.0 / CONTROL_HZ
        recorded_joints: List[JointState] = []
        recorded_frames: List[CameraFrame] = []

        for sp in setpoints:
            t_start = time.monotonic()
            self.command_joints(sp)

            if record:
                state = self.read_state()
                recorded_joints.append(state)
                if self.config.has_camera:
                    frame = self.read_camera()
                    if frame is not None:
                        recorded_frames.append(frame)

            elapsed = time.monotonic() - t_start
            sleep_s = dt - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

        return recorded_joints, recorded_frames

    def move_to(self, target: np.ndarray, duration_s: float = 2.0) -> None:
        """Blocking move to a single target pose."""
        self.execute_trajectory([target], segment_duration_s=duration_s, record=False)

    def go_home(self, duration_s: float = 2.0) -> None:
        self.move_to(np.array(HOME_POSE_DEG, dtype=float), duration_s)

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(role={self.role!r}, port={self.config.port!r})"


# ── Real hardware implementation ──────────────────────────────────────────────

class LeRobotArmInterface(ArmInterface):
    """Controls a physical SO-101 arm via the LeRobot leader/follower APIs."""

    def __init__(self, config: ArmConfig):
        super().__init__(config)
        self._robot = None
        self._camera = None   # OpenCV VideoCapture for wrist cam

    def _calibration_help(self) -> str:
        if self.role == "leader":
            return (
                "python -m lerobot.calibrate "
                f"--teleop.type=so101_leader --teleop.port={self.config.port} --teleop.id={self.config.id}"
            )
        return (
            "python -m lerobot.calibrate "
            f"--robot.type=so101_follower --robot.port={self.config.port} --robot.id={self.config.id}"
        )

    def connect(self) -> None:
        config = self.config
        try:
            if config.role == "leader":
                from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

                robot_cfg = SO101LeaderConfig(port=config.port, id=config.id)
                self._robot = SO101Leader(robot_cfg)
            else:
                from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

                robot_cfg = SO101FollowerConfig(port=config.port, id=config.id)
                self._robot = SO101Follower(robot_cfg)

            self._robot.connect(calibrate=False)
            log.info("[%s] Connected to %s (id=%s)", self.role, config.port, config.id)

            if self.config.has_camera:
                import cv2

                self._camera = cv2.VideoCapture(self.config.camera_index)
                if not self._camera.isOpened():
                    log.warning("[%s] Could not open camera index %d", self.role, self.config.camera_index)
                    self._camera = None

            self._connected = True
        except Exception:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        robot = self._robot
        camera = self._camera
        self._robot = None
        self._camera = None
        self._connected = False

        if robot is not None:
            try:
                robot.disconnect()
            except Exception as exc:
                log.warning("[%s] Robot disconnect error: %s", self.role, exc)

        if camera is not None:
            try:
                camera.release()
            except Exception as exc:
                log.warning("[%s] Camera release error: %s", self.role, exc)

        log.info("[%s] Disconnected", self.role)

    def _extract_joint_state(self, payload: dict) -> Dict[str, float]:
        if not isinstance(payload, dict):
            raise RuntimeError(f"[{self.role}] Expected dict payload, got {type(payload).__name__}")

        joint_state = payload.get("joint_state")
        if isinstance(joint_state, dict):
            return {str(k): float(v) for k, v in joint_state.items()}

        flat_state = {
            k.removesuffix(".pos"): float(v)
            for k, v in payload.items()
            if isinstance(k, str) and k.endswith(".pos")
        }
        if flat_state:
            return flat_state

        expected = {
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        }
        bare_state = {k: float(v) for k, v in payload.items() if k in expected}
        if bare_state:
            return bare_state

        raise RuntimeError(
            f"[{self.role}] No joint state found in LeRobot payload. Keys={list(payload.keys())}"
        )

    def read_state(self) -> JointState:
        if self._robot is None:
            raise RuntimeError(f"[{self.role}] Arm is not connected")

        try:
            if self.role == "leader":
                payload = self._robot.get_action()
            else:
                payload = self._robot.get_observation()
        except Exception as exc:
            msg = str(exc).lower()
            if self.role in {"prover", "verifier"} and "calibration" in msg and "registered" in msg:
                cmd = self._calibration_help()
                raise RuntimeError(
                    f"[{self.role}] Missing LeRobot calibration for id='{self.config.id}' on port '{self.config.port}'. "
                    f"Run: {cmd}"
                ) from exc
            raise

        if isinstance(payload, dict):
            log.debug("[%s] Raw LeRobot payload keys: %s", self.role, list(payload.keys()))
        else:
            log.debug("[%s] Raw LeRobot payload type: %s", self.role, type(payload).__name__)

        joint_state = self._extract_joint_state(payload)
        positions = np.array([joint_state[name] for name in JOINT_NAMES], dtype=float)
        return JointState(positions=positions, timestamp=time.monotonic())

    def read_camera(self) -> Optional[CameraFrame]:
        if self._camera is None:
            return None
        import cv2
        ret, frame = self._camera.read()
        if not ret:
            return None
        return CameraFrame(image=frame, timestamp=time.monotonic())

    def command_joints(self, target: np.ndarray) -> None:
        if self._robot is None:
            raise RuntimeError(f"[{self.role}] Arm is not connected")

        # Leader is read-only in this wrapper; writes are follower-only.
        if self.role == "leader":
            log.debug("[%s] Ignoring command_joints write on leader", self.role)
            return

        action = {
            f"{PCS_TO_LEROBOT[joint]}.pos": float(value)
            for joint, value in zip(JOINT_NAMES, target.tolist())
            if joint in PCS_TO_LEROBOT
        }
        if not action:
            raise RuntimeError(
                f"[{self.role}] Empty follower action payload after PCS→LeRobot mapping. "
                f"Input joints={list(JOINT_NAMES)}"
            )

        log.debug("[%s] Sending follower action payload: %s", self.role, action)
        self._robot.send_action(action)


# ── Mock / simulation implementation ─────────────────────────────────────────

class MockArmInterface(ArmInterface):
    """
    Software-only arm that simulates first-order joint dynamics.

    Useful for developing and testing PCS logic without physical hardware.
    The internal state drifts toward the commanded setpoint with a configurable
    time constant, mimicking real servo behaviour.
    """

    # Servo time-constant: how quickly the simulated joint tracks its target.
    # At CONTROL_HZ = 50 Hz with TC = 0.15 s, alpha ≈ 0.13 per step.
    TIME_CONSTANT_S: float = 0.15
    WATERMARK_FREQ_HZ: float = 4.0
    WATERMARK_PIXELS: int = 4

    def __init__(self, config: ArmConfig, initial_pose: Optional[np.ndarray] = None):
        super().__init__(config)
        self._q = np.array(
            initial_pose if initial_pose is not None else HOME_POSE_DEG, dtype=float
        )
        self._q_target = self._q.copy()
        self._alpha = 1.0 - np.exp(-1.0 / (self.TIME_CONSTANT_S * CONTROL_HZ))
        self._watermark_phase = 0.0

    def connect(self) -> None:
        self._connected = True
        log.info("[%s] Mock arm connected (simulation mode)", self.role)

    def disconnect(self) -> None:
        self._connected = False
        log.info("[%s] Mock arm disconnected", self.role)

    def _step_dynamics(self) -> None:
        """Update internal state toward target (call after each command)."""
        self._q += self._alpha * (self._q_target - self._q)
        # Add tiny noise to simulate encoder quantisation
        self._q += np.random.normal(0, 0.05, size=NUM_JOINTS)
        self._q = clip_to_limits(self._q)
        self._watermark_phase += 2.0 * np.pi * self.WATERMARK_FREQ_HZ / CONTROL_HZ

    def read_state(self) -> JointState:
        self._step_dynamics()
        return JointState(positions=self._q.copy(), timestamp=time.monotonic())

    def read_camera(self) -> Optional[CameraFrame]:
        if not self.config.has_camera:
            return None
        # Synthetic 120×160 BGR frame: grey background + coloured object blob.
        # Drawn with pure numpy (no cv2 dependency) so mock mode needs no extra installs.
        frame = np.full((120, 160, 3), 128, dtype=np.uint8)
        cy, cx, r = 60, 80, 18
        ys, xs = np.ogrid[:120, :160]
        mask = (xs - cx) ** 2 + (ys - cy) ** 2 <= r ** 2
        frame[mask] = [30, 200, 60]  # green blob (BGR)

        # Add physical watermark demo: tiny red marker that intentionally
        # micro-moves with a sinusoid + servo-load perturbation.
        # The verifier uses this to detect that observed motion has a
        # physically plausible temporal signature, not a static replay.
        servo_error = float(np.mean(np.abs(self._q_target - self._q)))
        amp_px = 2.0 + min(2.0, servo_error / 15.0)
        wx = 130 + int(round(amp_px * np.sin(self._watermark_phase)))
        wy = 25 + int(round(amp_px * np.cos(self._watermark_phase)))

        wy0 = max(0, wy - self.WATERMARK_PIXELS)
        wy1 = min(frame.shape[0], wy + self.WATERMARK_PIXELS + 1)
        wx0 = max(0, wx - self.WATERMARK_PIXELS)
        wx1 = min(frame.shape[1], wx + self.WATERMARK_PIXELS + 1)
        frame[wy0:wy1, wx0:wx1] = [20, 20, 240]  # red watermark patch (BGR)
        return CameraFrame(image=frame, timestamp=time.monotonic())

    def command_joints(self, target: np.ndarray) -> None:
        self._q_target = clip_to_limits(target.copy())


# ── Factory ───────────────────────────────────────────────────────────────────

def make_arm(config: ArmConfig) -> ArmInterface:
    """Instantiate the right arm implementation from an ArmConfig."""
    if config.mock:
        return MockArmInterface(config)
    return LeRobotArmInterface(config)
