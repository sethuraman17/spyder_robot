"""
MuJoCo simulation of an 8-servo quadruped robot.

High-level motion logic uses DEGREES (same as the ESP32 / Servo.write API).
Conversion to radians happens only when writing actuator targets to MuJoCo.

Keyboard (arrow keys preferred):
  UP    = STAND
  DOWN  = SIT
  LEFT  = MOVE LEFT LEG  (requires STANDING)
  RIGHT = MOVE RIGHT LEG (requires STANDING)
  R     = RESET to sitting

Letter aliases still work: S / D / L
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Dict, List, Optional

import mujoco
import mujoco.viewer


# =============================================================================
# Paths / model
# =============================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, "quadruped.xml")


# =============================================================================
# Real-robot joint map (GPIO numbers are identifiers only — no hardware access)
# =============================================================================

JOINT_NAMES = [
    "leftup1",
    "leftup2",
    "rightup1",
    "rightup2",
    "leftdown1",
    "leftdown2",
    "rightdown1",
    "rightdown2",
]

# Logical ESP32 mapping used by the real robot
GPIO_MAP = {
    "leftup1": 14,      # FRONT LEFT  joint 1
    "leftup2": 16,      # FRONT LEFT  joint 2
    "rightup1": 12,     # FRONT RIGHT joint 1
    "rightup2": 26,     # FRONT RIGHT joint 2
    "leftdown1": 17,    # REAR LEFT   joint 1
    "leftdown2": 4,     # REAR LEFT   joint 2
    "rightdown1": 13,   # REAR RIGHT  joint 1
    "rightdown2": 2,    # REAR RIGHT  joint 2
}

HIP_JOINTS = ["leftup1", "rightup1", "leftdown1", "rightdown1"]
KNEE_JOINTS = ["leftup2", "rightup2", "leftdown2", "rightdown2"]

# Pose values in DEGREES (match real servo commands)
HIP_DEFAULT_DEG = 145.0
KNEE_SIT_DEG = 0.0
KNEE_STAND_DEG = 90.0

SIT_POSE_DEG = {
    "leftup1": HIP_DEFAULT_DEG,
    "rightup1": HIP_DEFAULT_DEG,
    "leftdown1": HIP_DEFAULT_DEG,
    "rightdown1": HIP_DEFAULT_DEG,
    "leftup2": KNEE_SIT_DEG,
    "rightup2": KNEE_SIT_DEG,
    "leftdown2": KNEE_SIT_DEG,
    "rightdown2": KNEE_SIT_DEG,
}

STAND_POSE_DEG = {
    "leftup1": HIP_DEFAULT_DEG,
    "rightup1": HIP_DEFAULT_DEG,
    "leftdown1": HIP_DEFAULT_DEG,
    "rightdown1": HIP_DEFAULT_DEG,
    "leftup2": KNEE_STAND_DEG,
    "rightup2": KNEE_STAND_DEG,
    "leftdown2": KNEE_STAND_DEG,
    "rightdown2": KNEE_STAND_DEG,
}

# Motion durations (seconds) — tune after first test
STAND_DURATION = 2.0
SIT_DURATION = 2.0
LEG_STEP_DURATION = 1.2

# GLFW keycodes used by mujoco.viewer key_callback
KEY_LEFT = 263
KEY_RIGHT = 262
KEY_UP = 265
KEY_DOWN = 266


# =============================================================================
# Angle helpers (degrees <-> radians)
# =============================================================================

def deg_to_rad(degrees: float) -> float:
    """Convert degrees to radians. Does NOT wrap/normalize angles."""
    return degrees * math.pi / 180.0


def rad_to_deg(radians: float) -> float:
    """Convert radians to degrees. Does NOT wrap/normalize angles."""
    return radians * 180.0 / math.pi


def smoothstep(t: float) -> float:
    """Hermite smoothstep easing in [0, 1]."""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


# =============================================================================
# Robot state
# =============================================================================

class RobotState(Enum):
    SITTING = auto()
    STANDING = auto()
    STANDING_UP = auto()
    SITTING_DOWN = auto()
    MOVING_LEFT_LEG = auto()
    MOVING_RIGHT_LEG = auto()


STATE_LABELS = {
    RobotState.SITTING: "SITTING",
    RobotState.STANDING: "STANDING",
    RobotState.STANDING_UP: "STANDING UP",
    RobotState.SITTING_DOWN: "SITTING DOWN",
    RobotState.MOVING_LEFT_LEG: "MOVING LEFT LEG",
    RobotState.MOVING_RIGHT_LEG: "MOVING RIGHT LEG",
}


# =============================================================================
# Non-blocking motion primitives
# =============================================================================

@dataclass
class JointMotion:
    """Interpolate one joint target from start_deg to end_deg over duration."""
    joint_name: str
    start_deg: float
    end_deg: float
    duration: float
    t0: float


@dataclass
class MotionStep:
    """One step of a sequence: several joints moving together, then a callback."""
    motions: List[JointMotion] = field(default_factory=list)
    on_complete: Optional[Callable[[], None]] = None


class MotionEngine:
    """Time-based interpolator that never blocks the MuJoCo render loop."""

    def __init__(self) -> None:
        self._queue: List[MotionStep] = []
        self._active: Optional[MotionStep] = None
        self._busy = False
        self._on_sequence_done: Optional[Callable[[], None]] = None

    @property
    def busy(self) -> bool:
        return self._busy

    def clear(self) -> None:
        self._queue.clear()
        self._active = None
        self._busy = False
        self._on_sequence_done = None

    def start_sequence(
        self,
        steps: List[MotionStep],
        on_done: Optional[Callable[[], None]] = None,
        targets: Optional[Dict[str, float]] = None,
    ) -> None:
        self.clear()
        self._queue = list(steps)
        self._on_sequence_done = on_done
        self._busy = True
        self._start_next(time.time(), targets)

    def _start_next(
        self,
        now: float,
        targets: Optional[Dict[str, float]] = None,
    ) -> None:
        if not self._queue:
            self._active = None
            self._busy = False
            cb = self._on_sequence_done
            self._on_sequence_done = None
            if cb is not None:
                cb()
            return

        step = self._queue.pop(0)
        for motion in step.motions:
            # Begin each step from the live target (no teleport / no angle wrap)
            if targets is not None:
                motion.start_deg = targets[motion.joint_name]
            motion.t0 = now
        self._active = step

    def update(self, now: float, targets: Dict[str, float]) -> None:
        """Advance active motions and write interpolated degree targets."""
        if self._active is None:
            return

        step = self._active
        all_done = True

        for motion in step.motions:
            if motion.duration <= 0.0:
                targets[motion.joint_name] = motion.end_deg
                continue

            elapsed = now - motion.t0
            t = elapsed / motion.duration
            if t >= 1.0:
                targets[motion.joint_name] = motion.end_deg
            else:
                all_done = False
                alpha = smoothstep(t)
                # Linear blend in DEGREE space — 145 -> 360 is NOT wrapped to 0
                targets[motion.joint_name] = (
                    motion.start_deg + (motion.end_deg - motion.start_deg) * alpha
                )

        if all_done:
            if step.on_complete is not None:
                step.on_complete()
            self._start_next(now, targets)


# =============================================================================
# Command / control layer (keep this logic shared with future ESP32 output)
# =============================================================================

class RobotController:
    """
    High-level robot commands in DEGREES.

    Later, replace apply_to_mujoco() with Wi-Fi / ESP32 output without rewriting
    stand_robot / sit_robot / move_left_leg.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model = model
        self.data = data
        self.state = RobotState.SITTING
        self.targets: Dict[str, float] = dict(SIT_POSE_DEG)
        self.motion = MotionEngine()
        self._last_status_print = 0.0
        self._status_interval = 2.0

        # Cache actuator / joint addresses
        self.act_ids = {
            name: model.actuator(f"act_{name}").id for name in JOINT_NAMES
        }
        self.joint_qpos_addrs = {
            name: model.joint(name).qposadr[0] for name in JOINT_NAMES
        }

    # ----- status -----

    def print_status(self, force: bool = False) -> None:
        now = time.time()
        if not force and (now - self._last_status_print) < self._status_interval:
            return
        self._last_status_print = now

        print()
        print(f"Robot state: {STATE_LABELS[self.state]}")
        print()
        for name in JOINT_NAMES:
            gpio = GPIO_MAP[name]
            print(f"{name:<12} target: {self.targets[name]:6.1f} deg   (GPIO {gpio})")
        print()

    def set_state(self, new_state: RobotState) -> None:
        self.state = new_state
        print(f"STATE: {STATE_LABELS[new_state]}")
        self.print_status(force=True)

    # ----- smooth motion helpers -----

    def move_joint_smooth(
        self,
        joint_name: str,
        start_angle: float,
        target_angle: float,
        duration: float,
        now: Optional[float] = None,
    ) -> MotionStep:
        """Build a single-joint smooth motion step (degrees)."""
        if now is None:
            now = time.time()
        return MotionStep(
            motions=[
                JointMotion(
                    joint_name=joint_name,
                    start_deg=start_angle,
                    end_deg=target_angle,
                    duration=duration,
                    t0=now,
                )
            ]
        )

    def move_group_smooth(
        self,
        joint_names: List[str],
        target_angles: List[float],
        duration: float,
        now: Optional[float] = None,
    ) -> MotionStep:
        """Build a multi-joint smooth motion step (degrees)."""
        if now is None:
            now = time.time()
        if len(joint_names) != len(target_angles):
            raise ValueError("joint_names and target_angles length mismatch")

        motions = []
        for name, end_deg in zip(joint_names, target_angles):
            motions.append(
                JointMotion(
                    joint_name=name,
                    start_deg=self.targets[name],
                    end_deg=end_deg,
                    duration=duration,
                    t0=now,
                )
            )
        return MotionStep(motions=motions)

    # ----- high-level commands (ESP32-equivalent API) -----

    def stand_robot(self) -> None:
        if self.motion.busy:
            print("Ignored STAND: another movement is running.")
            return
        if self.state == RobotState.STANDING:
            print("Ignored STAND: already standing.")
            return
        if self.state == RobotState.STANDING_UP:
            print("Ignored STAND: already standing up.")
            return

        self.set_state(RobotState.STANDING_UP)
        now = time.time()
        step = self.move_group_smooth(
            KNEE_JOINTS,
            [KNEE_STAND_DEG] * 4,
            STAND_DURATION,
            now=now,
        )
        self.motion.start_sequence(
            [step],
            on_done=lambda: self.set_state(RobotState.STANDING),
            targets=self.targets,
        )

    def sit_robot(self) -> None:
        if self.motion.busy:
            print("Ignored SIT: another movement is running.")
            return
        if self.state == RobotState.SITTING:
            print("Ignored SIT: already sitting.")
            return
        if self.state == RobotState.SITTING_DOWN:
            print("Ignored SIT: already sitting down.")
            return

        self.set_state(RobotState.SITTING_DOWN)
        now = time.time()
        step = self.move_group_smooth(
            KNEE_JOINTS,
            [KNEE_SIT_DEG] * 4,
            SIT_DURATION,
            now=now,
        )
        self.motion.start_sequence(
            [step],
            on_done=lambda: self.set_state(RobotState.SITTING),
            targets=self.targets,
        )

    def _move_rear_leg(
        self,
        hip_joint: str,
        knee_joint: str,
        moving_state: RobotState,
        label: str,
    ) -> None:
        """
        Shared rear-leg sequence (same as real ESP32 left-leg motion):

          start: hip=145, knee=90  (must already be standing)
          1) knee:  90 -> 125
          2) hip:  145 -> 360   (360 is kept, NOT normalized to 0)
          3) knee: 125 -> 90
          4) hip:  360 -> 145
          5) stop, remain standing
        """
        if self.motion.busy:
            print(f"Ignored {label}: another movement is running.")
            return
        if self.state != RobotState.STANDING:
            print(f"Ignored {label}: robot must be STANDING first.")
            return

        self.targets[hip_joint] = HIP_DEFAULT_DEG
        self.targets[knee_joint] = KNEE_STAND_DEG

        self.set_state(moving_state)
        now = time.time()
        dur = LEG_STEP_DURATION

        def step_msg(msg: str) -> Callable[[], None]:
            return lambda: print(f"  ... {msg}")

        steps = [
            self.move_joint_smooth(knee_joint, 90.0, 125.0, dur, now),
            self.move_joint_smooth(hip_joint, 145.0, 360.0, dur, now),
            self.move_joint_smooth(knee_joint, 125.0, 90.0, dur, now),
            self.move_joint_smooth(hip_joint, 360.0, 145.0, dur, now),
        ]
        steps[0].on_complete = step_msg(f"STEP 1 done: {knee_joint} -> 125")
        steps[1].on_complete = step_msg(f"STEP 2 done: {hip_joint} -> 360")
        steps[2].on_complete = step_msg(f"STEP 3 done: {knee_joint} -> 90")
        steps[3].on_complete = step_msg(f"STEP 4 done: {hip_joint} -> 145")

        def finished() -> None:
            self.targets[hip_joint] = HIP_DEFAULT_DEG
            self.targets[knee_joint] = KNEE_STAND_DEG
            print("  ... STEP 5: stop")
            self.set_state(RobotState.STANDING)

        self.motion.start_sequence(steps, on_done=finished, targets=self.targets)

    def move_left_leg(self) -> None:
        """Rear-left leg sequence (leftdown1 / leftdown2)."""
        self._move_rear_leg(
            hip_joint="leftdown1",
            knee_joint="leftdown2",
            moving_state=RobotState.MOVING_LEFT_LEG,
            label="MOVE LEFT LEG",
        )

    def move_right_leg(self) -> None:
        """Rear-right leg sequence (rightdown1 / rightdown2) — mirror of left."""
        self._move_rear_leg(
            hip_joint="rightdown1",
            knee_joint="rightdown2",
            moving_state=RobotState.MOVING_RIGHT_LEG,
            label="MOVE RIGHT LEG",
        )

    def reset_robot(self) -> None:
        """Snap back to sitting pose and reset physics state."""
        self.motion.clear()
        self.targets = dict(SIT_POSE_DEG)

        # Restore keyframe "sitting"
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "sitting")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        else:
            mujoco.mj_resetData(self.model, self.data)

        self.apply_to_mujoco()
        mujoco.mj_forward(self.model, self.data)
        self.set_state(RobotState.SITTING)
        print("Robot reset to SITTING.")

    # ----- MuJoCo backend (swap this later for ESP32 output) -----

    def apply_to_mujoco(self) -> None:
        """Write degree targets to MuJoCo position actuators as radians."""
        for name in JOINT_NAMES:
            # IMPORTANT: no wrap/normalize — 360 deg stays 360 deg (2*pi rad)
            self.data.ctrl[self.act_ids[name]] = deg_to_rad(self.targets[name])

    def update(self, now: float) -> None:
        self.motion.update(now, self.targets)
        self.apply_to_mujoco()
        self.print_status(force=False)


# =============================================================================
# Keyboard handling
# =============================================================================

def make_key_callback(controller: RobotController):
    def key_callback(keycode: int) -> None:
        # Arrow keys (GLFW codes from mujoco.viewer)
        if keycode == KEY_UP:
            print("COMMAND: STAND (UP)")
            controller.stand_robot()
            return
        if keycode == KEY_DOWN:
            print("COMMAND: SIT (DOWN)")
            controller.sit_robot()
            return
        if keycode == KEY_LEFT:
            print("COMMAND: MOVE LEFT LEG (LEFT)")
            controller.move_left_leg()
            return
        if keycode == KEY_RIGHT:
            print("COMMAND: MOVE RIGHT LEG (RIGHT)")
            controller.move_right_leg()
            return

        try:
            key = chr(keycode).lower()
        except ValueError:
            return

        # Letter aliases
        if key == "s":
            print("COMMAND: STAND")
            controller.stand_robot()
        elif key == "d":
            print("COMMAND: SIT")
            controller.sit_robot()
        elif key == "l":
            print("COMMAND: MOVE LEFT LEG")
            controller.move_left_leg()
        elif key == "r":
            print("COMMAND: RESET")
            controller.reset_robot()

    return key_callback


# =============================================================================
# Main
# =============================================================================

def setup_camera(viewer: mujoco.viewer.Handle) -> None:
    """Useful 3/4 view of the whole quadruped."""
    viewer.cam.lookat[:] = [0.0, 0.0, 0.15]
    viewer.cam.distance = 1.6
    viewer.cam.azimuth = -45.0
    viewer.cam.elevation = -20.0


def main() -> None:
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"Model not found: {MODEL_PATH}")

    print("Loading MuJoCo model:", MODEL_PATH)
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    controller = RobotController(model, data)
    controller.reset_robot()

    print()
    print("=" * 54)
    print("  QUADRUPED MUJOCO SIMULATION")
    print("=" * 54)
    print("  UP ARROW    = STAND")
    print("  DOWN ARROW  = SIT")
    print("  LEFT ARROW  = MOVE LEFT LEG  (must be standing)")
    print("  RIGHT ARROW = MOVE RIGHT LEG (must be standing)")
    print("  R           = RESET")
    print("=" * 54)
    print()

    key_cb = make_key_callback(controller)

    with mujoco.viewer.launch_passive(model, data, key_callback=key_cb) as viewer:
        setup_camera(viewer)

        while viewer.is_running():
            step_start = time.time()

            controller.update(step_start)
            mujoco.mj_step(model, data)
            viewer.sync()

            # Keep simulation roughly realtime
            elapsed = time.time() - step_start
            sleep_time = model.opt.timestep - elapsed
            if sleep_time > 0.0:
                time.sleep(sleep_time)


if __name__ == "__main__":
    main()
