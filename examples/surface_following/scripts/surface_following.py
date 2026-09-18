"""Small Cartesian-impedance surface follower used by the demo and notebook.

The implementation is intentionally independent of the large experimental
``trackpy/csf_utils.py`` module.  It keeps only the spatial tangent filter and
the tangent-aligned compliance idea used by ``run_track.ipynb``.
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any

import numpy as np


@dataclass(frozen=True)
class SurfaceConfig:
    fixture_x: float = 0.38
    fixture_base_z: float = 0.55
    start_x: float = 0.40
    center_y: float = 0.0
    # Keep the nominal upright Gazebo flange orientation; do not reorient the
    # arm while positioning over the fixture.
    tool_quaternion_wxyz: tuple[float, float, float, float] = (
        0.0, -0.9238795, 0.3826834, 0.0,
    )
    # At start_x=0.40 the transformed STL face is y=-22.1 mm.  The probe is a
    # vertical cylinder, so its relevant lateral offset is its 25 mm radius.
    fixture_surface_start_y: float = -0.0221
    probe_radius: float = 0.025
    safe_clearance: float = 0.03
    approach_step: float = 0.001
    approach_limit: float = 0.06
    contact_error: float = 0.003
    contact_samples: int = 3
    travel: float = 0.18
    tangent_lead: float = 0.010
    tangent_filter_gain: float = 0.15
    tangent_window_samples: int = 25
    tangent_min_window_progress: float = 0.001
    tangent_min_x: float = 0.87
    sample_period: float = 0.020
    max_follow_time: float = 90.0
    penetration_bias: float = 0.005
    tangent_stiffness: float = 1200.0
    lateral_stiffness: float = 1200.0
    normal_stiffness: float = 80.0
    approach_normal_stiffness: float = 1200.0
    rotational_stiffness: float = 25.0
    preload: float = 4.0
    max_tracking_error: float = 0.045
    max_lateral_error: float = 0.012

    @property
    def fixture_peak_z(self) -> float:
        # slope.STL is 30 mm high after its 0.001 scale is applied.
        return self.fixture_base_z + 0.030

    @property
    def safe_y(self) -> float:
        # The curved face is the -Y (right-hand, viewed by the camera) side.
        # Keep the complete cylindrical probe clear before approaching in +Y.
        return self.fixture_surface_start_y - self.probe_radius - self.safe_clearance

    @property
    def contact_z(self) -> float:
        return self.fixture_base_z + 0.015


def _unit(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    return vector / norm if norm > 1e-9 else np.asarray(fallback, dtype=float).copy()


def spatial_filter_step(tangent: np.ndarray, displacement: np.ndarray, gain: float) -> np.ndarray:
    """Update a unit tangent from measured Cartesian displacement."""
    direction = _unit(displacement, tangent)
    tangent = np.asarray(tangent, dtype=float)
    updated = tangent + gain * (np.eye(3) - np.outer(tangent, tangent)) @ direction
    return _unit(updated, tangent)


def constrain_forward_tangent(tangent: np.ndarray, min_x: float) -> np.ndarray:
    """Keep an XY tangent within the forward slope range of the fixture."""
    tangent = np.asarray(tangent, dtype=float).copy()
    tangent[2] = 0.0
    tangent = _unit(tangent, np.array([1.0, 0.0, 0.0]))
    tangent[0] = max(float(tangent[0]), min_x)
    max_y = float(np.sqrt(max(0.0, 1.0 - min_x * min_x)))
    tangent[1] = float(np.clip(tangent[1], -max_y, max_y))
    return _unit(tangent, np.array([1.0, 0.0, 0.0]))


def surface_frame(tangent: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return a stable X-Z tangent and its upward-facing surface normal.

    Columns of the returned rotation are tangent, lateral world-Y, and normal.
    The frame is right handed and is suitable for RobotBlockSet's ``R``
    compliance argument.
    """
    tangent = np.asarray(tangent, dtype=float).copy()
    tangent[2] = 0.0
    tangent = _unit(tangent, np.array([1.0, 0.0, 0.0]))
    if tangent[0] < 0.15:
        tangent[0] = 0.15
        tangent = _unit(tangent, np.array([1.0, 0.0, 0.0]))
    lateral = np.array([0.0, 0.0, 1.0])
    normal = _unit(np.cross(lateral, tangent), np.array([0.0, 1.0, 0.0]))
    if normal[1] < 0.0:
        lateral *= -1.0
        normal *= -1.0
    return np.column_stack((tangent, lateral, normal)), tangent


def _motion_ok(robot: Any, result: int, label: str) -> None:
    description = robot.MotionResultStr(result) if hasattr(robot, "MotionResultStr") else str(result)
    print(f"{label}: {description}")
    if result != 0:
        raise RuntimeError(f"{label} failed with result code {result} ({description})")


def _go_to(
    robot: Any,
    target: np.ndarray,
    config: SurfaceConfig,
    tangent: np.ndarray,
    preload: float,
    normal_stiffness: float | None = None,
) -> int:
    frame, tangent = surface_frame(tangent)
    # Column 2 is the inward surface normal.  The force therefore remains
    # orthogonal to the tracked tangent as the fixture curves.
    wrench = np.zeros(6)
    wrench[:3] = abs(preload) * frame[:, 2]
    result = robot.GoTo_X(
        target,
        np.zeros(6),
        wrench,
        robot.tsamp,
        Kp=np.array(
            [
                config.tangent_stiffness,
                config.lateral_stiffness,
                config.normal_stiffness if normal_stiffness is None else normal_stiffness,
            ]
        ),
        Kr=np.full(3, config.rotational_stiffness),
        R=frame,
        D=1.5,
    )
    if result != 0:
        _motion_ok(robot, result, "Cartesian impedance command")
    return result


def move_to_safe_start(robot: Any, config: SurfaceConfig, duration: float = 8.0) -> np.ndarray:
    """Move above the low end of the fixture without changing orientation."""
    robot.GetState()
    robot.ResetCurrentTarget(do_move=False)
    robot.SetCartesianStiffness([1200.0, 1200.0, 1200.0, 25.0, 25.0, 25.0])
    target = robot.x.copy()
    target[2] = config.contact_z + config.safe_clearance
    target[1] = config.safe_y
    # Preserve the simulator's current orientation; changing quaternion
    # conventions here can trigger a large redundant-arm reconfiguration.
    _motion_ok(robot, robot.CMove(target, t=duration / 2.0, state="Actual"), "Lift clear of fixture")
    target[:3] = [config.start_x, config.safe_y, config.contact_z + config.safe_clearance]
    _motion_ok(robot, robot.CMove(target, t=duration / 2.0, state="Actual"), "Move above fixture start")
    robot.GetState()
    return robot.x.copy()


def approach_until_contact(robot: Any, config: SurfaceConfig) -> dict[str, np.ndarray | float | int | bool]:
    """Approach the curved face in +Y and detect sustained normal pose lag.

    RobotBlockSet deliberately does not create an external-wrench subscription
    for ``fr3(SIM=True)``.  Pose lag is therefore the observable used here: in
    free space follows the commanded pose, while collision with the static
    fixture makes the inward reference advance beyond the measured pose.
    """
    robot.GetState()
    target = robot.x.copy()
    tangent = np.array([1.0, 0.0, 0.0])
    reference_log: list[np.ndarray] = []
    actual_log: list[np.ndarray] = []
    error_log: list[float] = []
    consecutive = 0
    travelled = 0.0

    while travelled < config.approach_limit:
        target = target.copy()
        target[1] += config.approach_step
        # Do not apply feed-forward preload during contact search: with a soft
        # normal axis it creates free-space pose lag that looks like contact.
        # Preload is enabled only after collision has been established.
        _go_to(
            robot,
            target,
            config,
            tangent,
            preload=0.0,
            normal_stiffness=config.approach_normal_stiffness,
        )
        time.sleep(max(config.sample_period, 0.060))
        robot.GetState()
        actual = robot.x.copy()
        normal_error = float(target[1] - actual[1])
        reference_log.append(target[:3].copy())
        actual_log.append(actual[:3].copy())
        error_log.append(normal_error)
        travelled += config.approach_step

        if travelled >= 0.012 and normal_error >= config.contact_error:
            consecutive += 1
        else:
            consecutive = 0
        if consecutive >= config.contact_samples:
            print(
                f"Contact detected after {travelled:.3f} m approach "
                f"(normal tracking error {normal_error:.4f} m)."
            )
            return {
                "contact": True,
                "travel": travelled,
                "reference_positions": np.asarray(reference_log),
                "positions": np.asarray(actual_log),
                "normal_errors": np.asarray(error_log),
                "target": target,
            }

    raise RuntimeError(
        f"No fixture contact within {config.approach_limit:.3f} m; "
        f"last normal error was {error_log[-1] if error_log else 0.0:.4f} m"
    )


def follow_surface(robot: Any, contact_target: np.ndarray, config: SurfaceConfig) -> dict[str, np.ndarray]:
    """Traverse with a fixed tangent and orthogonal fixture-directed preload."""
    robot.GetState()
    start = robot.x.copy()
    previous = start[:3].copy()
    target = np.asarray(contact_target, dtype=float).copy()
    # +X points away from the robot base and initially aligns with the fixture.
    tangent = np.array([1.0, 0.0, 0.0])

    positions: list[np.ndarray] = []
    references: list[np.ndarray] = []
    tangents: list[np.ndarray] = []
    normals: list[np.ndarray] = []
    tracking_errors: list[np.ndarray] = []
    timestamps: list[float] = []
    tangent_history: list[np.ndarray] = [previous.copy()]
    max_steps = int(np.ceil(config.max_follow_time / config.sample_period))

    for step in range(max_steps):
        frame, tangent = surface_frame(tangent)
        target = target.copy()
        # Build the next reference from measured pose so reference lead cannot
        # accumulate when contact slows the arm. A small offset into the
        # surface preserves normal spring deflection in addition to preload.
        target[:3] = previous + config.tangent_lead * tangent + config.penetration_bias * frame[:, 2]
        # Hold the height measured at contact.  Copying the latest measured Z
        # here would integrate tiny tracking errors into a visible vertical
        # drift over a long run.
        target[2] = start[2]
        _go_to(robot, target, config, tangent, preload=config.preload)
        time.sleep(config.sample_period)
        robot.GetState()

        actual = robot.x.copy()
        error = actual[:3] - target[:3]
        progress = float(actual[0] - start[0])
        if np.linalg.norm(error) > config.max_tracking_error:
            raise RuntimeError(
                f"Tracking error {np.linalg.norm(error):.4f} m exceeded "
                f"{config.max_tracking_error:.4f} m at step {step}; "
                f"progress={progress:.4f} m, actual={np.round(actual[:3], 4)}, "
                f"reference={np.round(target[:3], 4)}"
            )
        # Keep recording the contact response even when the tool drifts in Z;
        # the experiment is evaluated from contact/progress data below.

        tangent_history.append(actual[:3].copy())
        if len(tangent_history) > config.tangent_window_samples:
            tangent_history.pop(0)
        window_displacement = tangent_history[-1] - tangent_history[0]
        window_displacement[2] = 0.0
        # Do not learn a direction from contact jitter.  A valid observation
        # must contain measurable +X progress away from the robot.
        if window_displacement[0] >= config.tangent_min_window_progress:
            tangent = spatial_filter_step(
                tangent, window_displacement, config.tangent_filter_gain
            )
        tangent = constrain_forward_tangent(tangent, config.tangent_min_x)
        previous = actual[:3].copy()

        positions.append(actual.copy())
        references.append(target.copy())
        tangents.append(tangent.copy())
        normals.append(frame[:, 2].copy())
        tracking_errors.append(error.copy())
        timestamps.append(float(robot.t))

        if step % 250 == 0:
            print(
                f"track step={step} progress={progress:.4f} m "
                f"actual={np.round(actual[:3], 4)} reference={np.round(target[:3], 4)}",
                flush=True,
            )

        euclidean_progress = float(np.linalg.norm(actual[:3] - start[:3]))
        if euclidean_progress >= config.travel:
            print(
                f"Surface traversal completed at step {step}: "
                f"euclidean displacement={euclidean_progress:.4f} m"
            )
            break
    else:
        final_progress = float(np.linalg.norm(previous - start[:3]))
        raise RuntimeError(
            f"Surface traversal did not reach {config.travel:.3f} m; "
            f"final Euclidean displacement was {final_progress:.4f} m after {max_steps} steps"
        )

    return {
        "positions": np.asarray(positions),
        "reference_poses": np.asarray(references),
        "tangents": np.asarray(tangents),
        "normals": np.asarray(normals),
        "tracking_errors": np.asarray(tracking_errors),
        "timestamps": np.asarray(timestamps),
    }


def retreat(robot: Any, distance: float = 0.07, duration: float = 3.0) -> None:
    """Remove preload, synchronize the reference, and retreat in world -Y."""
    robot.GetState()
    hold = robot.x.copy()
    robot.GoTo_X(
        hold,
        np.zeros(6),
        np.zeros(6),
        robot.tsamp,
        Kp=np.array([1200.0, 1200.0, 1200.0]),
        Kr=np.array([25.0, 25.0, 25.0]),
        R=np.eye(3),
        D=1.5,
    )
    time.sleep(0.1)
    robot.GetState()
    robot.ResetCurrentTarget(do_move=False)
    _motion_ok(robot, robot.CMoveFor([0.0, -abs(distance), 0.0], t=duration), "Retreat from fixture")


def validate_run(data: dict[str, np.ndarray], config: SurfaceConfig) -> dict[str, float]:
    positions = np.asarray(data["positions"])
    if len(positions) < 2:
        raise RuntimeError("Surface run returned too few samples")
    progress = float(np.linalg.norm(positions[-1, :3] - positions[0, :3]))
    y_span = float(np.ptp(positions[:, 1]))
    max_error = float(np.max(np.linalg.norm(data["tracking_errors"], axis=1)))
    if progress < 0.85 * config.travel:
        raise RuntimeError(f"Insufficient surface progress: {progress:.4f} m")
    required_y_span = 0.006 if config.travel >= 0.10 else 0.001
    if y_span < required_y_span:
        raise RuntimeError(f"Tool did not follow the curved profile: Y span was only {y_span:.4f} m")
    return {"progress": progress, "y_span": y_span, "max_tracking_error": max_error}
