#!/usr/bin/env python3
"""Generate and optionally execute the geometry-only fixture-edge baseline."""

from __future__ import annotations

import argparse
import struct
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_profile(path: Path, samples: int = 151) -> np.ndarray:
    raw = path.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    vertices = []
    for i in range(count):
        values = struct.unpack_from("<12f", raw, 84 + 50 * i)
        vertices.extend((values[3:6], values[6:9], values[9:12]))
    v = np.asarray(vertices, dtype=float)
    # The STL profile is x-z and is extruded along y.  Take the upper curved
    # edge by binning x and retaining the maximum z in each bin.
    x = v[:, 0]
    # STL vertices repeat per triangle and include the bottom/side faces. Keep
    # the upper envelope at every unique x before resampling it smoothly.
    ux = np.unique(x)
    uz = np.asarray([v[x == value, 2].max() for value in ux])
    keep = uz > 1e-3
    ux, uz = ux[keep], uz[keep]
    bins = np.linspace(float(x.min()), float(x.max()), samples)
    z = np.interp(bins, ux, uz)
    return np.column_stack((bins, z)) * 1e-3


def world_edge(profile: np.ndarray, translation=(0.38, -0.015, 0.55), extrusion=0.015) -> np.ndarray:
    """Apply the Gazebo spawn pose and +90° X rotation to the STL edge."""
    tx, ty, tz = translation
    x = tx + profile[:, 0]
    y = ty - profile[:, 1]
    z = np.full_like(x, tz + extrusion)
    return np.column_stack((x, y, z))


def save_plot(edge: np.ndarray, path: Path, clearance: float) -> None:
    baseline = edge.copy()
    baseline[:, 1] += clearance
    fig = plt.figure(figsize=(10, 5))
    ax = fig.add_subplot(121)
    ax.plot(edge[:, 0], edge[:, 1], label="fixture curved edge")
    ax.plot(baseline[:, 0], baseline[:, 1], "--", label=f"baseline +Y {clearance:.3f} m")
    ax.set_xlabel("world X [m]"); ax.set_ylabel("world Y [m]"); ax.axis("equal"); ax.grid(); ax.legend()
    ax = fig.add_subplot(122, projection="3d")
    ax.plot(edge[:, 0], edge[:, 1], edge[:, 2], label="fixture edge")
    ax.plot(baseline[:, 0], baseline[:, 1], baseline[:, 2], "--", label="baseline")
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z"); ax.legend()
    fig.tight_layout(); fig.savefig(path, dpi=160); plt.close(fig)


def execute_baseline(points: np.ndarray, stride: int = 10) -> None:
    import rclpy
    from robotblockset.ros2.franka_ros2 import fr3
    from surface_ros_common import wait_for_fr3_graph

    rclpy.init(); robot = None
    try:
        wait_for_fr3_graph("", 60.0)
        robot = fr3(control_strategy="CartesianImpedance", SIM=True)
        robot.SetLogLevel("ERROR", logger="Python")
        robot.GetState(); robot.ResetCurrentTarget(do_move=False)
        target = robot.x.copy()
        selected = points[::stride]
        if not np.array_equal(selected[-1], points[-1]):
            selected = np.vstack((selected, points[-1]))
        for i, point in enumerate(selected):
            target[:3] = point
            result = robot.CMove(target, t=0.15, state="Actual")
            robot.GetState()
            print(f"baseline {i:03d}/{len(selected)-1}: result={result} actual={np.round(robot.x[:3], 4)}", flush=True)
    finally:
        if robot is not None:
            robot.StopMotion(); robot.Shutdown()
        if rclpy.ok(): rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--stl",
        type=Path,
        default=Path("/opt/demo/fixtures/slope.STL"),
    )
    parser.add_argument("--output", type=Path, default=Path("/tmp/surface_baseline.npz"))
    parser.add_argument("--plot", type=Path, default=Path("/tmp/surface_baseline.png"))
    parser.add_argument("--clearance", type=float, default=0.03)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--stride", type=int, default=10)
    args = parser.parse_args()
    edge = world_edge(load_profile(args.stl))
    baseline = edge.copy(); baseline[:, 1] += args.clearance
    np.savez(args.output, fixture_edge=edge, baseline=baseline)
    save_plot(edge, args.plot, args.clearance)
    print(f"fixture edge: x=[{edge[:,0].min():.4f},{edge[:,0].max():.4f}] "
          f"y=[{edge[:,1].min():.4f},{edge[:,1].max():.4f}] z={edge[:,2].mean():.4f}")
    print(f"saved trajectory={args.output} plot={args.plot}")
    if args.execute: execute_baseline(baseline, max(1, args.stride))


if __name__ == "__main__": main()
