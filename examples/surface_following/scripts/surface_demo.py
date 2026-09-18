#!/usr/bin/env python3
"""Headless execution path for the interactive surface-following notebook."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from robotblockset.ros2.franka_ros2 import fr3

from surface_ros_common import wait_for_fr3_graph
from surface_following import (
    SurfaceConfig,
    approach_until_contact,
    follow_surface,
    move_to_safe_start,
    retreat,
    validate_run,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="Acknowledge that robot motion will be commanded")
    parser.add_argument("--namespace", default="")
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--travel", type=float, default=0.18)
    parser.add_argument("--max-follow-time", type=float, default=90.0)
    parser.add_argument("--approach-only", action="store_true")
    parser.add_argument("--hold-contact", type=float, default=5.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not args.execute:
        parser.error("motion is disabled unless --execute is supplied")
    if not 0.03 <= args.travel <= 0.20:
        parser.error("--travel must be between 0.03 and 0.20 m")
    return args


def main() -> None:
    args = parse_args()
    config = SurfaceConfig(
        fixture_base_z=float(os.environ.get("FIXTURE_BASE_Z", "0.55")),
        travel=args.travel,
        max_follow_time=args.max_follow_time,
    )
    robot = None
    contact = None
    data = None
    rclpy.init()
    try:
        wait_for_fr3_graph(args.namespace, args.timeout)
        robot = fr3(control_strategy="CartesianImpedance", SIM=True, ns=args.namespace)
        robot.SetLogLevel("ERROR", logger="Python")
        # Synchronize the trajectory origin before any Cartesian command.  The
        # controller can drift while waiting for the client to connect.
        robot.GetState()
        robot.ResetCurrentTarget(do_move=False)
        move_to_safe_start(robot, config)
        contact = approach_until_contact(robot, config)
        if args.approach_only:
            print(
                "Approach-only run completed: "
                f"travel={contact['travel']:.4f} m, "
                f"target={np.round(np.asarray(contact['target'])[:3], 4)}"
            )
            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                np.savez(args.output, **{f"contact_{key}": value for key, value in contact.items()})
            if args.hold_contact > 0:
                print(f"Holding contact for {args.hold_contact:.1f} s for visual inspection.", flush=True)
                time.sleep(args.hold_contact)
            return
        data = follow_surface(robot, np.asarray(contact["target"]), config)
        metrics = validate_run(data, config)
        print(
            "Surface-following validation passed: "
            f"progress={metrics['progress']:.4f} m, "
            f"Y span={metrics['y_span']:.4f} m, "
            f"max tracking error={metrics['max_tracking_error']:.4f} m"
        )
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            np.savez(args.output, **data, **{f"contact_{key}": value for key, value in contact.items()})
            print(f"Saved run data to {args.output}")
    finally:
        if robot is not None:
            try:
                if contact is not None:
                    retreat(robot)
            finally:
                robot.StopMotion()
                robot.Shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
