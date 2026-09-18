#!/usr/bin/env python3
"""Diagnostic Cartesian hold/step with independently specified damping."""
import argparse
import math
import time

import rclpy
from compliant_controllers_msgs.msg import CartesianCommand
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


def diagonal(value):
    return [value, 0.0, 0.0, 0.0, value, 0.0, 0.0, 0.0, value]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--ee-frame', default='tool0')
    parser.add_argument('--k-lin', type=float, default=150.0)
    parser.add_argument('--k-rot', type=float, default=10.0)
    parser.add_argument('--d-lin', type=float, default=10.0)
    parser.add_argument('--d-rot', type=float, default=0.05)
    parser.add_argument('--dx', type=float, default=0.0)
    args = parser.parse_args()
    gains = (args.k_lin, args.k_rot, args.d_lin, args.d_rot)
    if any(not math.isfinite(v) or v < 0 for v in gains):
        parser.error('Gains must be finite and nonnegative')
    if not math.isfinite(args.dx) or abs(args.dx) > 0.01:
        parser.error('Use a finite X offset of at most 0.01 m')
    rclpy.init()
    node = rclpy.create_node('send_damped_hold')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    publisher = node.create_publisher(CartesianCommand, '/cartesian_command', 10)
    deadline = time.monotonic() + 15.0
    transform = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = buffer.lookup_transform(args.base_frame, args.ee_frame, Time())
        except TransformException:
            continue
        if publisher.get_subscription_count():
            break
    else:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit('No end-effector transform or command subscriber within 15 seconds')
    cmd = CartesianCommand()
    cmd.header.stamp = node.get_clock().now().to_msg()
    # This API identifies the controlled frame in the header; pose is in base_frame.
    cmd.header.frame_id = args.ee_frame
    cmd.pose.position.x = transform.transform.translation.x + args.dx
    cmd.pose.position.y = transform.transform.translation.y
    cmd.pose.position.z = transform.transform.translation.z
    cmd.pose.orientation = transform.transform.rotation
    cmd.stiffness_pos = diagonal(args.k_lin)
    cmd.stiffness_ori = diagonal(args.k_rot)
    cmd.damping_pos = diagonal(args.d_lin)
    cmd.damping_ori = diagonal(args.d_rot)
    publisher.publish(cmd)
    print(f'Published hold/step: dx={args.dx}, K=({args.k_lin}, {args.k_rot}), '
          f'D=({args.d_lin}, {args.d_rot}); target={cmd.pose}')
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
