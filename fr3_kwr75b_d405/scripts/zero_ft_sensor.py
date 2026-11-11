#!/usr/bin/env python3
"""Set the FT software zero once fresh samples become available."""
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = Node('zero_ft_sensor')
    prefix = node.declare_parameter('sensor_prefix', 'ft_sensor').value
    timeout = float(node.declare_parameter('timeout', 30.0).value)
    client = node.create_client(Trigger, '/' + prefix + '/zero')
    deadline = time.monotonic() + timeout
    success = False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=min(0.5, max(0.0, deadline-time.monotonic()))):
                continue
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.0, deadline-time.monotonic()))
            if not future.done():
                break
            response = future.result()
            if response.success:
                node.get_logger().info(response.message)
                success = True
                break
            # Service may exist before acquisition has produced a fresh sample.
            rclpy.spin_once(node, timeout_sec=min(0.25, max(0.0, deadline-time.monotonic())))
        if not success:
            node.get_logger().error(f'FT startup zero failed within {timeout:g}s; bringup continues.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
