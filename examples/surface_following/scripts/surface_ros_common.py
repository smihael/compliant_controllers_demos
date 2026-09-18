#!/usr/bin/env python3
"""Bounded ROS graph checks shared by the FR3 surface demos."""

from __future__ import annotations

import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def normalized_namespace(namespace: str) -> str:
    stripped = namespace.strip("/")
    return f"/{stripped}" if stripped else ""


def wait_for_fr3_graph(namespace: str, timeout: float) -> None:
    """Wait for the controller manager and a complete FR3 joint-state sample."""
    ns = normalized_namespace(namespace)
    node = Node("surface_demo_preflight")
    service_name = f"{ns}/controller_manager/list_controllers"
    joint_state_topic = f"{ns}/joint_states"
    client = node.create_client(ListControllers, service_name)
    expected_joints = {f"fr3_joint{index}" for index in range(1, 8)}
    observed_joints: set[str] = set()

    def joint_state_callback(message: JointState) -> None:
        observed_joints.update(message.name)

    subscription = node.create_subscription(
        JointState,
        joint_state_topic,
        joint_state_callback,
        qos_profile_sensor_data,
    )
    del subscription  # The node retains ownership of the subscription.

    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if client.service_is_ready() and expected_joints.issubset(observed_joints):
                print(f"ROS preflight passed: {service_name}, {joint_state_topic}")
                return
        missing = sorted(expected_joints - observed_joints)
        raise TimeoutError(
            f"ROS preflight timed out after {timeout:.1f}s; "
            f"controller_manager_ready={client.service_is_ready()}, "
            f"missing_joint_states={missing}"
        )
    finally:
        node.destroy_node()
