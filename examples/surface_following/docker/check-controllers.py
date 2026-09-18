"""Query controllers directly, avoiding a shared or stale ROS CLI daemon."""
import rclpy
from controller_manager_msgs.srv import ListControllers

rclpy.init()
node = rclpy.create_node("surface_controller_check")
try:
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    if not client.wait_for_service(timeout_sec=2):
        raise RuntimeError("Controller manager unavailable")
    future = client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=2)
    result = future.result()
    states = {} if result is None else {c.name: c.state for c in result.controller}
    required = ("joint_state_broadcaster", "cartesian_impedance_controller")
    if not all(states.get(name) == "active" for name in required):
        raise RuntimeError(f"Controllers not active: {states}")
finally:
    node.destroy_node()
    rclpy.shutdown()
