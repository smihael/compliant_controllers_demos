#!/usr/bin/env python3
import math

import rclpy
from compliant_controllers_msgs.msg import CartesianCommand
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def quat_normalize(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [v / norm for v in q]


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return quat_normalize([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def small_angle_quaternion(wx, wy, wz):
    angle = math.sqrt(wx * wx + wy * wy + wz * wz)
    if angle <= 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    half = 0.5 * angle
    scale = math.sin(half) / angle
    return quat_normalize([wx * scale, wy * scale, wz * scale, math.cos(half)])


class SpaceMouseToCartesianCommand(Node):
    def __init__(self):
        super().__init__("spacemouse_to_cartesian_command")

        self.declare_parameter("input_topic", "franka_controller/target_cartesian_velocity_percent")
        self.declare_parameter("output_topic", "cartesian_command")
        self.declare_parameter("base_frame", "fr3_link0")
        self.declare_parameter("ee_frame", "fr3_link8")
        self.declare_parameter("publish_hz", 100.0)
        self.declare_parameter("linear_scale", 0.08)
        self.declare_parameter("angular_scale", 0.35)
        self.declare_parameter("deadband", 0.04)
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("max_linear_step", 0.004)
        self.declare_parameter("max_angular_step", 0.02)
        self.declare_parameter("k_lin", 250.0)
        self.declare_parameter("k_rot", 18.0)
        self.declare_parameter("damping_ratio", 1.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.deadband = float(self.get_parameter("deadband").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.max_linear_step = float(self.get_parameter("max_linear_step").value)
        self.max_angular_step = float(self.get_parameter("max_angular_step").value)
        self.k_lin = float(self.get_parameter("k_lin").value)
        self.k_rot = float(self.get_parameter("k_rot").value)
        self.damping_ratio = float(self.get_parameter("damping_ratio").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(CartesianCommand, self.output_topic, 10)
        self.subscription = self.create_subscription(Twist, self.input_topic, self._twist_callback, 10)

        self.target_position = None
        self.target_orientation = None
        self.latest_twist = Twist()
        self.latest_twist_time = None
        self.last_update_time = self.get_clock().now()

        period = 1.0 / max(self.publish_hz, 1.0)
        self.timer = self.create_timer(period, self._timer_callback)
        self.get_logger().info(
            f"Bridging Twist '{self.input_topic}' to CartesianCommand '{self.output_topic}' "
            f"for {self.base_frame}->{self.ee_frame} at {self.publish_hz:.1f} Hz"
        )

    def _twist_callback(self, msg):
        self.latest_twist = msg
        self.latest_twist_time = self.get_clock().now()

    def _lookup_current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception as ex:
            self.get_logger().debug(f"TF {self.base_frame}->{self.ee_frame} unavailable: {ex}")
            return False

        self.target_position = [
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ]
        self.target_orientation = quat_normalize([
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ])
        return True

    def _filtered_axis(self, value):
        if abs(value) < self.deadband:
            return 0.0
        return float(value)

    def _active_twist(self):
        if self.latest_twist_time is None:
            return Twist()
        age = (self.get_clock().now() - self.latest_twist_time).nanoseconds * 1e-9
        if age > self.command_timeout:
            return Twist()
        return self.latest_twist

    def _integrate_target(self, dt):
        twist = self._active_twist()
        linear = [
            self._filtered_axis(twist.linear.x) * self.linear_scale,
            self._filtered_axis(twist.linear.y) * self.linear_scale,
            self._filtered_axis(twist.linear.z) * self.linear_scale,
        ]
        angular = [
            self._filtered_axis(twist.angular.x) * self.angular_scale,
            self._filtered_axis(twist.angular.y) * self.angular_scale,
            self._filtered_axis(twist.angular.z) * self.angular_scale,
        ]

        for i in range(3):
            step = clamp(linear[i] * dt, -self.max_linear_step, self.max_linear_step)
            self.target_position[i] += step

        angular_step = [
            clamp(angular[0] * dt, -self.max_angular_step, self.max_angular_step),
            clamp(angular[1] * dt, -self.max_angular_step, self.max_angular_step),
            clamp(angular[2] * dt, -self.max_angular_step, self.max_angular_step),
        ]
        dq = small_angle_quaternion(*angular_step)
        self.target_orientation = quat_multiply(dq, self.target_orientation)
        return linear, angular

    def _impedance(self):
        d_lin = self.damping_ratio * 2.0 * math.sqrt(max(self.k_lin, 0.0))
        d_rot = self.damping_ratio * 2.0 * math.sqrt(max(self.k_rot, 0.0))
        k_pos = [self.k_lin, 0.0, 0.0, 0.0, self.k_lin, 0.0, 0.0, 0.0, self.k_lin]
        k_ori = [self.k_rot, 0.0, 0.0, 0.0, self.k_rot, 0.0, 0.0, 0.0, self.k_rot]
        d_pos = [d_lin, 0.0, 0.0, 0.0, d_lin, 0.0, 0.0, 0.0, d_lin]
        d_ori = [d_rot, 0.0, 0.0, 0.0, d_rot, 0.0, 0.0, 0.0, d_rot]
        return k_pos, k_ori, d_pos, d_ori

    def _timer_callback(self):
        if self.target_position is None or self.target_orientation is None:
            if not self._lookup_current_pose():
                return
            self.get_logger().info("Initialized teleop target from current TF pose")

        now = self.get_clock().now()
        dt = max((now - self.last_update_time).nanoseconds * 1e-9, 0.0)
        self.last_update_time = now
        linear, angular = self._integrate_target(dt)
        k_pos, k_ori, d_pos, d_ori = self._impedance()

        cmd = CartesianCommand()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = self.ee_frame
        cmd.pose.position.x = self.target_position[0]
        cmd.pose.position.y = self.target_position[1]
        cmd.pose.position.z = self.target_position[2]
        cmd.pose.orientation.x = self.target_orientation[0]
        cmd.pose.orientation.y = self.target_orientation[1]
        cmd.pose.orientation.z = self.target_orientation[2]
        cmd.pose.orientation.w = self.target_orientation[3]
        cmd.velocity.linear.x = linear[0]
        cmd.velocity.linear.y = linear[1]
        cmd.velocity.linear.z = linear[2]
        cmd.velocity.angular.x = angular[0]
        cmd.velocity.angular.y = angular[1]
        cmd.velocity.angular.z = angular[2]
        cmd.stiffness_pos = k_pos
        cmd.stiffness_ori = k_ori
        cmd.damping_pos = d_pos
        cmd.damping_ori = d_ori
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SpaceMouseToCartesianCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
