#!/usr/bin/env python3
"""Republish the already-zeroed sensor wrench about the TCP origin."""

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def cross(a, b):
    return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0])


def rotate(vector, quaternion):
    """Rotate by a unit TF quaternion (source axes into target axes)."""
    q = (quaternion.x, quaternion.y, quaternion.z)
    uv = cross(q, vector)
    uuv = cross(q, uv)
    return tuple(vector[i] + 2*(quaternion.w*uv[i] + uuv[i]) for i in range(3))


def transform_wrench(message, transform, target_frame):
    result = WrenchStamped()
    result.header.stamp = message.header.stamp
    result.header.frame_id = target_frame
    f, t = message.wrench.force, message.wrench.torque
    force = rotate((f.x, f.y, f.z), transform.rotation)
    torque = rotate((t.x, t.y, t.z), transform.rotation)
    p = transform.translation
    # p is the source origin expressed in target coordinates. Move the moment
    # reference point as well as rotating its axes: tau_T = R*tau_S + p x F_T.
    moment = cross((p.x, p.y, p.z), force)
    result.wrench.force.x, result.wrench.force.y, result.wrench.force.z = force
    (result.wrench.torque.x, result.wrench.torque.y,
     result.wrench.torque.z) = tuple(torque[i] + moment[i] for i in range(3))
    return result


class TcpWrenchRepublisher(Node):
    def __init__(self):
        super().__init__('tcp_wrench_republisher')
        input_topic = self.declare_parameter('input_topic', '/ft_sensor/wrench').value
        output_topic = self.declare_parameter('output_topic', '/ft_sensor/wrench_tcp').value
        self.target = self.declare_parameter('target_frame', 'fr3_hand_tcp').value
        if not self.target or not input_topic or not output_topic:
            raise ValueError('Topics and target_frame must be nonempty')
        self.publisher = self.create_publisher(WrenchStamped, output_topic, 10)
        if self.resolve_topic_name(input_topic) == self.resolve_topic_name(output_topic):
            raise ValueError('Input and output topics must differ')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.subscription = self.create_subscription(
            WrenchStamped, input_topic, self.callback, qos_profile_sensor_data)
        self.get_logger().info(f'{input_topic} -> {output_topic}, about {self.target}')

    def callback(self, message):
        if not message.header.frame_id:
            self.get_logger().warning('Dropping wrench with empty frame_id', throttle_duration_sec=5.0)
            return
        try:
            transform = self.buffer.lookup_transform(
                self.target, message.header.frame_id, Time.from_msg(message.header.stamp))
        except TransformException as exc:
            self.get_logger().warning(f'Dropping wrench: {exc}', throttle_duration_sec=5.0)
            return
        self.publisher.publish(transform_wrench(message, transform.transform, self.target))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TcpWrenchRepublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
