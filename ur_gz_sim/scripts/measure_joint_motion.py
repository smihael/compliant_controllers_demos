#!/usr/bin/env python3
"""Read-only measurement of joint motion over a simulation-time window."""
import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--duration', type=float, default=10.0)
parser.add_argument('--timeout', type=float, default=60.0)
parser.add_argument('--output', help='Optional JSON containing summary and raw samples')
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('measure_joint_motion')
samples = []
names = []

def receive(msg):
    global names
    if names and list(msg.name) != names:
        return
    names = list(msg.name)
    samples.append(dict(t=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                        position=list(msg.position), velocity=list(msg.velocity),
                        effort=list(msg.effort)))

subscription = node.create_subscription(JointState, '/joint_states', receive, qos_profile_sensor_data)
deadline = time.monotonic() + args.timeout
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    if len(samples) > 1 and samples[-1]['t'] - samples[0]['t'] >= args.duration:
        break
node.destroy_node()
rclpy.shutdown()
if len(samples) < 2 or samples[-1]['t'] - samples[0]['t'] < args.duration:
    raise SystemExit('Insufficient joint-state data within wall-clock timeout')
q = np.array([s['position'] for s in samples])
v = np.array([s['velocity'] for s in samples])
summary = dict(samples=len(samples), sim_seconds=samples[-1]['t'] - samples[0]['t'], joints={})
for i, name in enumerate(names):
    summary['joints'][name] = dict(position_peak_to_peak_rad=float(np.ptp(q[:, i])),
                                 velocity_rms_rad_s=float(np.sqrt(np.mean(v[:, i] ** 2))),
                                 velocity_peak_rad_s=float(np.max(np.abs(v[:, i]))))
print(json.dumps(summary, indent=2))
if args.output:
    with open(args.output, 'w') as stream:
        json.dump(dict(summary=summary, samples=samples), stream)
