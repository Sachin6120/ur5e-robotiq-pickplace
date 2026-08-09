#!/usr/bin/env python3
# Dedicated rclpy subscriber to /joint_states, sampling the master gripper
# joint's position+velocity at whatever rate it actually publishes, for a
# fixed wall-clock duration. Same reasoning as the project's original
# 2026-08-06 noise-floor measurement: `ros2 topic echo` text parsing can't
# keep up with a fast publisher, and gz-topic polling (this session's own
# ad-hoc method) is far too coarse (~1 sample/sec) to see sub-second
# velocity structure. This does neither -- direct subscription, callback
# timestamps the arrival, not a polling loop.
import sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT = 'robotiq_85_left_knuckle_joint'

class Sampler(Node):
    def __init__(self, outfile, duration_s):
        super().__init__('velocity_sampler')
        self.outfile = open(outfile, 'w')
        self.t0 = time.time()
        self.duration_s = duration_s
        self.count = 0
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 50)

    def cb(self, msg):
        now = time.time()
        if now - self.t0 > self.duration_s:
            return
        try:
            idx = msg.name.index(JOINT)
        except ValueError:
            return
        pos = msg.position[idx] if idx < len(msg.position) else float('nan')
        vel = msg.velocity[idx] if idx < len(msg.velocity) else float('nan')
        self.outfile.write(f'{now:.6f} {pos:.6f} {vel:.6f}\n')
        self.outfile.flush()
        self.count += 1

def main():
    outfile, duration_s = sys.argv[1], float(sys.argv[2])
    rclpy.init()
    node = Sampler(outfile, duration_s)
    end = time.time() + duration_s + 1.0
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    print(f'captured {node.count} samples', file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
