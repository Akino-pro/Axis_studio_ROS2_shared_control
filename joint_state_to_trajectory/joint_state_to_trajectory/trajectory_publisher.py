#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import numpy as np

class AngleToTrajectory(Node):
    def __init__(self):
        super().__init__('angle_to_trajectory')
        self.get_logger().info("angle_to_trajectory node started")

        # Joints to follow (must match controller)
        self.declare_parameter('controller_joints', [
            'joint_1','joint_2','joint_3','joint_4',
            'joint_5','joint_6','joint_7'
        ])
        self.controller_joints = self.get_parameter('controller_joints').value

        # Limits
        self.max_vel = 0.2   # rad/s
        self.min_norm = 0.01 # rad, ignore smaller moves
        self.max_time = 15.0 # seconds, abort beyond

        # State buffers
        n = len(self.controller_joints)
        self.previous_positions = [0.0] * n  # start at zero
        self.buffered_positions = None
        self.last_published_positions = None
        self.busy = False
        self.timer = None

        # Publisher
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)

        # Subscribe to fake joint states
        self.create_subscription(
            JointState,
            '/fake_joint_states',
            self.joint_state_cb,
            10)

    def joint_state_cb(self, msg: JointState):
        # Map names to positions
        name_pos = dict(zip(msg.name, msg.position))
        try:
            current = [ name_pos[j] for j in self.controller_joints ]
        except KeyError as e:
            self.get_logger().error(f"Missing joint {e} in /fake_joint_states")
            return

        # buffer incoming
        self.buffered_positions = current
        # try to publish if idle
        self.try_publish()

    def try_publish(self):
        if self.busy or self.buffered_positions is None:
            return

        # Prepare positions
        new_pos = self.buffered_positions
        old_pos = self.last_published_positions if self.last_published_positions is not None else self.previous_positions

        # compute deltas
        deltas = [new - old for new, old in zip(new_pos, old_pos)]

        # skip minor moves
        norm = float(np.linalg.norm(deltas))
        if norm < self.min_norm:
            self.get_logger().info(f"Movement norm {norm:.3f} < {self.min_norm}, ignoring small jitter.")
            self.buffered_positions = None
            return

        # compute individual times and pick max
        times = [abs(delta) / self.max_vel for delta in deltas]
        exec_time = max(times)

        # check for time limit
        if exec_time > self.max_time:
            self.get_logger().error(
                f"Execution time {exec_time:.2f}s exceeds limit ({self.max_time}s), skipping trajectory.")
            self.buffered_positions = None
            return

        # compute velocities proportional to delta/exectime
        velocities = [delta / exec_time for delta in deltas]

        # build trajectory message
        traj = JointTrajectory()
        traj.joint_names = self.controller_joints

        pt = JointTrajectoryPoint()
        pt.positions = new_pos
        pt.velocities = velocities
        sec = int(math.floor(exec_time))
        nsec = int((exec_time - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)

        traj.points = [pt]
        self.traj_pub.publish(traj)
        self.get_logger().info(
            f"Published traj: pos={[f'{x:.3f}' for x in new_pos]}"
            f" vel={[f'{v:.3f}' for v in velocities]}"
            f" time={exec_time:.3f}s"
        )

        # mark busy and schedule next
        self.busy = True
        self.last_published_positions = new_pos
        self.buffered_positions = None
        # create one-shot timer
        self.timer = self.create_timer(exec_time, self.on_timer_done)

    def on_timer_done(self):
        # clear busy and destroy timer
        self.busy = False
        if self.timer:
            self.timer.cancel()
            self.timer = None
        # attempt to publish next buffered
        self.try_publish()


def main(args=None):
    rclpy.init(args=args)
    node = AngleToTrajectory()
    try:
        rclpy.spin(node)
    finally:
        if node.timer:
            node.timer.cancel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
