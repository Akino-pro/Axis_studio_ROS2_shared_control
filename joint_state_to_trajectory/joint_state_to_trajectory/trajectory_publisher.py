```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

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

        # Storage for positions: start previous at zeros so first trajectory is valid
        n = len(self.controller_joints)
        self.previous_positions = [0.0] * n
        self.latest_positions = None

        # Publisher for trajectory
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

        # no timer; publish on each incoming state

    def joint_state_cb(self, msg: JointState):
        # Map names to positions
        name_pos = dict(zip(msg.name, msg.position))
        try:
            current = [ name_pos[j] for j in self.controller_joints ]
        except KeyError as e:
            self.get_logger().error(f"Missing joint {e} in /fake_joint_states")
            return

        # set latest and publish
        self.latest_positions = current
        self.publish_trajectory()

        # update previous for next cycle
        self.previous_positions = self.latest_positions

    def publish_trajectory(self):
        # compute deltas
        deltas = [curr - prev for curr, prev in zip(self.latest_positions, self.previous_positions)]

        # fixed velocity magnitude
        vel_mag = 0.02

        # compute execution time per joint
        times = [abs(delta) / vel_mag for delta in deltas]
        exec_time = max(times)

        # build velocity vector with correct sign
        velocities = [vel_mag if delta >= 0 else -vel_mag for delta in deltas]

        # build trajectory message
        traj = JointTrajectory()
        traj.joint_names = self.controller_joints

        pt = JointTrajectoryPoint()
        pt.positions = self.latest_positions
        pt.velocities = velocities

        # convert exec_time to sec & nanosec
        sec = int(math.floor(exec_time))
        nsec = int((exec_time - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)

        traj.points = [pt]
        self.traj_pub.publish(traj)

        self.get_logger().info(
            f"Published traj: pos={[f'{x:.3f}' for x in self.latest_positions]}"
            f" vel={[f'{v:.3f}' for v in velocities]}"
            f" time={exec_time:.3f}s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = AngleToTrajectory()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

