#!/usr/bin/env python3
#
# Smooth online time-optimal trajectories with Ruckig
#
#   • Joint_7 may use a higher velocity (0.7 rad/s).
#   • All joints share common accel / jerk limits (configurable below).
#   • Each incoming target is converted into a multi-point JointTrajectory
#     sampled at dt = 0.02 s from the Ruckig OTG.
#
# Install:  pip install ruckig
#
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import numpy as np
from ruckig import InputParameter, OutputParameter, Ruckig, Result

DT = 0.02  # controller sample time (s)


class AngleToTrajectory(Node):
    def __init__(self):
        super().__init__('angle_to_trajectory')
        self.get_logger().info("angle_to_trajectory (Ruckig) node started")

        # ---- joint list ----------------------------------------------------
        self.declare_parameter(
            'controller_joints',
            ['joint_1', 'joint_2', 'joint_3',
             'joint_4', 'joint_5', 'joint_6', 'joint_7']
        )
        self.controller_joints = self.get_parameter('controller_joints').value
        self.dof = len(self.controller_joints)

        # ---- limits --------------------------------------------------------
        self.max_vel           = 0.3   # rad/s  (default joints)
        self.max_acc           = 0.6   # rad/s²
        self.max_jerk          = 3.0   # rad/s³
        self.special_joint     = 'joint_7'
        self.special_max_vel   = 0.7   # rad/s  (joint_7)
        self.special_max_acc   = 0.6   # rad/s²
        self.special_max_jerk  = 3.0   # rad/s³

        self.min_norm = 0.01          # rad, ignore tiny moves
        self.max_time = 30.0          # s, safety cap

        # ---- state ---------------------------------------------------------
        self.previous_positions      = [0.0] * self.dof
        self.buffered_positions      = None
        self.last_published_positions = None
        self.busy  = False
        self.timer = None

        # ---- ROS I/O -------------------------------------------------------
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.create_subscription(
            JointState,
            '/fake_joint_states',
            self.joint_state_cb,
            10
        )

        # ---- Ruckig object -------------------------------------------------
        self.otg = Ruckig(self.dof, DT)

    # ------------------------------------------------------------------ cb
    def joint_state_cb(self, msg: JointState):
        name_pos = dict(zip(msg.name, msg.position))
        try:
            current = [name_pos[j] for j in self.controller_joints]
        except KeyError as e:
            self.get_logger().error(f"Missing joint {e} in /fake_joint_states")
            return

        self.buffered_positions = current
        self.try_publish()

    # ------------------------------------------------------------------ main
    def try_publish(self):
        if self.busy or self.buffered_positions is None:
            return

        old_pos = (self.last_published_positions
                   if self.last_published_positions is not None
                   else self.previous_positions)
        raw_deltas = [n - o for n, o in zip(self.buffered_positions, old_pos)]

        # ---- π-wrap shortest path ----------------------------------------
        deltas = []
        for d in raw_deltas:
            if d < 0:
                d = d + 2.0*math.pi if abs(d) > abs(d + 2.0*math.pi) else d
            else:
                d = d - 2.0*math.pi if abs(d - 2.0*math.pi) < abs(d) else d
            deltas.append(d)

        if np.linalg.norm(deltas) < self.min_norm:
            self.buffered_positions = None
            return

        goal_pos = [o + d for o, d in zip(old_pos, deltas)]

        # ---- Ruckig input -------------------------------------------------
        inp = InputParameter(self.dof)
        out = OutputParameter(self.dof)

        inp.current_position     = old_pos
        inp.current_velocity     = [0.0] * self.dof
        inp.current_acceleration = [0.0] * self.dof
        inp.target_position      = goal_pos
        inp.target_velocity      = [0.0] * self.dof
        inp.target_acceleration  = [0.0] * self.dof

        max_v, max_a, max_j = [], [], []
        for j_name in self.controller_joints:
            if j_name == self.special_joint:
                max_v.append(self.special_max_vel)
                max_a.append(self.special_max_acc)
                max_j.append(self.special_max_jerk)
            else:
                max_v.append(self.max_vel)
                max_a.append(self.max_acc)
                max_j.append(self.max_jerk)

        inp.max_velocity     = max_v
        inp.max_acceleration = max_a
        inp.max_jerk         = max_j

        # ---- generate full trajectory offline ----------------------------
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.controller_joints

        t_from_start = 0.0
        while True:
            res = self.otg.update(inp, out)
            if res == Result.Error:
                self.get_logger().error("Ruckig returned an error; skipping.")
                self.buffered_positions = None
                return

            pt = JointTrajectoryPoint()
            pt.positions      = out.new_position.copy()
            pt.velocities     = out.new_velocity.copy()
            pt.accelerations  = out.new_acceleration.copy()
            sec               = int(t_from_start)
            nsec              = int((t_from_start - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            traj_msg.points.append(pt)

            if res == Result.Finished:
                break

            # prepare next cycle
            t_from_start += DT
            inp.current_position     = out.new_position
            inp.current_velocity     = out.new_velocity
            inp.current_acceleration = out.new_acceleration

        exec_time = (traj_msg.points[-1].time_from_start.sec +
                     traj_msg.points[-1].time_from_start.nanosec * 1e-9)

        if exec_time > self.max_time:
            self.get_logger().error(
                f"Ruckig duration {exec_time:.2f}s exceeds {self.max_time}s limit."
            )
            self.buffered_positions = None
            return

        # ---- publish ------------------------------------------------------
        self.traj_pub.publish(traj_msg)
        self.get_logger().info(
            f"Published smooth traj with {len(traj_msg.points)} points "
            f"(duration {exec_time:.3f}s)"
        )

        # mark busy – release when done
        self.busy  = True
        self.last_published_positions = goal_pos
        self.buffered_positions = None
        self.timer = self.create_timer(exec_time, self.on_timer_done)

    # ---------------------------------------------------------------- timer
    def on_timer_done(self):
        self.busy = False
        if self.timer:
            self.timer.cancel()
            self.timer = None
        self.try_publish()


# ===================================================================== main
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

