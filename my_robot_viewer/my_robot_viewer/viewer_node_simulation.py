#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class RelayJointState(Node):
    def __init__(self):
        super().__init__('relay_joint_state')
        # 1) Publisher on the real joint_states topic (or whatever your controller expects)
        self.pub = self.create_publisher(
            JointState,
            '/joint_states',    # change to the topic you need
            10
        )
        # 2) Subscriber to the fake source
        self.sub = self.create_subscription(
            JointState,
            '/fake_joint_states',
            self.joint_state_cb,
            10
        )

    def joint_state_cb(self, msg: JointState):
        # 3) Optionally stamp the header with 'now' so controllers know it's fresh
        msg.header.stamp = self.get_clock().now().to_msg()
        # 4) Republish
        self.pub.publish(msg)
        self.get_logger().debug(f"Relayed {len(msg.name)} joints")

def main(args=None):
    rclpy.init(args=args)
    node = RelayJointState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

