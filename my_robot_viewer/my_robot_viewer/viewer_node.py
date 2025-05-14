#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ViewerNode(Node):
    def __init__(self):
        super().__init__('viewer_node')
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

    def joint_state_cb(self, msg: JointState):
        self.get_logger().info(f"Got joints: {msg.name} → {msg.position}")

def main(args=None):
    rclpy.init(args=args)
    node = ViewerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

