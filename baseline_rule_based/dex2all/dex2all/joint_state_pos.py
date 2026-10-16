#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class JointStatePosPrinter(Node):
    def __init__(self):
        super().__init__('joint_state_pos_printer')

        # QoS tuned for joint_states from hardware drivers (best-effort / volatile)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(JointState, '/joint_states', self._cb, qos)

    def _cb(self, msg: JointState):
        # Print only the positions as plain numbers
        if msg.position:
            print(' '.join(f'{p:.6f}' for p in msg.position), flush=True)


def main():
    rclpy.init()
    node = JointStatePosPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()